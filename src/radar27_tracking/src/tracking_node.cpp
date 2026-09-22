#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <radar27_interfaces/msg/world_measurement_array.hpp>
#include <radar27_interfaces/msg/world_target_array.hpp>
#include <radar27_tracking/tracker.hpp>
#include <radar27_tracking/tracker_message.hpp>
#include <radar27_tracking/config.hpp>
#include <rm_field/robot_id.hpp>
#include <algorithm>
#include <cmath>
class TrackingNode : public rclcpp::Node {
public:
    explicit TrackingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("tracking_node", options) {
        const auto dir = declare_parameter<std::string>("config_dir", ".");
        const TrackingConfig config(dir);
        TrackerParams tp;
        // Track 生命周期
        tp.max_lost_time_s = config.tracker.maxLostTimeS;
        tp.max_predict_time_s = config.tracker.maxPredictTimeS;
        tp.dead_retention_time_s = config.tracker.deadRetentionTimeS;
        tp.min_hit = config.tracker.minHit;
        tp.max_tracks = config.tracker.maxTracks;
        // 物理匹配 gate
        tp.max_gate_box = config.tracker.maxGateBox;
        tp.max_gate_world = config.tracker.maxGateWorld;
        tp.kalman_gate_box = config.tracker.kalmanGateBox;
        tp.kalman_gate_world = config.tracker.kalmanGateWorld;
        tp.negative_gate_box = config.tracker.negativeGateBox;
        tp.negative_gate_world = config.tracker.negativeGateWorld;
        // Hungarian 匹配代价
        tp.w_box = config.tracker.wBox;
        tp.w_world = config.tracker.wWorld;
        tp.class_mismatch_min_penalty = config.tracker.classMismatchMinPenalty;
        tp.class_mismatch_penalty = config.tracker.classMismatchPenalty;
        // BotIdentity 身份稳定器
        tp.botIdentity = config.tracker.botIdentity;
        // 身份更新阈值
        tp.min_identity_update_conf = config.tracker.minIdentityUpdateConf;
        tp.identity_confirm_observations = config.tracker.identityConfirmObservations;
        tp.identity_switch_confirm_observations = config.tracker.identitySwitchConfirmObservations;
        tp.identity_confirm_min_time_ms = config.tracker.identityConfirmMinTimeMs;
        tp.identity_switch_confirm_min_time_ms = config.tracker.identitySwitchConfirmMinTimeMs;
        // Official slot owner 机制
        tp.slot_bind_min_conf = config.tracker.slotBindMinConf;
        tp.slot_lease_time_s = config.tracker.slotLeaseTimeS;
        tp.slot_min_stability = config.tracker.slotMinStability;
        tp.slot_max_switch_rate = config.tracker.slotMaxSwitchRate;
        tp.max_slot_jump_dist = config.tracker.maxSlotJumpDist;

        tracker_ = Tracker(tp);
        dead_target_hold_time_s_ = std::max(0.0f, config.tracker.deadTargetHoldTimeS);
        RCLCPP_INFO(this->get_logger(),
            "Tracker 参数: max_lost=%.3fs max_predict=%.3fs dead_retention=%.3fs min_hit=%d max_tracks=%d | "
            "gate: box=%.1f world=%.2f kalman_box=%.3f kalman_world=%.3f negative_box=%.1f negative_world=%.2f | cost: w_box=%.2f w_world=%.2f class_pen=[%.3f, %.3f] | "
            "identity: initial=%d switch=%d observations | slot: bind=%.2f lease=%.3fs stability=%.2f switch_rate=%.2f jump=%.2f",
            tp.max_lost_time_s, tp.max_predict_time_s, tp.dead_retention_time_s,
            tp.min_hit, tp.max_tracks,
            tp.max_gate_box, tp.max_gate_world, tp.kalman_gate_box, tp.kalman_gate_world,
            tp.negative_gate_box, tp.negative_gate_world,
            tp.w_box, tp.w_world, tp.class_mismatch_min_penalty,
            tp.class_mismatch_penalty,
            tp.identity_confirm_observations, tp.identity_switch_confirm_observations,
            tp.slot_bind_min_conf, tp.slot_lease_time_s, tp.slot_min_stability,
            tp.slot_max_switch_rate, tp.max_slot_jump_dist);
        RCLCPP_INFO(this->get_logger(),
            "BotIdentity 参数: max_history=%d purge_after_lost=%.3fs min_stable=%d decay=%.3f num_classes=%d",
            tp.botIdentity.maxHistory, tp.botIdentity.purgeAfterLostTimeS,
            tp.botIdentity.minHistoryForStable, tp.botIdentity.decay, tp.botIdentity.numClasses);


        world_pub_ = create_publisher<radar27_interfaces::msg::WorldTargetArray>(
            declare_parameter<std::string>("output_topic", "/world_targets"),
            rclcpp::QoS(10).best_effort());
        sub_ = create_subscription<radar27_interfaces::msg::WorldMeasurementArray>(
            declare_parameter<std::string>("input_topic", "/world_measurements"),
            rclcpp::QoS(10).best_effort(),
            [this](radar27_interfaces::msg::WorldMeasurementArray::ConstSharedPtr message) {
                update(message);
            });
    }

private:
    void update(const radar27_interfaces::msg::WorldMeasurementArray::ConstSharedPtr& msg) {
        if (epoch_ != msg->calibration_version) {
            tracker_.reset();
            cached_dead_targets_.clear();
            last_dead_target_observed_ns_ = 0;
            last_stamp_ns_ = 0;
            epoch_ = msg->calibration_version;
        }
        float tracker_dt = -1.0f;
        int64_t stamp_ns = rclcpp::Time(
            msg->header.stamp, this->get_clock()->get_clock_type()).nanoseconds();
        if (stamp_ns <= 0) {
            stamp_ns = this->get_clock()->now().nanoseconds();
        }
        if (last_stamp_ns_ > 0) {
            const double dt_seconds =
                static_cast<double>(stamp_ns - last_stamp_ns_) * 1e-9;
            if (!std::isfinite(dt_seconds) || dt_seconds < 0.0) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "检测时间倒退（dt=%.6f s），重置 Tracker 时间状态",
                    dt_seconds);
                tracker_.reset();

                cached_dead_targets_.clear();
                last_dead_target_observed_ns_ = 0;
            } else {
                constexpr double max_filter_dt_s = 1.0;
                tracker_dt = static_cast<float>(std::min(dt_seconds, max_filter_dt_s));
                if (dt_seconds > max_filter_dt_s) {
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(), *this->get_clock(), 5000,
                        "检测间隔 %.3f s，生命周期使用真实间隔，Kalman dt 限幅为 %.1f s",
                        dt_seconds, max_filter_dt_s);
                }
            }
        }
        last_stamp_ns_ = stamp_ns;

        // ---- 1. 解算所有检测的世界坐标，构建观测输入 ----
        // Outpost 不走 Tracker，直接透传
        // 死亡装甲板（ARMOR + is_dead）作为 Tracker 负观测，同时动态追加死亡点
        // 正常装甲板（R1~S）进入固定槽位跟踪
        std::vector<WorldMeasurement> meas;
        meas.reserve(msg->measurements.size());
        std::vector<radar27_interfaces::msg::WorldTarget> dead_targets;
        radar27_interfaces::msg::WorldTarget outpost_target;
        bool has_outpost = false;


        constexpr bool ENABLE_CLASS_MARGIN = true;

        for (size_t i = 0; i < msg->measurements.size(); ++i) {
            const auto& item = msg->measurements[i];
            if (!item.valid) continue;
            const auto& det = item.detection;
            const cv::Point2f world_pos(item.world_x, item.world_z);
            // Outpost 直接透传，不进入 Tracker
            if (det.idx == robot_id::OUTPOST) {
                outpost_target.idx      = 10;
                outpost_target.class_id = det.idx;
                outpost_target.team_id  = det.armor_color;
                outpost_target.is_dead  = det.is_dead;
                outpost_target.valid    = true;
                outpost_target.bbox_x   = det.x;
                outpost_target.bbox_y   = det.y;
                outpost_target.bbox_w   = det.width;
                outpost_target.bbox_h   = det.height;
                outpost_target.world_x  = world_pos.x;
                outpost_target.world_y  = 0.0f;
                outpost_target.world_z  = world_pos.y;
                const bool directly_observed = det.width > 0 && det.height > 0;
                tracker_message::mark_direct_measurement(
                    outpost_target, directly_observed, det.confidence);
                if (directly_observed) {
                    outpost_target.last_observed_time =
                        static_cast<builtin_interfaces::msg::Time>(
                            rclcpp::Time(stamp_ns, this->get_clock()->get_clock_type()));
                }
                has_outpost = true;
                continue;
            }

            // 死亡装甲板仍动态发布，同时作为负观测传入 Tracker。
            if (item.is_negative) {
                radar27_interfaces::msg::WorldTarget t;
                t.idx      = 11 + static_cast<int>(dead_targets.size());
                t.class_id = robot_id::ARMOR;
                t.team_id  = robot_id::UNKNOWN;
                t.is_dead  = true;
                t.valid    = true;
                t.bbox_x   = det.x;
                t.bbox_y   = det.y;
                t.bbox_w   = det.width;
                t.bbox_h   = det.height;
                t.world_x  = world_pos.x;
                t.world_y  = 0.0f;
                t.world_z  = world_pos.y;
                t.stable_class_id  = -1;
                t.stable_class_conf = 0.0f;
                tracker_message::mark_direct_measurement(t, true, det.confidence);
                t.last_observed_time =
                    static_cast<builtin_interfaces::msg::Time>(
                        rclcpp::Time(stamp_ns, this->get_clock()->get_clock_type()));
                dead_targets.push_back(t);

                WorldMeasurement negative;
                negative.class_id = robot_id::ARMOR;
                negative.team_id = robot_id::UNKNOWN;
                negative.score = det.confidence;
                negative.class_conf = det.class_conf;
                negative.class_margin = ENABLE_CLASS_MARGIN
                    ? det.class_margin
                    : 0.0f;
                negative.is_dead = true;
                negative.is_negative = true;
                negative.box = cv::Rect(det.x, det.y, det.width, det.height);
                negative.world = world_pos;
                negative.world_covariance = item.covariance;
                negative.world_covariance_valid =
                    item.covariance_valid;
                meas.push_back(negative);
                continue;
            }

            WorldMeasurement m;
            m.class_id = det.idx;
            m.team_id  = det.armor_color;
            m.score    = det.confidence;
            m.class_conf = det.class_conf;
            m.class_margin = ENABLE_CLASS_MARGIN
                ? det.class_margin
                : 0.0f;
            m.is_dead  = det.is_dead;
            m.box      = cv::Rect(det.x, det.y, det.width, det.height);
            m.world    = world_pos;  // x=world_x, y=world_z
            m.world_covariance = item.covariance;
            m.world_covariance_valid = item.covariance_valid;
            meas.push_back(m);
        }

        // 死亡装甲板不进入 Tracker；短暂漏检时保留最近结果，避免地图单帧闪烁。
        if (!dead_targets.empty()) {
            cached_dead_targets_ = dead_targets;
            last_dead_target_observed_ns_ = stamp_ns;
        } else if (!cached_dead_targets_.empty() &&
                   last_dead_target_observed_ns_ > 0 &&
                   stamp_ns >= last_dead_target_observed_ns_ &&
                   static_cast<double>(stamp_ns - last_dead_target_observed_ns_) * 1e-9 <=
                       dead_target_hold_time_s_) {
            dead_targets = cached_dead_targets_;
        } else {
            cached_dead_targets_.clear();
            last_dead_target_observed_ns_ = 0;
        }

        // ---- 2. Tracker 更新（正常观测 + 死亡装甲板负观测；不含 Outpost）----
        tracker_.update(meas, tracker_dt, stamp_ns);

        // ---- 3. 固定槽位 + Outpost + 动态死亡装甲板 发布 ----
        auto world_msg = std::make_unique<radar27_interfaces::msg::WorldTargetArray>();
        world_msg->header = msg->header;
        world_msg->calibration_version = msg->calibration_version;
        // 0-9: Tracker official slots（含 track→slot 映射 + 仲裁）；10: Outpost 透传
        world_msg->targets.resize(11);

        // 批量获取 10 个 official slot 输出（已含 track→slot 映射 + 仲裁）
        auto slot_outputs = tracker_.get_outputs();

        int valid_count = 0;
        for (int i = 0; i < Tracker::NUM_SLOTS; ++i) {
            const auto& slot = slot_outputs[i];
            auto& target = world_msg->targets[i];
            tracker_message::fill_world_target(i, slot, target);
            if (slot.valid) valid_count++;
        }

        // Outpost 直接放到索引 10
        if (has_outpost) {
            world_msg->targets[10] = outpost_target;
            valid_count++;
        } else {
            auto& target = world_msg->targets[10];
            target.idx      = 10;
            target.class_id = robot_id::OUTPOST;
            target.team_id  = robot_id::UNKNOWN;
            target.valid    = false;
            tracker_message::mark_direct_measurement(target, false, 0.0f);
        }

        // 动态追加死亡装甲板
        for (const auto& dt : dead_targets) {
            world_msg->targets.push_back(dt);
        }

        world_pub_->publish(std::move(world_msg));


    }

    Tracker tracker_;
    uint64_t epoch_ = 0;
    int64_t last_stamp_ns_ = 0;
    int64_t last_dead_target_observed_ns_ = 0;
    float dead_target_hold_time_s_ = 0.1f;
    std::vector<radar27_interfaces::msg::WorldTarget> cached_dead_targets_;
    rclcpp::Subscription<radar27_interfaces::msg::WorldMeasurementArray>::SharedPtr sub_;
    rclcpp::Publisher<radar27_interfaces::msg::WorldTargetArray>::SharedPtr world_pub_;
};
RCLCPP_COMPONENTS_REGISTER_NODE(TrackingNode)
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrackingNode>());
    rclcpp::shutdown();
}
