#include "position_prior/coordinate_transform.hpp"
#include "position_prior/observation_confirmation.hpp"
#include "position_prior/position_prior_model.hpp"
#include "position_prior/prior_gate.hpp"
#include "position_prior/prior_lifecycle.hpp"
#include "position_prior/team_selection.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tensorrt_detect_msgs/msg/prior_candidate.hpp>
#include <tensorrt_detect_msgs/msg/prior_prediction.hpp>
#include <tensorrt_detect_msgs/msg/prior_prediction_array.hpp>
#include <tensorrt_detect_msgs/msg/world_target_array.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace position_prior {

namespace {

constexpr std::int64_t NS_PER_SECOND = 1000000000LL;

std::int64_t time_to_ns(const builtin_interfaces::msg::Time& time) {
    return static_cast<std::int64_t>(time.sec) * NS_PER_SECOND + time.nanosec;
}

builtin_interfaces::msg::Time ns_to_time(std::int64_t timestamp_ns) {
    builtin_interfaces::msg::Time result;
    if (timestamp_ns <= 0) {
        return result;
    }
    result.sec = static_cast<std::int32_t>(timestamp_ns / NS_PER_SECOND);
    result.nanosec = static_cast<std::uint32_t>(timestamp_ns % NS_PER_SECOND);
    return result;
}

std::string role_for_class(int class_id) {
    switch (class_id) {
        case 2: return "hero";
        case 3: return "engineer";
        case 4: return "infantry3";
        case 5: return "infantry4";
        case 6: return "sentry";
        default: return {};
    }
}

}  // namespace

class PositionPriorNode : public rclcpp::Node {
public:
    PositionPriorNode() : Node("position_prior_node") {
        declare_parameter<std::string>("input_topic", "/world_targets");
        declare_parameter<std::string>("output_topic", "/prior_predictions");
        declare_parameter<std::string>("model_path", "");
        declare_parameter<std::string>("model_sha256", "");
        declare_parameter<std::string>("context", "all_phase");
        declare_parameter<double>("guess_after_s", 2.0);
        declare_parameter<int>("observation_confirm_count", 3);
        declare_parameter<double>("observation_confirm_radius_m", 0.8);
        declare_parameter<double>("observation_confirm_max_gap_s", 0.35);
        declare_parameter<int>("query_top_k", 16);
        declare_parameter<int>("output_top_k", 5);
        declare_parameter<std::vector<std::string>>(
            "enabled_roles", std::vector<std::string>{});
        declare_parameter<double>("max_speed_mps", 5.0);
        declare_parameter<double>("reachability_margin_m", 0.5);
        declare_parameter<double>("max_guess_distance_m", 6.0);
        declare_parameter<double>("distance_preference_sigma_m", 3.0);
        declare_parameter<double>("minimum_confidence", 0.05);
        declare_parameter<bool>("initial_flip_team", false);
        declare_parameter<std::vector<double>>(
            "blocked_regions_canonical", std::vector<double>{});
        declare_parameter<std::string>("shadow_log_path", "");
        declare_parameter<double>("log_interval_s", 0.5);

        input_topic_ = get_parameter("input_topic").as_string();
        output_topic_ = get_parameter("output_topic").as_string();
        context_ = get_parameter("context").as_string();
        guess_after_s_ = std::max(0.0, get_parameter("guess_after_s").as_double());
        observation_confirmation_config_.required_count = static_cast<std::size_t>(
            std::max(1, static_cast<int>(
                get_parameter("observation_confirm_count").as_int())));
        observation_confirmation_config_.cluster_radius_m = std::max(
            0.0, get_parameter("observation_confirm_radius_m").as_double());
        observation_confirmation_config_.maximum_gap_s = std::max(
            0.0, get_parameter("observation_confirm_max_gap_s").as_double());
        query_top_k_ = std::max(
            1, static_cast<int>(get_parameter("query_top_k").as_int()));
        log_interval_s_ = std::max(0.0, get_parameter("log_interval_s").as_double());

        const auto configured_roles = get_parameter("enabled_roles").as_string_array();
        all_roles_enabled_ = configured_roles.empty();
        for (const auto& role : configured_roles) {
            if (role == "hero" || role == "engineer" || role == "infantry3" ||
                role == "infantry4" || role == "sentry") {
                enabled_roles_.insert(role);
            } else {
                RCLCPP_WARN(get_logger(),
                    "忽略未知 enabled_roles 项: %s", role.c_str());
            }
        }
        if (all_roles_enabled_) {
            RCLCPP_INFO(get_logger(), "位置先验启用兵种: all");
        } else {
            std::ostringstream roles_text;
            bool first = true;
            for (const auto& role : enabled_roles_) {
                if (!first) roles_text << ',';
                roles_text << role;
                first = false;
            }
            RCLCPP_INFO(get_logger(), "位置先验启用兵种: %s",
                roles_text.str().empty() ? "none" : roles_text.str().c_str());
        }

        flip_team_ = get_parameter("initial_flip_team").as_bool();
        transform_.set_world_z_toward_blue(!flip_team_);
        RCLCPP_INFO(get_logger(), "位置先验仅服务敌方: 我方=%s 敌方=%s",
            own_team_for_view(flip_team_) == TEAM_RED ? "red" : "blue",
            opponent_team_for_view(flip_team_) == TEAM_RED ? "red" : "blue");

        PriorGateConfig gate_config;
        gate_config.max_speed_mps = std::max(0.1, get_parameter("max_speed_mps").as_double());
        gate_config.reachability_margin_m =
            std::max(0.0, get_parameter("reachability_margin_m").as_double());
        gate_config.max_guess_distance_m =
            std::max(0.0, get_parameter("max_guess_distance_m").as_double());
        gate_config.distance_preference_sigma_m =
            std::max(0.0, get_parameter("distance_preference_sigma_m").as_double());
        gate_config.minimum_confidence =
            std::clamp(get_parameter("minimum_confidence").as_double(), 0.0, 1.0);
        gate_config.output_top_k = static_cast<std::size_t>(
            std::max(1, static_cast<int>(get_parameter("output_top_k").as_int())));

        const auto blocked = get_parameter("blocked_regions_canonical").as_double_array();
        if (blocked.size() % 4 != 0) {
            RCLCPP_WARN(get_logger(),
                "blocked_regions_canonical 元素数不是 4 的倍数，将忽略尾部元素");
        }
        for (std::size_t i = 0; i + 3 < blocked.size(); i += 4) {
            Rect2d region{blocked[i], blocked[i + 1], blocked[i + 2], blocked[i + 3]};
            if (region.min_x > region.max_x) std::swap(region.min_x, region.max_x);
            if (region.min_y > region.max_y) std::swap(region.min_y, region.max_y);
            gate_config.blocked_regions_canonical.push_back(region);
        }
        gate_ = PriorGate(gate_config);

        const std::string model_path = get_parameter("model_path").as_string();
        const std::string expected_hash = get_parameter("model_sha256").as_string();
        try {
            model_.load(model_path, expected_hash);
            model_enabled_ = true;
            model_status_ = "loaded:" + model_.map_id() + ":v" +
                std::to_string(model_.model_version());
            RCLCPP_INFO(get_logger(),
                "位置先验模型加载完成: %s sha256=%s roles ready, horizons=%zu",
                model_path.c_str(), model_.model_sha256().c_str(),
                model_.horizons_seconds().size());
        } catch (const std::exception& error) {
            model_enabled_ = false;
            model_status_ = error.what();
            RCLCPP_ERROR(get_logger(),
                "位置先验模型加载失败，shadow mode 安全禁用: %s", error.what());
        }

        open_log(get_parameter("shadow_log_path").as_string());

        publisher_ = create_publisher<tensorrt_detect_msgs::msg::PriorPredictionArray>(
            output_topic_, rclcpp::QoS(10).best_effort());
        target_subscription_ =
            create_subscription<tensorrt_detect_msgs::msg::WorldTargetArray>(
                input_topic_, rclcpp::QoS(10).best_effort(),
                std::bind(&PositionPriorNode::targets_callback, this, std::placeholders::_1));
        flip_subscription_ = create_subscription<std_msgs::msg::Bool>(
            "/flip_team", rclcpp::QoS(1).reliable(),
            [this](const std_msgs::msg::Bool::ConstSharedPtr message) {
                if (flip_team_ != message->data) {
                    flip_team_ = message->data;
                    transform_.set_world_z_toward_blue(!flip_team_);
                    caches_.clear();
                    observation_confirmations_.clear();
                    RCLCPP_WARN(get_logger(),
                        "场地方向切换，已清空 position prior 观测缓存；我方=%s 敌方=%s",
                        own_team_for_view(flip_team_) == TEAM_RED ? "red" : "blue",
                        opponent_team_for_view(flip_team_) == TEAM_RED ? "red" : "blue");
                }
            });
    }

private:
    struct TargetCache {
        bool has_observation = false;
        int slot_idx = -1;
        int track_id = -1;
        int team_id = 0;
        int role_class_id = -1;
        std::int64_t last_observed_ns = 0;
        Point2d last_world;
        Point2d last_field;
        Point2d last_canonical;
        Point2d canonical_velocity;
        Point2d tracker_world;
        double last_tracking_confidence = 0.0;
        double last_position_covariance_trace = 0.0;
        bool had_prediction = false;
        Point2d last_prediction_field;
        Point2d last_prediction_world;
        double last_prediction_confidence = 0.0;
        int last_fallback_level = 0;
        double last_prediction_lost_s = 0.0;
        std::int64_t last_log_ns = 0;
    };

    void open_log(const std::string& path) {
        if (path.empty()) {
            return;
        }
        try {
            const std::filesystem::path log_path(path);
            if (!log_path.parent_path().empty()) {
                std::filesystem::create_directories(log_path.parent_path());
            }
            const bool write_header = !std::filesystem::exists(log_path) ||
                std::filesystem::file_size(log_path) == 0;
            log_stream_.open(path, std::ios::app);
            if (log_stream_ && write_header) {
                log_stream_ << "event,timestamp_ns,slot_idx,track_id,team_id,role_class_id,"
                    "last_world_x,last_world_z,lost_duration_s,tracker_world_x,tracker_world_z,"
                    "prior_world_x,prior_world_z,prior_confidence,fallback_level,"
                    "reacquired_world_x,reacquired_world_z,final_error_m\n";
                log_stream_.flush();
            }
        } catch (const std::exception& error) {
            RCLCPP_WARN(get_logger(), "无法打开 shadow log: %s", error.what());
        }
    }

    void log_prediction(
        const std::string& event,
        std::int64_t timestamp_ns,
        const TargetCache& cache,
        const Point2d& prior_world,
        double confidence,
        int fallback_level,
        const std::optional<Point2d>& reacquired_world = std::nullopt,
        double final_error_m = -1.0) {
        if (!log_stream_) return;
        log_stream_ << event << ',' << timestamp_ns << ',' << cache.slot_idx << ','
            << cache.track_id << ',' << cache.team_id << ',' << cache.role_class_id << ','
            << std::fixed << std::setprecision(6)
            << cache.last_world.x << ',' << cache.last_world.y << ','
            << cache.last_prediction_lost_s << ','
            << cache.tracker_world.x << ',' << cache.tracker_world.y << ','
            << prior_world.x << ',' << prior_world.y << ',' << confidence << ','
            << fallback_level << ',';
        if (reacquired_world) {
            log_stream_ << reacquired_world->x << ',' << reacquired_world->y << ','
                << final_error_m;
        } else {
            log_stream_ << ",,";
        }
        log_stream_ << '\n';
        log_stream_.flush();
    }

    void handle_observation(
        int slot_idx,
        const tensorrt_detect_msgs::msg::WorldTarget& target,
        std::int64_t now_ns) {
        auto& cache = caches_[slot_idx];
        const Point2d observed_world{target.world_x, target.world_z};
        const auto observed_field = transform_.world_to_field(observed_world);
        const auto observed_canonical = transform_.world_to_canonical(
            target.team_id, observed_world);
        if (!observed_field || !observed_canonical) {
            return;
        }

        if (cache.has_observation && cache.had_prediction) {
            const double final_error = std::hypot(
                observed_field->x - cache.last_prediction_field.x,
                observed_field->y - cache.last_prediction_field.y);
            log_prediction(
                "REACQUIRED", now_ns, cache, cache.last_prediction_world,
                cache.last_prediction_confidence, cache.last_fallback_level,
                observed_world, final_error);
        }

        cache = TargetCache{};
        cache.has_observation = true;
        cache.slot_idx = slot_idx;
        cache.track_id = target.track_id;
        cache.team_id = target.team_id;
        cache.role_class_id = target.class_id;
        cache.last_observed_ns = time_to_ns(target.last_observed_time);
        if (cache.last_observed_ns <= 0) cache.last_observed_ns = now_ns;
        cache.last_world = observed_world;
        cache.last_field = *observed_field;
        cache.last_canonical = *observed_canonical;
        cache.tracker_world = observed_world;
        const Point2d world_velocity{target.velocity_x, target.velocity_z};
        const auto canonical_velocity = transform_.field_velocity_to_canonical(
            target.team_id, transform_.world_velocity_to_field(world_velocity));
        if (canonical_velocity) cache.canonical_velocity = *canonical_velocity;
        cache.last_tracking_confidence = std::clamp(
            static_cast<double>(target.tracking_confidence), 0.0, 1.0);
        if (target.covariance_valid) {
            cache.last_position_covariance_trace = std::max(
                0.0, target.state_covariance[0] + target.state_covariance[5]);
        }
    }

    tensorrt_detect_msgs::msg::PriorPrediction make_prediction(
        int slot_idx,
        const tensorrt_detect_msgs::msg::WorldTarget& current_target,
        TargetCache& cache,
        std::int64_t now_ns) {
        using Prediction = tensorrt_detect_msgs::msg::PriorPrediction;
        Prediction message;
        message.slot_idx = slot_idx;
        message.track_id = cache.track_id;
        message.team_id = cache.team_id;
        message.role_class_id = cache.role_class_id;
        message.last_observed_time = ns_to_time(cache.last_observed_ns);
        message.last_world_x = cache.last_world.x;
        message.last_world_z = cache.last_world.y;

        if (!cache.has_observation) {
            message.rejection_code = Prediction::REJECTION_NO_OBSERVATION;
            message.rejection_reason = "no_reliable_observation";
            return message;
        }
        const double lost_duration_s = static_cast<double>(
            std::max<std::int64_t>(0, now_ns - cache.last_observed_ns)) * 1e-9;
        message.lost_duration_s = lost_duration_s;
        if (lost_duration_s < guess_after_s_) {
            message.rejection_code = Prediction::REJECTION_TOO_EARLY;
            message.rejection_reason = "waiting_for_prior_window";
            return message;
        }
        const std::string role = role_for_class(cache.role_class_id);
        if (role.empty() || !model_.supports_role(role)) {
            message.rejection_code = Prediction::REJECTION_UNSUPPORTED_ROLE;
            message.rejection_reason = "unsupported_role";
            return message;
        }

        if (current_target.track_id == cache.track_id &&
            current_target.position_source ==
                tensorrt_detect_msgs::msg::WorldTarget::POSITION_PREDICTED) {
            cache.tracker_world = Point2d{current_target.world_x, current_target.world_z};
        }
        message.tracker_world_x = cache.tracker_world.x;
        message.tracker_world_z = cache.tracker_world.y;

        const int horizon = model_.nearest_horizon(lost_duration_s);
        message.horizon_seconds = horizon;
        const auto distribution = model_.query(
            role, cache.last_canonical, horizon, context_,
            static_cast<std::size_t>(query_top_k_));
        if (!distribution.valid) {
            message.rejection_code = Prediction::REJECTION_NO_DISTRIBUTION;
            message.rejection_reason = distribution.error;
            return message;
        }

        const double covariance_reliability =
            1.0 / std::sqrt(1.0 + cache.last_position_covariance_trace);
        const double base_confidence =
            cache.last_tracking_confidence * covariance_reliability;
        const auto gated = gate_.apply(
            distribution, cache.last_canonical, cache.canonical_velocity,
            lost_duration_s, base_confidence);

        message.normalized_entropy = distribution.normalized_entropy;
        message.reachable_probability_mass = gated.reachable_probability_mass;
        message.fallback_level = static_cast<std::uint8_t>(distribution.fallback_level);
        message.sample_count = distribution.sample_count;
        message.motion_gated = gated.motion_gated;
        message.prior_confidence = gated.confidence;

        for (const auto& candidate : gated.candidates) {
            tensorrt_detect_msgs::msg::PriorCandidate output;
            output.grid_index = static_cast<std::uint32_t>(candidate.prior.grid_index);
            output.prior_probability = candidate.prior.probability;
            output.fused_probability = candidate.fused_probability;
            output.canonical_x = candidate.prior.canonical.x;
            output.canonical_y = candidate.prior.canonical.y;
            output.reachable = candidate.reachable;
            output.blocked = candidate.blocked;
            output.distance_from_last_m = candidate.distance_from_last_m;
            const auto field = transform_.canonical_to_field(
                cache.team_id, candidate.prior.canonical);
            const auto world = transform_.canonical_to_world(
                cache.team_id, candidate.prior.canonical);
            if (field) {
                output.field_x = field->x;
                output.field_y = field->y;
            }
            if (world) {
                output.world_x = world->x;
                output.world_z = world->y;
            }
            message.candidates.push_back(output);
        }

        if (!gated.valid) {
            message.rejection_code = gated.rejection_reason == "no_reachable_candidate"
                ? Prediction::REJECTION_UNREACHABLE
                : Prediction::REJECTION_LOW_CONFIDENCE;
            message.rejection_reason = gated.rejection_reason;
            return message;
        }

        const auto predicted_field = transform_.canonical_to_field(
            cache.team_id, gated.predicted_canonical);
        const auto predicted_world = transform_.canonical_to_world(
            cache.team_id, gated.predicted_canonical);
        if (!predicted_field || !predicted_world) {
            message.rejection_code = Prediction::REJECTION_OUT_OF_FIELD;
            message.rejection_reason = "inverse_coordinate_transform_failed";
            return message;
        }

        message.valid = true;
        message.rejection_code = Prediction::REJECTION_NONE;
        message.rejection_reason.clear();
        message.prior_canonical_x = gated.predicted_canonical.x;
        message.prior_canonical_y = gated.predicted_canonical.y;
        message.prior_field_x = predicted_field->x;
        message.prior_field_y = predicted_field->y;
        message.prior_world_x = predicted_world->x;
        message.prior_world_z = predicted_world->y;

        cache.had_prediction = true;
        cache.last_prediction_field = *predicted_field;
        cache.last_prediction_world = *predicted_world;
        cache.last_prediction_confidence = gated.confidence;
        cache.last_fallback_level = static_cast<int>(distribution.fallback_level);
        cache.last_prediction_lost_s = lost_duration_s;
        if (now_ns - cache.last_log_ns >=
            static_cast<std::int64_t>(log_interval_s_ * NS_PER_SECOND)) {
            log_prediction(
                "PREDICTION", now_ns, cache, *predicted_world,
                gated.confidence, cache.last_fallback_level);
            cache.last_log_ns = now_ns;
        }
        return message;
    }

    void targets_callback(
        const tensorrt_detect_msgs::msg::WorldTargetArray::ConstSharedPtr input) {
        auto output = std::make_unique<tensorrt_detect_msgs::msg::PriorPredictionArray>();
        output->header = input->header;
        output->model_enabled = model_enabled_;
        output->model_status = model_status_;

        std::int64_t now_ns = time_to_ns(input->header.stamp);
        if (now_ns <= 0) now_ns = get_clock()->now().nanoseconds();
        if (last_input_time_ns_ > 0 && now_ns < last_input_time_ns_) {
            caches_.clear();
            observation_confirmations_.clear();
            RCLCPP_WARN(get_logger(), "输入时间倒退，已清空 position prior 缓存");
        }
        last_input_time_ns_ = now_ns;

        if (!model_enabled_) {
            publisher_->publish(std::move(output));
            return;
        }

        using Prediction = tensorrt_detect_msgs::msg::PriorPrediction;
        // 即使上游异常地产生重复槽位，输出仍按“阵营 + 兵种”强制唯一。
        std::map<std::int64_t, Prediction> unique_predictions;
        const std::size_t slot_count = std::min<std::size_t>(10, input->targets.size());
        for (std::size_t index = 0; index < slot_count; ++index) {
            const auto& target = input->targets[index];
            // 猜点只服务敌方。主动清除己方/未知阵营的旧缓存，避免配置切换、
            // 异常上游消息或切换回调时序让己方目标继续进入先验链路。
            if (!is_opponent_team(target.team_id, flip_team_)) {
                caches_.erase(static_cast<int>(index));
                observation_confirmations_.erase(static_cast<int>(index));
                continue;
            }
            const std::string target_role = role_for_class(target.class_id);
            const bool role_enabled = all_roles_enabled_ ||
                enabled_roles_.find(target_role) != enabled_roles_.end();
            const bool observation_candidate = target.observed &&
                target.position_source ==
                    tensorrt_detect_msgs::msg::WorldTarget::POSITION_TRACKED &&
                target.team_id != 0 && !target_role.empty() && role_enabled;
            bool reliable_observation = false;
            if (observation_candidate) {
                auto [confirmation_it, inserted] = observation_confirmations_.try_emplace(
                    static_cast<int>(index), observation_confirmation_config_);
                (void)inserted;
                reliable_observation = confirmation_it->second.observe(
                    Point2d{target.world_x, target.world_z}, now_ns);
            }

            double lost_duration_s = 0.0;
            const auto existing_cache = caches_.find(static_cast<int>(index));
            if (existing_cache != caches_.end() && existing_cache->second.has_observation) {
                lost_duration_s = static_cast<double>(std::max<std::int64_t>(
                    0, now_ns - existing_cache->second.last_observed_ns)) * 1e-9;
            }
            const auto cache_action = decide_prior_cache_action(PriorLifecycleSample{
                    target.is_dead,
                    reliable_observation,
                    target.tracking_state ==
                            tensorrt_detect_msgs::msg::WorldTarget::TRACKING_DEAD ||
                        target.tracking_state ==
                            tensorrt_detect_msgs::msg::WorldTarget::TRACKING_INVALID,
                    lost_duration_s});

            if (cache_action == PriorCacheAction::CLEAR_CONFIRMED_DEAD ||
                cache_action == PriorCacheAction::CLEAR_INVALID_TIME) {
                caches_.erase(static_cast<int>(index));
                observation_confirmations_.erase(static_cast<int>(index));
                continue;
            }
            if (cache_action == PriorCacheAction::UPDATE_OBSERVATION) {
                handle_observation(static_cast<int>(index), target, now_ns);
                continue;
            }

            auto cache_it = caches_.find(static_cast<int>(index));
            if (cache_it == caches_.end() || !cache_it->second.has_observation) {
                continue;
            }
            auto prediction = make_prediction(
                static_cast<int>(index), target, cache_it->second, now_ns);
            // 进入过丢失窗口后才发布拒绝项，避免稳定识别时制造无意义消息。
            if (prediction.lost_duration_s > 0.0f) {
                const std::int64_t semantic_key =
                    static_cast<std::int64_t>(prediction.team_id) * 100LL +
                    prediction.role_class_id;
                auto existing = unique_predictions.find(semantic_key);
                const bool replace = existing == unique_predictions.end() ||
                    (prediction.valid && !existing->second.valid) ||
                    (prediction.valid == existing->second.valid &&
                     time_to_ns(prediction.last_observed_time) >
                         time_to_ns(existing->second.last_observed_time)) ||
                    (prediction.valid == existing->second.valid &&
                     time_to_ns(prediction.last_observed_time) ==
                         time_to_ns(existing->second.last_observed_time) &&
                     prediction.prior_confidence > existing->second.prior_confidence);
                if (replace) {
                    unique_predictions[semantic_key] = std::move(prediction);
                }
            }
        }

        output->predictions.reserve(unique_predictions.size());
        for (auto& [semantic_key, prediction] : unique_predictions) {
            (void)semantic_key;
            output->predictions.push_back(std::move(prediction));
        }

        publisher_->publish(std::move(output));
    }

    std::string input_topic_;
    std::string output_topic_;
    std::string context_;
    double guess_after_s_ = 2.0;
    int query_top_k_ = 16;
    double log_interval_s_ = 0.5;
    bool flip_team_ = false;
    bool all_roles_enabled_ = true;
    bool model_enabled_ = false;
    std::string model_status_ = "not_loaded";
    std::int64_t last_input_time_ns_ = 0;

    CoordinateTransform transform_;
    PositionPriorModel model_;
    PriorGate gate_;
    ObservationConfirmationConfig observation_confirmation_config_;
    std::unordered_map<int, TargetCache> caches_;
    std::unordered_map<int, ObservationConfirmation> observation_confirmations_;
    std::unordered_set<std::string> enabled_roles_;
    std::ofstream log_stream_;

    rclcpp::Subscription<tensorrt_detect_msgs::msg::WorldTargetArray>::SharedPtr
        target_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flip_subscription_;
    rclcpp::Publisher<tensorrt_detect_msgs::msg::PriorPredictionArray>::SharedPtr publisher_;
};

}  // namespace position_prior

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<position_prior::PositionPriorNode>());
    rclcpp::shutdown();
    return 0;
}
