#include <radar27_decision/map_analyzer.hpp>
#include <radar27_interfaces/msg/map_tactics.hpp>
#include <radar27_interfaces/msg/match_state.hpp>
#include <radar27_interfaces/msg/radar_map.hpp>
#include <radar27_interfaces/msg/world_target_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <cmath>
#include <chrono>
#include <stdexcept>
#include <string>
#include <vector>

// Structured outputs are independent of map image loading and rendering.
class DecisionNode : public rclcpp::Node {
public:
    explicit DecisionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("decision_node", options) {
        length_ = declare_parameter<double>("field_length", 28.0);
        width_ = declare_parameter<double>("field_width", 15.0);
        pixels_x_ = declare_parameter<int>("map_width", 388);
        pixels_y_ = declare_parameter<int>("map_height", 722);
        if (!std::isfinite(length_) || !std::isfinite(width_) ||
            length_ <= 0 || width_ <= 0 || pixels_x_ <= 0 || pixels_y_ <= 0) {
            throw std::invalid_argument("invalid field/map dimensions");
        }
        analyzer_.setFieldXFlip(declare_parameter<bool>("world_z_toward_blue", true));
        analyzer_.setTeamByFlip(
            declare_parameter<int>("own_team", robot_id::BLUE) == robot_id::RED);
        if (length_ != 28.0 || width_ != 15.0) throw std::invalid_argument("region data requires 28x15 field");
        const auto regions_dir = declare_parameter<std::string>("regions_dir", "");
        // 重叠归属规则先装载，loadRegions() 会校验规则里的区域名真实存在。
        analyzer_.setOverlapPriority(declare_parameter<std::vector<std::string>>(
            "region_overlap_priority", std::vector<std::string>{"gully>middle"}));
        analyzer_.loadRegions(regions_dir, declare_parameter<double>("boundary_margin_m", 0.75),
                              declare_parameter<double>("max_prediction_age_s", 1.0));
        input_timeout_s_ = declare_parameter<double>("input_timeout_s", 1.0);
        if (!std::isfinite(input_timeout_s_) || input_timeout_s_ <= 0) throw std::invalid_argument("invalid input timeout");
        timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { publishTactics(); });
        maps_ = create_publisher<radar27_interfaces::msg::RadarMap>("/radar_map", 10);
        tactics_ = create_publisher<radar27_interfaces::msg::MapTactics>("/map_tactics", rclcpp::QoS(1).reliable().transient_local());
        state_ = create_subscription<radar27_interfaces::msg::MatchState>(
            "/match_state", rclcpp::QoS(1).reliable().transient_local(),
            [this](radar27_interfaces::msg::MatchState::ConstSharedPtr message) {
                analyzer_.setTeamByFlip(message->own_team == robot_id::RED);
            });
        sub_ = create_subscription<radar27_interfaces::msg::WorldTargetArray>(
            "/world_targets", rclcpp::QoS(10).best_effort(),
            [this](radar27_interfaces::msg::WorldTargetArray::ConstSharedPtr message) {
                latest_ = message;
                last_received_ = std::chrono::steady_clock::now();
                publish(*message);
                publishTactics();
            });
    }

private:
    void publish(const radar27_interfaces::msg::WorldTargetArray& message) {
        radar27_interfaces::msg::RadarMap map;
        map.header = message.header;
        map.header.frame_id = "radar_map";
        for (const auto& target : message.targets) {
            if (!target.valid || target.is_dead || target.stable_class_conf <= 0 ||
                !std::isfinite(target.world_x) || !std::isfinite(target.world_z)) {
                continue;
            }
            int index = -1;
            if (target.stable_class_id >= robot_id::R1 && target.stable_class_id <= robot_id::R4) {
                index = target.stable_class_id - robot_id::R1;
            } else if (target.stable_class_id == robot_id::S) {
                index = 5;  // Preserve the legacy six-number wire layout (number 5 is empty).
            }
            if (index < 0) continue;
            const float x = target.world_x * pixels_x_ / width_ + pixels_x_ / 2.0;
            const float y = target.world_z * pixels_y_ / length_ + pixels_y_ / 2.0;
            if (target.team_id == robot_id::RED) {
                map.red_x[index] = x;
                map.red_y[index] = y;
            } else if (target.team_id == robot_id::BLUE) {
                map.blue_x[index] = x;
                map.blue_y[index] = y;
            }
        }
        maps_->publish(map);

    }

    void publishTactics() {
        const bool fresh = latest_ && std::chrono::duration<double>(
            std::chrono::steady_clock::now() - last_received_).count() <= input_timeout_s_;
        auto tactics = analyzer_.evaluate(fresh ? latest_->targets :
            std::vector<radar27_interfaces::msg::WorldTarget>{});
        tactics.input_fresh = fresh;
        if (latest_) {
            tactics.header = latest_->header;
            tactics.calibration_version = latest_->calibration_version;
        }
        tactics_->publish(tactics);
    }

    radar27_interfaces::msg::WorldTargetArray::ConstSharedPtr latest_;
    std::chrono::steady_clock::time_point last_received_;
    double input_timeout_s_;
    rclcpp::TimerBase::SharedPtr timer_;

    double length_, width_;
    int pixels_x_, pixels_y_;
    MapAnalyzer analyzer_;
    rclcpp::Publisher<radar27_interfaces::msg::RadarMap>::SharedPtr maps_;
    rclcpp::Publisher<radar27_interfaces::msg::MapTactics>::SharedPtr tactics_;
    rclcpp::Subscription<radar27_interfaces::msg::MatchState>::SharedPtr state_;
    rclcpp::Subscription<radar27_interfaces::msg::WorldTargetArray>::SharedPtr sub_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(DecisionNode)

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DecisionNode>());
    rclcpp::shutdown();
}
