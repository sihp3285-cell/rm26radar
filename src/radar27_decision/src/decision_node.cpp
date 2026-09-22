#include <radar27_decision/map_analyzer.hpp>
#include <radar27_interfaces/msg/map_tactics.hpp>
#include <radar27_interfaces/msg/match_state.hpp>
#include <radar27_interfaces/msg/radar_map.hpp>
#include <radar27_interfaces/msg/world_target_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <cmath>
#include <stdexcept>

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
        maps_ = create_publisher<radar27_interfaces::msg::RadarMap>("/radar_map", 10);
        tactics_ = create_publisher<radar27_interfaces::msg::MapTactics>("/map_tactics", 10);
        state_ = create_subscription<radar27_interfaces::msg::MatchState>(
            "/match_state", rclcpp::QoS(1).reliable().transient_local(),
            [this](radar27_interfaces::msg::MatchState::ConstSharedPtr message) {
                analyzer_.setTeamByFlip(message->own_team == robot_id::RED);
            });
        sub_ = create_subscription<radar27_interfaces::msg::WorldTargetArray>(
            "/world_targets", rclcpp::QoS(10).best_effort(),
            [this](radar27_interfaces::msg::WorldTargetArray::ConstSharedPtr message) {
                publish(*message);
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

        analyzer_.evaluate(message.targets);
        radar27_interfaces::msg::MapTactics tactics;
        tactics.header = message.header;
        tactics.engineer_on_island = analyzer_.engineer_on_island();
        tactics.opponent_attack = analyzer_.opponent_attack();
        tactics.our_attack = analyzer_.our_attack();
        tactics.opponent_near_fortress = analyzer_.opponent_near_fortress();
        tactics_->publish(tactics);
    }

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
