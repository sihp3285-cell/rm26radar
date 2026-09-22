#include <radar27_interfaces/msg/match_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rm_field/robot_class.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <stdexcept>

// One durable state publisher serves both headless clients and the optional UI.
class MatchStateNode : public rclcpp::Node {
public:
    MatchStateNode() : Node("match_state_node") {
        state_.own_team = declare_parameter<int>("own_team", rm_field::kTeamBlue);
        if (state_.own_team != rm_field::kTeamRed && state_.own_team != rm_field::kTeamBlue) {
            throw std::invalid_argument("own_team must be red(1) or blue(2)");
        }
        publisher_ = create_publisher<radar27_interfaces::msg::MatchState>(
            "/match_state", rclcpp::QoS(1).reliable().transient_local());
        commands_ = create_subscription<std_msgs::msg::Bool>(
            "/flip_team", rclcpp::QoS(1).reliable(),
            [this](std_msgs::msg::Bool::ConstSharedPtr message) { setTeam(message->data); });
        service_ = create_service<std_srvs::srv::SetBool>(
            "/match/set_red_team",
            [this](std_srvs::srv::SetBool::Request::SharedPtr request,
                   std_srvs::srv::SetBool::Response::SharedPtr response) {
                setTeam(request->data);
                response->success = true;
                response->message = "Own team updated; world axes and display rotation unchanged";
            });
        publisher_->publish(state_);
    }

private:
    void setTeam(bool red) {
        const int team = red ? rm_field::kTeamRed : rm_field::kTeamBlue;
        if (state_.own_team != team) {
            state_.own_team = team;
            ++state_.revision;
        }
        publisher_->publish(state_);
    }

    radar27_interfaces::msg::MatchState state_;
    rclcpp::Publisher<radar27_interfaces::msg::MatchState>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr commands_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MatchStateNode>());
    rclcpp::shutdown();
}
