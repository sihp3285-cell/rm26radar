#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "tensorrt_detect/core/target_fusion.hpp"

// Default mutually-exclusive callback group serializes both input callbacks.
class FusionNode : public rclcpp::Node {
public:
    explicit FusionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("fusion_node", options) {
        const auto world_topic = declare_parameter<std::string>("world_input_topic", "/world_targets");
        const auto prior_topic = declare_parameter<std::string>("prior_input_topic", "/prior_predictions");
        const auto output_topic = declare_parameter<std::string>("output_topic", "/fused_targets");
        max_prior_age_s_ = declare_parameter<double>("max_prior_age_s", 0.5);
        if (!std::isfinite(max_prior_age_s_) || max_prior_age_s_ < 0.0)
            throw std::invalid_argument("max_prior_age_s must be finite and nonnegative");
        const auto qos = rclcpp::QoS(10).best_effort();
        publisher_ = create_publisher<FusedArray>(output_topic, qos);
        world_sub_ = create_subscription<WorldArray>(world_topic, qos,
            [this](WorldArray::ConstSharedPtr msg) {
                if (world_ && target_fusion::stamp_ns(msg->header.stamp) <
                    target_fusion::stamp_ns(world_->header.stamp)) prior_.reset();
                world_ = std::move(msg);
                publish();
            });
        prior_sub_ = create_subscription<PriorArray>(prior_topic, qos,
            [this](PriorArray::ConstSharedPtr msg) {
                if (prior_ && target_fusion::stamp_ns(msg->header.stamp) <
                    target_fusion::stamp_ns(prior_->header.stamp)) return;
                prior_ = std::move(msg);
                // Prior normally arrives after world; update the same snapshot promptly.
                if (world_ && target_fusion::stamp_ns(prior_->header.stamp) <=
                    target_fusion::stamp_ns(world_->header.stamp)) publish();
            });
        RCLCPP_INFO(get_logger(), "Fusion ready: %s + %s -> %s", world_topic.c_str(),
            prior_topic.c_str(), output_topic.c_str());
    }
private:
    using WorldArray = radar27_interfaces::msg::WorldTargetArray;
    using PriorArray = radar27_interfaces::msg::PriorPredictionArray;
    using FusedArray = radar27_interfaces::msg::FusedTargetArray;
    void publish() {
        publisher_->publish(target_fusion::fuse(*world_, prior_.get(), max_prior_age_s_));
    }
    double max_prior_age_s_ = 0.5;
    WorldArray::ConstSharedPtr world_;
    PriorArray::ConstSharedPtr prior_;
    rclcpp::Subscription<WorldArray>::SharedPtr world_sub_;
    rclcpp::Subscription<PriorArray>::SharedPtr prior_sub_;
    rclcpp::Publisher<FusedArray>::SharedPtr publisher_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(FusionNode)

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FusionNode>());
    rclcpp::shutdown();
    return 0;
}
