/**
 * @file rviz_debug_node.cpp
 * @brief 只读订阅现有输出并发布 RViz MarkerArray 的纯旁路 component。
 *
 * 该节点不参与任何检测/跟踪算法：只订阅 /world_targets、/prior_predictions 和
 * /flip_team，把已有结果交给 RvizVisualizer 转成 Marker。静态场景（Mesh/盲区/
 * NavGrid）在启动和阵营切换时重发；rviz_debug_enabled=false 时不创建任何
 * publisher/subscription，进程保持空转。
 */
#include "tensorrt_detect/debug/rviz_visualizer.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tensorrt_detect_msgs/msg/prior_prediction_array.hpp>
#include <tensorrt_detect_msgs/msg/world_target_array.hpp>

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

/** RViz 调试旁路节点：订阅侧全只读，无任何写回主流程的 publisher。 */
class RvizDebugNode : public rclcpp::Node {
public:
    explicit RvizDebugNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("rviz_debug_node", options) {
        // ──────── 1. 参数声明 ────────
        declare_parameter<bool>("rviz_debug_enabled", true);
        declare_parameter<std::string>("world_targets_topic", "/world_targets");
        declare_parameter<std::string>("prior_predictions_topic", "/prior_predictions");
        declare_parameter<std::string>("world_frame", "world");
        declare_parameter<std::string>("static_topic", "/radar/rviz/static");
        declare_parameter<std::string>("tracker_topic", "/radar/rviz/tracker");
        declare_parameter<std::string>("guesser_topic", "/radar/rviz/guesser");
        // 静态资源路径：空串表示不加载对应要素
        declare_parameter<std::string>("mesh_path", "");
        declare_parameter<std::string>("navgrid_path", "");
        declare_parameter<std::string>("navgrid_role", "hero");
        declare_parameter<std::vector<std::string>>(
            "blind_zone_paths", std::vector<std::string>{});
        declare_parameter<bool>("initial_flip_team", false);
        declare_parameter<int>("trajectory_length", 50);
        declare_parameter<double>("velocity_scale_seconds", 1.0);

        // 各图层开关：全部默认开启，可单独关闭以减轻 RViz 渲染压力
        declare_parameter<bool>("mesh", true);
        declare_parameter<bool>("field_grid", true);
        declare_parameter<bool>("tracks", true);
        declare_parameter<bool>("trajectories", true);
        declare_parameter<bool>("velocity", true);
        declare_parameter<bool>("covariance", true);
        declare_parameter<bool>("measurement_covariance", true);
        declare_parameter<bool>("guess_candidates", true);
        declare_parameter<bool>("blind_zones", true);
        declare_parameter<bool>("nav_grid", true);

        // ──────── 2. 参数 → Visualizer 配置 ────────
        if (!get_parameter("rviz_debug_enabled").as_bool()) {
            // 关闭时不创建任何 publisher/subscription，节点空转，对主流程零影响
            RCLCPP_INFO(get_logger(),
                "RViz debug 已关闭：不创建可视化 publisher/subscription");
            return;
        }

        tensorrt_detect::debug::RvizVisualizerOptions visualizer_options;
        visualizer_options.observer_layer = true;
        visualizer_options.world_frame = get_parameter("world_frame").as_string();
        visualizer_options.static_topic = get_parameter("static_topic").as_string();
        visualizer_options.tracker_topic = get_parameter("tracker_topic").as_string();
        visualizer_options.guesser_topic = get_parameter("guesser_topic").as_string();
        // 轨迹长度与速度倍率做下限保护，避免负值/零值产生退化渲染
        visualizer_options.trajectory_length = static_cast<std::size_t>(std::max(
            1, static_cast<int>(get_parameter("trajectory_length").as_int())));
        visualizer_options.velocity_scale_seconds = std::max(
            0.0, get_parameter("velocity_scale_seconds").as_double());
        visualizer_options.mesh = get_parameter("mesh").as_bool();
        visualizer_options.field_grid = get_parameter("field_grid").as_bool();
        visualizer_options.tracks = get_parameter("tracks").as_bool();
        visualizer_options.trajectories = get_parameter("trajectories").as_bool();
        visualizer_options.velocity = get_parameter("velocity").as_bool();
        visualizer_options.covariance = get_parameter("covariance").as_bool();
        visualizer_options.measurement_covariance =
            get_parameter("measurement_covariance").as_bool();
        visualizer_options.guess_candidates =
            get_parameter("guess_candidates").as_bool();
        visualizer_options.blind_zones = get_parameter("blind_zones").as_bool();
        visualizer_options.nav_grid = get_parameter("nav_grid").as_bool();

        visualizer_ = std::make_unique<tensorrt_detect::debug::RvizVisualizer>(
            *this, std::move(visualizer_options));
        mesh_path_ = get_parameter("mesh_path").as_string();
        navgrid_path_ = get_parameter("navgrid_path").as_string();
        navgrid_role_ = get_parameter("navgrid_role").as_string();
        blind_zone_paths_ = get_parameter("blind_zone_paths").as_string_array();
        flip_team_ = get_parameter("initial_flip_team").as_bool();
        // 静态场景先按初始阵营发一次（transient_local 保证晚启动的 RViz 也能收到）
        publish_static_scene();

        // 阵营切换是低频控制信号，用 reliable 保证不丢；变化时整个静态场景重发
        flip_team_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/flip_team", rclcpp::QoS(1).reliable(),
            [this](const std_msgs::msg::Bool::ConstSharedPtr message) {
                if (flip_team_ == message->data) return;
                flip_team_ = message->data;
                publish_static_scene();
                RCLCPP_INFO(get_logger(),
                    "RViz 静态 Guesser 场景已切换: 我方=%s 敌方=%s",
                    flip_team_ ? "red" : "blue",
                    flip_team_ ? "blue" : "red");
            });

        // ──────── 3. 动态数据订阅 ────────
        // /world_targets 与 /prior_predictions 都是高频调试数据，
        // best_effort 允许丢帧换取低延迟，depth=10 防止 RViz 消费慢时积压
        const std::string world_targets_topic =
            get_parameter("world_targets_topic").as_string();
        const std::string prior_predictions_topic =
            get_parameter("prior_predictions_topic").as_string();
        world_targets_sub_ = create_subscription<
            tensorrt_detect_msgs::msg::WorldTargetArray>(
            world_targets_topic, rclcpp::QoS(10).best_effort(),
            [this](const tensorrt_detect_msgs::msg::WorldTargetArray::ConstSharedPtr message) {
                visualizer_->publishWorldTargets(*message);
            });
        prior_predictions_sub_ = create_subscription<
            tensorrt_detect_msgs::msg::PriorPredictionArray>(
            prior_predictions_topic, rclcpp::QoS(10).best_effort(),
            [this](const tensorrt_detect_msgs::msg::PriorPredictionArray::ConstSharedPtr message) {
                visualizer_->publishPriorPredictions(*message);
            });

        RCLCPP_INFO(get_logger(),
            "RViz 旁路已启用: targets=%s prior=%s",
            world_targets_topic.c_str(), prior_predictions_topic.c_str());
    }

private:
    /** 用当前阵营重发静态场景（Mesh/盲区/NavGrid）。 */
    void publish_static_scene() {
        visualizer_->publishStaticScene(
            mesh_path_, navgrid_path_, navgrid_role_, blind_zone_paths_, flip_team_);
    }

    std::unique_ptr<tensorrt_detect::debug::RvizVisualizer> visualizer_;  // Marker 组装与发布
    // 静态资源配置：路径在启动后不变，阵营切换只影响渲染朝向
    std::string mesh_path_;
    std::string navgrid_path_;
    std::string navgrid_role_;
    std::vector<std::string> blind_zone_paths_;
    bool flip_team_ = false;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flip_team_sub_;   // /flip_team 控制信号
    rclcpp::Subscription<tensorrt_detect_msgs::msg::WorldTargetArray>::SharedPtr
        world_targets_sub_;                                               // → publishWorldTargets
    rclcpp::Subscription<tensorrt_detect_msgs::msg::PriorPredictionArray>::SharedPtr
        prior_predictions_sub_;                                           // → publishPriorPredictions
};

RCLCPP_COMPONENTS_REGISTER_NODE(RvizDebugNode)

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RvizDebugNode>());
    rclcpp::shutdown();
    return 0;
}
