#include <radar27_interfaces/msg/match_state.hpp>
#include <radar27_interfaces/msg/projection_debug_array.hpp>
#include <radar27_interfaces/msg/calibration_state.hpp>
/**
 * @file rviz_debug_node.cpp
 * @brief 只读订阅现有输出并发布 RViz MarkerArray 的纯旁路 component。
 *
 * 该节点不参与任何检测/跟踪算法：只订阅 /world_targets、/prior_predictions 和
 * /flip_team，把已有结果交给 RvizVisualizer 转成 Marker。静态场景（Mesh/盲区/
 * NavGrid）在启动和阵营切换时重发；rviz_debug_enabled=false 时不创建任何
 * publisher/subscription，进程保持空转。
 */
#include <radar27_visualization/rviz_visualizer.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/bool.hpp>
#include <radar27_interfaces/msg/prior_prediction_array.hpp>
#include <radar27_interfaces/msg/world_target_array.hpp>

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

        radar27_visualization::debug::RvizVisualizerOptions visualizer_options;
        visualizer_options.world_z_toward_blue=declare_parameter<bool>("world_z_toward_blue",true);
        visualizer_options.observer_layer = true;
        visualizer_options.pose_layer = false;
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

        auto pose_options=visualizer_options;
        pose_options.pose_layer=true; pose_options.observer_layer=false;
        pose_options.camera=declare_parameter<bool>("camera",true);
        pose_options.fov=declare_parameter<bool>("fov",true);
        pose_options.rays=declare_parameter<bool>("rays",true);
        pose_options.ray_hits=declare_parameter<bool>("ray_hits",true);
        pose_options.measurements=declare_parameter<bool>("measurements",true);
        pose_options.camera_frame=declare_parameter<std::string>("camera_frame","camera_link");
        pose_options.pose_topic=declare_parameter<std::string>("pose_topic","/radar/rviz/pose");
        pose_visualizer_=std::make_unique<radar27_visualization::debug::RvizVisualizer>(*this, pose_options);
        visualizer_ = std::make_unique<radar27_visualization::debug::RvizVisualizer>(
            *this, std::move(visualizer_options));
        mesh_path_ = get_parameter("mesh_path").as_string();
        navgrid_path_ = get_parameter("navgrid_path").as_string();
        navgrid_role_ = get_parameter("navgrid_role").as_string();
        blind_zone_paths_ = get_parameter("blind_zone_paths").as_string_array();
        flip_team_ = get_parameter("initial_flip_team").as_bool();
        // 静态场景先按初始阵营发一次（transient_local 保证晚启动的 RViz 也能收到）
        publish_static_scene();

        // 阵营切换是低频控制信号，用 reliable 保证不丢；变化时整个静态场景重发
        flip_team_sub_ = create_subscription<radar27_interfaces::msg::MatchState>(
            "/match_state", rclcpp::QoS(1).reliable().transient_local(),
            [this](const radar27_interfaces::msg::MatchState::ConstSharedPtr message) {
                if (flip_team_ == (message->own_team == 1)) return;
                flip_team_ = message->own_team == 1;
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
            radar27_interfaces::msg::WorldTargetArray>(
            world_targets_topic, rclcpp::QoS(10).best_effort(),
            [this](const radar27_interfaces::msg::WorldTargetArray::ConstSharedPtr message) {
                visualizer_->publishWorldTargets(*message);
            });
        prior_predictions_sub_ = create_subscription<
            radar27_interfaces::msg::PriorPredictionArray>(
            prior_predictions_topic, rclcpp::QoS(10).best_effort(),
            [this](const radar27_interfaces::msg::PriorPredictionArray::ConstSharedPtr message) {
                visualizer_->publishPriorPredictions(*message);
            });

        calibration_sub_=create_subscription<radar27_interfaces::msg::CalibrationState>("/calibration_state",rclcpp::QoS(1).reliable().transient_local(),[this](radar27_interfaces::msg::CalibrationState::ConstSharedPtr m){
            cv::Mat r(3,3,CV_64F),t(3,1,CV_64F),k(3,3,CV_64F);
            for(int i=0;i<9;++i){r.at<double>(i/3,i%3)=m->rotation[i];k.at<double>(i/3,i%3)=m->camera_matrix[i];}
            for(int i=0;i<3;++i)t.at<double>(i)=m->translation[i];
            pose_visualizer_->setCameraCalibration(r,t,k,m->image_width,m->image_height);
        });
        projection_sub_=create_subscription<radar27_interfaces::msg::ProjectionDebugArray>("/projection_debug",rclcpp::QoS(1).best_effort(),[this](radar27_interfaces::msg::ProjectionDebugArray::ConstSharedPtr m){
            std::vector<radar27_visualization::debug::PoseDebugSample> samples;
            for(const auto& d:m->samples){
                radar27_visualization::debug::PoseDebugSample s;
                s.marker_key=d.marker_key;s.class_id=d.class_id;s.team_id=d.team_id;s.confidence=d.confidence;
                s.is_dead=d.is_dead;s.tracker_measurement=d.tracker_measurement;s.negative_measurement=d.negative_measurement;
                s.ray_endpoint={d.ray_endpoint[0],d.ray_endpoint[1],d.ray_endpoint[2]};
                s.measurement={d.measurement[0],d.measurement[1],d.measurement[2]};samples.push_back(s);
            }
            pose_visualizer_->publishPoseFrame(m->header,samples);
        });
        RCLCPP_INFO(get_logger(),
            "RViz 旁路已启用: targets=%s prior=%s",
            world_targets_topic.c_str(), prior_predictions_topic.c_str());
    }

private:
    rclcpp::Subscription<radar27_interfaces::msg::CalibrationState>::SharedPtr calibration_sub_;
    rclcpp::Subscription<radar27_interfaces::msg::ProjectionDebugArray>::SharedPtr projection_sub_;
    /** 用当前阵营重发静态场景（Mesh/盲区/NavGrid）。 */
    void publish_static_scene() {
        visualizer_->publishStaticScene(
            mesh_path_, navgrid_path_, navgrid_role_, blind_zone_paths_, flip_team_);
    }

    std::unique_ptr<radar27_visualization::debug::RvizVisualizer> pose_visualizer_;
    std::unique_ptr<radar27_visualization::debug::RvizVisualizer> visualizer_;  // Marker 组装与发布
    // 静态资源配置：路径在启动后不变，阵营切换只影响渲染朝向
    std::string mesh_path_;
    std::string navgrid_path_;
    std::string navgrid_role_;
    std::vector<std::string> blind_zone_paths_;
    bool flip_team_ = false;
    rclcpp::Subscription<radar27_interfaces::msg::MatchState>::SharedPtr flip_team_sub_;   // /flip_team 控制信号
    rclcpp::Subscription<radar27_interfaces::msg::WorldTargetArray>::SharedPtr
        world_targets_sub_;                                               // → publishWorldTargets
    rclcpp::Subscription<radar27_interfaces::msg::PriorPredictionArray>::SharedPtr
        prior_predictions_sub_;                                           // → publishPriorPredictions
};

RCLCPP_COMPONENTS_REGISTER_NODE(RvizDebugNode)

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RvizDebugNode>());
    rclcpp::shutdown();
    return 0;
}
