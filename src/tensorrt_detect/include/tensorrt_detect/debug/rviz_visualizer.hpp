#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <tensorrt_detect_msgs/msg/prior_prediction_array.hpp>
#include <tensorrt_detect_msgs/msg/world_target_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <opencv2/core.hpp>

#include <cstddef>
#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace tf2_ros {
class StaticTransformBroadcaster;
}

namespace tensorrt_detect::debug {

/** PoseNode 已经计算出的单个投影结果的只读调试快照。 */
struct PoseDebugSample {
    int marker_key = 0;
    int class_id = 0;
    int team_id = 0;
    float confidence = 0.0f;
    bool is_dead = false;
    bool tracker_measurement = false;
    bool negative_measurement = false;
    cv::Point3f ray_endpoint;
    cv::Point3f measurement;
};

/** 统一控制可视化子系统；所有字段只影响 debug publisher。 */
struct RvizVisualizerOptions {
    bool pose_layer = false;
    bool observer_layer = false;

    bool mesh = true;
    bool camera = true;
    bool fov = true;
    bool rays = true;
    bool ray_hits = true;
    bool measurements = true;
    bool tracks = true;
    bool trajectories = true;
    bool velocity = true;
    bool covariance = true;
    bool guess_candidates = true;
    bool blind_zones = true;
    bool nav_grid = true;

    std::string world_frame = "world";
    std::string camera_frame = "camera_link";
    std::string static_topic = "/radar/rviz/static";
    std::string pose_topic = "/radar/rviz/pose";
    std::string tracker_topic = "/radar/rviz/tracker";
    std::string guesser_topic = "/radar/rviz/guesser";
    std::size_t trajectory_length = 50;
    double velocity_scale_seconds = 1.0;
};

/**
 * RViz2 旁路发布器。它从调用方接收已经存在的结果，只构造 Marker/TF；
 * 不持有、修改或回写 Pose、Tracker、Prior、NavGrid 的算法状态。
 */
class RvizVisualizer {
public:
    RvizVisualizer(rclcpp::Node& node, RvizVisualizerOptions options);
    ~RvizVisualizer();

    /** 动态 Pose topic 有订阅者时才请求调用方复制本帧调试快照。 */
    bool wantsPoseFrame() const;

    /** 复用当前 PoseSolver 的 R/T 与 K 发布 world->camera_link、相机和 FOV。 */
    void setCameraCalibration(
        const cv::Mat& camera_to_world_rotation,
        const cv::Mat& camera_origin_world,
        const cv::Mat& camera_matrix,
        int image_width,
        int image_height);

    /** 发布本帧实际选中的投影射线、有效落点与 Tracker 输入 measurement。 */
    void publishPoseFrame(
        const std_msgs::msg::Header& header,
        const std::vector<PoseDebugSample>& samples);

    /** 从与核心相同的静态资源加载 Mesh、BlindZone 和 NavGrid 并按当前敌方发布。 */
    void publishStaticScene(
        const std::string& mesh_path,
        const std::string& navgrid_path,
        const std::string& navgrid_role,
        const std::vector<std::string>& blind_zone_paths,
        bool flip_team);

    /** 直接可视化 /world_targets 的滤波状态、速度、轨迹、状态和协方差。 */
    void publishWorldTargets(
        const tensorrt_detect_msgs::msg::WorldTargetArray& message);

    /** 直接可视化 /prior_predictions 的 Top-K、Pfused、拒绝项和主猜点。 */
    void publishPriorPredictions(
        const tensorrt_detect_msgs::msg::PriorPredictionArray& message);

private:
    rclcpp::Node& node_;
    RvizVisualizerOptions options_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr static_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tracker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr guesser_pub_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;

    cv::Mat camera_to_world_rotation_;
    cv::Point3d camera_origin_world_{0.0, 0.0, 0.0};
    bool camera_calibrated_ = false;
    std::unordered_map<int, std::deque<cv::Point3f>> trajectories_;
};

}  // namespace tensorrt_detect::debug
