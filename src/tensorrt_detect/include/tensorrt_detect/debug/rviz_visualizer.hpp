/**
 * @file rviz_visualizer.hpp
 * @brief RViz2 调试旁路发布器的接口：把已有算法结果转成 MarkerArray 与静态 TF。
 *
 * 所有方法都是"读入快照 → 构造可视化消息"的单向流程，不持有或回写任何
 * 算法状态；动态 topic 无订阅者时相关入口直接短路，不影响主流程性能。
 */
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
    int marker_key = 0;           // 帧内唯一键（检测下标），复用为文本 Marker id
    int class_id = 0;
    int team_id = 0;
    float confidence = 0.0f;
    bool is_dead = false;
    bool tracker_measurement = false;    // 该落点进入 Tracker 作为测量
    bool negative_measurement = false;   // 死亡装甲板落点 = 确认无目标的负样本
    cv::Point3f ray_endpoint;            // 射线与场地平面/目标的交点
    cv::Point3f measurement;             // 实际被选中进入下游的 world 坐标
};

/** 统一控制可视化子系统；所有字段只影响 debug publisher。 */
struct RvizVisualizerOptions {
    // 发布层开关：pose_layer 覆盖相机标定与投影调试，observer_layer 覆盖
    // 场地静态场景与 tracker/guesser 动态图层；两者互不影响
    bool pose_layer = false;
    bool observer_layer = false;

    // 静态要素开关
    bool mesh = true;              // 场地三角面 Mesh
    bool camera = true;            // 相机本体立方体 + 光轴箭头
    bool fov = true;               // 由内参反投影出的 FOV 四棱锥线框
    bool blind_zones = true;       // 盲区边界/面积/标签
    bool nav_grid = true;          // NavGrid 可达/不可达/障碍薄片

    // 动态要素开关（逐帧发布）
    bool rays = true;              // 相机原点 → 全部射线终点
    bool ray_hits = true;          // 全部射线终点散点
    bool measurements = true;      // 本帧实际选中的落点 + 文本
    bool tracks = true;            // Tracker 目标球体
    bool trajectories = true;      // 目标历史轨迹折线
    bool velocity = true;          // 速度箭头
    bool covariance = true;        // xz 平面 2σ 不确定度椭圆
    bool guess_candidates = true;  // prior 候选球体与主猜点

    // topic 与坐标系
    std::string world_frame = "world";
    std::string camera_frame = "camera_link";
    std::string static_topic = "/radar/rviz/static";
    std::string pose_topic = "/radar/rviz/pose";
    std::string tracker_topic = "/radar/rviz/tracker";
    std::string guesser_topic = "/radar/rviz/guesser";

    // 调参
    std::size_t trajectory_length = 50;      // 轨迹保留点数
    double velocity_scale_seconds = 1.0;     // 速度箭头的时间放大倍率
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
    // 四个 publisher 按 layer 开关创建，未创建的保持空指针
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr static_pub_;   // 静态场景（transient_local）
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pose_pub_;     // 投影射线/落点
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tracker_pub_;  // /world_targets 直出
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr guesser_pub_;  // /prior_predictions 直出
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;  // 标定时广播 world->camera_link

    // setCameraCalibration 时拷贝的自持标定数据，与原 PoseSolver 生命周期解耦
    cv::Mat camera_to_world_rotation_;
    cv::Point3d camera_origin_world_{0.0, 0.0, 0.0};
    bool camera_calibrated_ = false;
    // track_id → 历史点队列；只保留当前存活目标的轨迹
    std::unordered_map<int, std::deque<cv::Point3f>> trajectories_;
};

}  // namespace tensorrt_detect::debug
