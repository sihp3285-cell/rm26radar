/**
 * @file rviz_visualizer.cpp
 * @brief RViz2 调试图层：把已有算法结果转成 MarkerArray 与静态 TF 的纯旁路。
 *
 * 该文件不持有也不修改 Pose/Tracker/Prior 的算法状态：只在各发布入口读取调用方
 * 传入的快照，构造 RViz 所需的 Marker（相机/FOV、投影射线与落点、场地 Mesh、
 * 盲区与 NavGrid、轨迹/速度/协方差椭圆、prior 候选）并经由独立 debug topic 发布。
 * 动态 topic 无订阅者时直接短路，静态场景用 transient_local 让晚启动的 RViz
 * 也能拿到，保证整条调试链不反向影响主流程性能。
 */
#include "tensorrt_detect/debug/rviz_visualizer.hpp"

#include "tensorrt_detect/core/robot_id.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <yaml-cpp/yaml.h>

#include <open3d/geometry/TriangleMesh.h>
#include <open3d/io/TriangleMeshIO.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>
#include <unordered_set>
#include <utility>

namespace tensorrt_detect::debug {
namespace {

using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

// ──────────────────────────────────────────────
// Marker 消息构造工具
// ──────────────────────────────────────────────

/** 构造 Point 消息的便捷函数，避免各处重复三行赋值。 */
geometry_msgs::msg::Point point(double x, double y, double z) {
    geometry_msgs::msg::Point result;
    result.x = x;
    result.y = y;
    result.z = z;
    return result;
}

/** 构造 ColorRGBA 消息的便捷函数。 */
std_msgs::msg::ColorRGBA color(float red, float green, float blue, float alpha = 1.0f) {
    std_msgs::msg::ColorRGBA result;
    result.r = red;
    result.g = green;
    result.b = blue;
    result.a = alpha;
    return result;
}

/** 红/蓝方固定主题色；未知阵营退回黄色便于肉眼发现数据错误。 */
std_msgs::msg::ColorRGBA team_color(int team_id, float alpha = 1.0f) {
    if (team_id == robot_id::RED) return color(0.95f, 0.12f, 0.10f, alpha);
    if (team_id == robot_id::BLUE) return color(0.10f, 0.35f, 1.0f, alpha);
    return color(1.0f, 0.85f, 0.10f, alpha);
}

/** 带常用默认值的 Marker 工厂：ADD 动作、单位四元数、单位缩放，ns/id 由调用方决定。 */
Marker make_marker(
    const std_msgs::msg::Header& header,
    std::string marker_namespace,
    int id,
    int type) {
    Marker marker;
    marker.header = header;
    marker.ns = std::move(marker_namespace);
    marker.id = id;
    marker.type = type;
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;
    // 缩放默认给 1：TRIANGLE_LIST 等类型不看 scale，但 rviz2 会把它应用到
    // 挂载 Marker 的 Ogre 场景节点上；缩放为 0 会得到退化节点变换，本机实测
    // 部分会话里整块 Mesh 会被静默剔除（同一条消息里的其他 Marker 正常渲染）。
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    return marker;
}

/** RViz 的 Marker 是累积的：每帧先 DELETEALL 清空上一帧，再整层重建。 */
Marker delete_all(const std_msgs::msg::Header& header) {
    Marker marker;
    marker.header = header;
    marker.action = Marker::DELETEALL;
    return marker;
}

// ──────────────────────────────────────────────
// 显示命名与状态配色
// ──────────────────────────────────────────────

/** 统一机器人显示名：前哨站/死亡装甲板特判，其余用 R/B + 编号。 */
std::string robot_label(int team_id, int class_id) {
    if (class_id == robot_id::OUTPOST) {
        return team_id == robot_id::RED ? "R-Outpost" :
            team_id == robot_id::BLUE ? "B-Outpost" : "Outpost";
    }
    if (class_id == robot_id::ARMOR) return "dead armor";
    const std::string number = robot_id::getRobotNumber(class_id);
    if (number.empty()) return "class=" + std::to_string(class_id);
    if (team_id == robot_id::RED) return "R" + number;
    if (team_id == robot_id::BLUE) return "B" + number;
    return "?" + number;
}

/** Tracker 状态枚举 → RViz 文本标签。 */
std::string tracking_state_name(std::uint8_t state) {
    using Target = tensorrt_detect_msgs::msg::WorldTarget;
    switch (state) {
        case Target::TRACKING_ACTIVE: return "ACTIVE";
        case Target::TRACKING_PREDICTED: return "PREDICTED";
        case Target::TRACKING_LOST: return "LOST";
        case Target::TRACKING_DEAD: return "DEAD";
        default: return "INVALID";
    }
}

/** Tracker 状态配色：ACTIVE 绿 / PREDICTED 青 / LOST 灰 / DEAD 深灰。 */
std_msgs::msg::ColorRGBA tracking_color(std::uint8_t state) {
    using Target = tensorrt_detect_msgs::msg::WorldTarget;
    switch (state) {
        case Target::TRACKING_ACTIVE: return color(0.10f, 0.95f, 0.30f, 0.95f);
        case Target::TRACKING_PREDICTED: return color(0.10f, 0.90f, 0.95f, 0.95f);
        case Target::TRACKING_LOST: return color(0.55f, 0.55f, 0.55f, 0.75f);
        case Target::TRACKING_DEAD: return color(0.25f, 0.25f, 0.25f, 0.65f);
        default: return color(0.8f, 0.8f, 0.8f, 0.5f);
    }
}

// ──────────────────────────────────────────────
// 几何计算工具
// ──────────────────────────────────────────────

/** 3×3 旋转矩阵 → 归一化四元数；先转 CV_64F 避免 float 精度损失。 */
geometry_msgs::msg::Quaternion quaternion_from_rotation(const cv::Mat& input) {
    cv::Mat rotation;
    input.convertTo(rotation, CV_64F);
    tf2::Matrix3x3 matrix(
        rotation.at<double>(0, 0), rotation.at<double>(0, 1), rotation.at<double>(0, 2),
        rotation.at<double>(1, 0), rotation.at<double>(1, 1), rotation.at<double>(1, 2),
        rotation.at<double>(2, 0), rotation.at<double>(2, 1), rotation.at<double>(2, 2));
    tf2::Quaternion quaternion;
    matrix.getRotation(quaternion);
    quaternion.normalize();
    geometry_msgs::msg::Quaternion result;
    result.x = quaternion.x();
    result.y = quaternion.y();
    result.z = quaternion.z();
    result.w = quaternion.w();
    return result;
}

/** 3×3 旋转矩阵作用于三维向量（rotation 须为 CV_64F）。 */
cv::Point3d rotate_vector(const cv::Mat& rotation, const cv::Point3d& value) {
    cv::Mat vector = (cv::Mat_<double>(3, 1) << value.x, value.y, value.z);
    const cv::Mat transformed = rotation * vector;
    return {transformed.at<double>(0), transformed.at<double>(1), transformed.at<double>(2)};
}

/** 投影/测量可能含 NaN 或 Inf（未命中、退化射线），绘制前必须过滤。 */
bool finite_point(const cv::Point3f& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

/**
 * 2×2 对称协方差 → 贴地 2σ 椭圆 LINE_STRIP Marker；负定/非有限输入直接跳过。
 * 滤波协方差 P 与测量噪声 R 共用同一特征分解，保证两类椭圆的几何语义一致。
 */
void add_covariance_ellipse(
    MarkerArray& output,
    const std_msgs::msg::Header& header,
    const std::string& marker_namespace,
    int id,
    double center_x,
    double center_y,
    double center_z,
    double xx,
    double xz,
    double zz,
    const std_msgs::msg::ColorRGBA& ellipse_color,
    double line_width) {
    // 2×2 对称矩阵特征值闭式解：λ = (xx+zz ± sqrt((xx-zz)²+4xz²)) / 2
    const double discriminant = std::sqrt(std::max(
        0.0, (xx - zz) * (xx - zz) + 4.0 * xz * xz));
    const double major_variance = 0.5 * (xx + zz + discriminant);
    const double minor_variance = 0.5 * (xx + zz - discriminant);
    if (!(major_variance >= 0.0 && minor_variance >= 0.0) ||
        !std::isfinite(major_variance) || !std::isfinite(minor_variance)) {
        return;
    }
    const double angle = 0.5 * std::atan2(2.0 * xz, xx - zz);
    const double major = 2.0 * std::sqrt(major_variance);   // 2σ 半轴
    const double minor = 2.0 * std::sqrt(minor_variance);
    auto ellipse = make_marker(
        header, marker_namespace, id, Marker::LINE_STRIP);
    ellipse.scale.x = line_width;
    ellipse.color = ellipse_color;
    constexpr int segments = 48;  // 48 段折线近似椭圆，<=segments 保证闭合
    for (int segment = 0; segment <= segments; ++segment) {
        const double theta = 2.0 * M_PI * segment / segments;
        const double local_x = major * std::cos(theta);
        const double local_z = minor * std::sin(theta);
        ellipse.points.push_back(point(
            center_x + local_x * std::cos(angle) -
                local_z * std::sin(angle),
            center_y,
            center_z + local_x * std::sin(angle) +
                local_z * std::cos(angle)));
    }
    output.markers.push_back(std::move(ellipse));
}

/** 静态场景用当前时刻的 world 系 header（transient_local 依赖 stamp 判定新旧）。 */
std_msgs::msg::Header static_header(
    rclcpp::Node& node,
    const std::string& frame) {
    std_msgs::msg::Header header;
    header.frame_id = frame;
    header.stamp = node.now();
    return header;
}

// ──────────────────────────────────────────────
// 盲区 YAML 加载与多边形几何
// ──────────────────────────────────────────────

/** 单个盲区多边形：文件名含 engineer 的标记为仅工程可走盲区。 */
struct BlindZonePolygon {
    std::string name;
    bool engineer_only = false;
    std::vector<std::array<double, 2>> vertices;
};

/** 读取盲区 YAML 列表：顶点不足 3 个的跳过，支持按场地中心(14,7.5)生成镜像盲区。 */
std::vector<BlindZonePolygon> load_blind_zones(
    const std::vector<std::string>& paths) {
    std::vector<BlindZonePolygon> result;
    for (const auto& path : paths) {
        if (path.empty()) continue;
        const YAML::Node root = YAML::LoadFile(path);
        const YAML::Node zones = root["blind_zones"];
        if (!zones || !zones.IsSequence()) continue;
        const bool engineer_only =
            // 约定：盲区文件名含 engineer 即标记为仅工程机器人盲区
            std::filesystem::path(path).stem().string().find("engineer") != std::string::npos;
        for (const auto& zone_node : zones) {
            const YAML::Node polygon_node = zone_node["polygon"];
            if (!polygon_node || !polygon_node.IsSequence() || polygon_node.size() < 3) {
                continue;
            }
            BlindZonePolygon zone;
            zone.name = zone_node["name"]
                ? zone_node["name"].as<std::string>()
                : std::filesystem::path(path).stem().string();
            zone.engineer_only = engineer_only;
            for (const auto& vertex : polygon_node) {
                if (vertex.IsSequence() && vertex.size() == 2) {
                    zone.vertices.push_back(
                        {vertex[0].as<double>(), vertex[1].as<double>()});
                }
            }
            if (zone.vertices.size() >= 3) result.push_back(zone);
            // mirror_centrally：按场地中心 (14, 7.5) 做 180° 点对称生成对称盲区
            const bool mirror = zone_node["mirror_centrally"] &&
                zone_node["mirror_centrally"].as<bool>();
            if (mirror && zone.vertices.size() >= 3) {
                BlindZonePolygon mirrored = zone;
                mirrored.name += "_mirrored";
                for (auto& vertex : mirrored.vertices) {
                    vertex[0] = 28.0 - vertex[0];
                    vertex[1] = 15.0 - vertex[1];
                }
                result.push_back(std::move(mirrored));
            }
        }
    }
    return result;
}

geometry_msgs::msg::Point canonical_to_world(
    double canonical_x,
    double canonical_y,
    bool flip_team,
    double height = 0.04,
    double field_length = 28.0,
    double field_width = 15.0) {
    // 与 PositionPriorNode 的 CoordinateTransform 保持同一顺序：BlindZone/NavGrid
    // 固定为红方 canonical；红方视角时敌方为蓝方，先做中心对称恢复到蓝方
    // field，再按当前场地方向转成 PoseNode 使用的 world(x,z)。
    const bool enemy_is_blue = flip_team;
    const double field_x = enemy_is_blue
        ? field_length - canonical_x : canonical_x;
    const double field_y = enemy_is_blue
        ? field_width - canonical_y : canonical_y;
    const bool world_z_toward_blue = !flip_team;
    const double world_x = field_y - field_width / 2.0;
    const double world_z = world_z_toward_blue
        ? field_x - field_length / 2.0
        : field_length / 2.0 - field_x;
    return point(world_x, height, world_z);
}

/** 鞋带公式求有向面积：>0 逆时针，<0 顺时针，用于判定耳切方向。 */
double signed_polygon_area(const BlindZonePolygon& polygon) {
    double area = 0.0;
    for (std::size_t index = 0; index < polygon.vertices.size(); ++index) {
        const auto& current = polygon.vertices[index];
        const auto& next = polygon.vertices[(index + 1) % polygon.vertices.size()];
        area += current[0] * next[1] - next[0] * current[1];
    }
    return 0.5 * area;
}

/** 三角形 abc 的有向叉积（2D）：符号表示点序方向。 */
double triangle_cross(
    const std::array<double, 2>& a,
    const std::array<double, 2>& b,
    const std::array<double, 2>& c) {
    return (b[0] - a[0]) * (c[1] - a[1]) -
        (b[1] - a[1]) * (c[0] - a[0]);
}

/** 点在三角形内（含边界，带 1e-9 容差）判定：三边叉积符号一致。 */
bool point_in_triangle(
    const std::array<double, 2>& point_value,
    const std::array<double, 2>& a,
    const std::array<double, 2>& b,
    const std::array<double, 2>& c) {
    const double first = triangle_cross(a, b, point_value);
    const double second = triangle_cross(b, c, point_value);
    const double third = triangle_cross(c, a, point_value);
    constexpr double tolerance = 1e-9;
    const bool has_negative = first < -tolerance || second < -tolerance ||
        third < -tolerance;
    const bool has_positive = first > tolerance || second > tolerance ||
        third > tolerance;
    return !(has_negative && has_positive);
}

/** Ear clipping 只把已有 polygon 拆成三角形，不改变盲区边界或算法语义。 */
std::vector<std::array<std::size_t, 3>> triangulate_polygon(
    const BlindZonePolygon& polygon) {
    std::vector<std::array<std::size_t, 3>> triangles;
    if (polygon.vertices.size() < 3) return triangles;
    std::vector<std::size_t> remaining(polygon.vertices.size());
    for (std::size_t index = 0; index < remaining.size(); ++index) {
        remaining[index] = index;
    }
    const double orientation = signed_polygon_area(polygon) >= 0.0 ? 1.0 : -1.0;
    std::size_t attempts_without_ear = 0;  // 防退化多边形死循环的护栏
    while (remaining.size() > 3 &&
           attempts_without_ear < remaining.size() * remaining.size()) {
        bool clipped = false;
        for (std::size_t index = 0; index < remaining.size(); ++index) {
            const std::size_t previous = remaining[
                (index + remaining.size() - 1) % remaining.size()];
            const std::size_t current = remaining[index];
            const std::size_t next = remaining[(index + 1) % remaining.size()];
            const auto& a = polygon.vertices[previous];
            const auto& b = polygon.vertices[current];
            const auto& c = polygon.vertices[next];
            if (orientation * triangle_cross(a, b, c) <= 1e-9) continue;  // 凹角不能作为耳
            bool contains_vertex = false;
            for (const std::size_t candidate : remaining) {
                if (candidate == previous || candidate == current || candidate == next) {
                    continue;
                }
                if (point_in_triangle(polygon.vertices[candidate], a, b, c)) {
                    contains_vertex = true;
                    break;
                }
            }
            if (contains_vertex) continue;  // 耳内包着其他顶点则不是有效耳
            triangles.push_back({previous, current, next});
            // 剪掉该耳顶点后回到循环头重新扫描
            remaining.erase(remaining.begin() + static_cast<std::ptrdiff_t>(index));
            clipped = true;
            attempts_without_ear = 0;
            break;
        }
        if (!clipped) ++attempts_without_ear;
    }
    if (remaining.size() == 3) {  // 剩余最后三个顶点直接成三角形
        triangles.push_back({remaining[0], remaining[1], remaining[2]});
    }
    return triangles;
}

}  // namespace

RvizVisualizer::RvizVisualizer(
    rclcpp::Node& node,
    RvizVisualizerOptions options)
    : node_(node), options_(std::move(options)) {
    // 静态场景只发一次，transient_local 让晚启动的 RViz 也能拿到最后一帧历史；
    // 动态图层是高频调试数据，best_effort + volatile 丢弃旧帧、不保证送达，降低开销
    const auto static_qos = rclcpp::QoS(1).reliable().transient_local();
    const auto dynamic_qos = rclcpp::QoS(1).best_effort().durability_volatile();
    if (options_.pose_layer || options_.observer_layer) {
        static_pub_ = node_.create_publisher<MarkerArray>(
            options_.static_topic, static_qos);
    }
    if (options_.pose_layer) {
        pose_pub_ = node_.create_publisher<MarkerArray>(
            options_.pose_topic, dynamic_qos);
        // 相机标定后广播 world->camera_link 静态 TF，供 RViz 摆放相机 Marker
        static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(&node_);
    }
    if (options_.observer_layer) {
        tracker_pub_ = node_.create_publisher<MarkerArray>(
            options_.tracker_topic, dynamic_qos);
        guesser_pub_ = node_.create_publisher<MarkerArray>(
            options_.guesser_topic, dynamic_qos);
    }
}

RvizVisualizer::~RvizVisualizer() = default;

bool RvizVisualizer::wantsPoseFrame() const {
    // 无订阅者或三个 pose 图层全关时，PoseNode 连快照复制都可以省掉
    return pose_pub_ && pose_pub_->get_subscription_count() > 0 &&
        (options_.rays || options_.ray_hits || options_.measurements);
}

void RvizVisualizer::setCameraCalibration(
    const cv::Mat& camera_to_world_rotation,
    const cv::Mat& camera_origin_world,
    const cv::Mat& camera_matrix,
    int image_width,
    int image_height) {
    if (!options_.pose_layer || camera_to_world_rotation.empty() ||
        camera_origin_world.empty() || camera_matrix.empty()) {
        return;
    }
    // 转 CV_64F 拷贝成自持矩阵：调用方 PoseSolver 的 Mat 生命周期与本类解耦
    camera_to_world_rotation.convertTo(camera_to_world_rotation_, CV_64F);
    cv::Mat origin;
    camera_origin_world.convertTo(origin, CV_64F);
    camera_origin_world_ = {
        origin.at<double>(0), origin.at<double>(1), origin.at<double>(2)};
    camera_calibrated_ = true;

    // world -> camera_link 静态 TF：相机位置 + 姿态与 PoseSolver 当前标定一致
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = node_.now();
    transform.header.frame_id = options_.world_frame;
    transform.child_frame_id = options_.camera_frame;
    transform.transform.translation.x = camera_origin_world_.x;
    transform.transform.translation.y = camera_origin_world_.y;
    transform.transform.translation.z = camera_origin_world_.z;
    transform.transform.rotation = quaternion_from_rotation(camera_to_world_rotation_);
    static_tf_->sendTransform(transform);

    if (!static_pub_ || (!options_.camera && !options_.fov)) return;
    const auto header = static_header(node_, options_.world_frame);
    MarkerArray output;
    if (options_.camera) {
        // 相机本体：按相机实际朝向摆放的小立方体
        auto body = make_marker(header, "radar/camera", 0, Marker::CUBE);
        body.pose.position = point(
            camera_origin_world_.x, camera_origin_world_.y, camera_origin_world_.z);
        body.pose.orientation = transform.transform.rotation;
        body.scale.x = 0.32;
        body.scale.y = 0.20;
        body.scale.z = 0.16;
        body.color = color(0.95f, 0.55f, 0.05f, 0.95f);
        output.markers.push_back(body);

        // 光轴方向 = 相机系 z+ 单位向量经 R 转到 world
        const cv::Point3d forward = rotate_vector(
            camera_to_world_rotation_, cv::Point3d(0.0, 0.0, 1.0));
        auto arrow = make_marker(header, "radar/camera", 1, Marker::ARROW);
        arrow.points.push_back(body.pose.position);
        arrow.points.push_back(point(
            camera_origin_world_.x + forward.x,
            camera_origin_world_.y + forward.y,
            camera_origin_world_.z + forward.z));
        arrow.scale.x = 0.06;
        arrow.scale.y = 0.12;
        arrow.scale.z = 0.18;
        arrow.color = color(1.0f, 0.75f, 0.10f, 1.0f);
        output.markers.push_back(std::move(arrow));
    }

    if (options_.fov && image_width > 0 && image_height > 0) {
        cv::Mat intrinsics;
        camera_matrix.convertTo(intrinsics, CV_64F);
        const double fx = intrinsics.at<double>(0, 0);
        const double fy = intrinsics.at<double>(1, 1);
        const double cx = intrinsics.at<double>(0, 2);
        const double cy = intrinsics.at<double>(1, 2);
        if (fx > 0.0 && fy > 0.0) {
            std::array<geometry_msgs::msg::Point, 4> corners;
            const std::array<std::array<double, 2>, 4> pixels{{
                {0.0, 0.0}, {static_cast<double>(image_width), 0.0},
                {static_cast<double>(image_width), static_cast<double>(image_height)},
                {0.0, static_cast<double>(image_height)}}};
            for (std::size_t i = 0; i < pixels.size(); ++i) {
                // 用内参把四个像角反投影为相机系单位方向，再转到 world
                cv::Point3d direction(
                    (pixels[i][0] - cx) / fx,
                    (pixels[i][1] - cy) / fy,
                    1.0);
                direction = rotate_vector(camera_to_world_rotation_, direction);
                const double length = std::sqrt(
                    direction.x * direction.x + direction.y * direction.y +
                    direction.z * direction.z);
                const double scale = length > 0.0 ? 3.0 / length : 0.0;  // FOV 锥显示到 3m 处
                corners[i] = point(
                    camera_origin_world_.x + direction.x * scale,
                    camera_origin_world_.y + direction.y * scale,
                    camera_origin_world_.z + direction.z * scale);
            }
            auto fov = make_marker(header, "radar/fov", 0, Marker::LINE_LIST);
            fov.scale.x = 0.025;
            fov.color = color(1.0f, 0.65f, 0.05f, 0.9f);
            const auto origin_point = point(
                camera_origin_world_.x, camera_origin_world_.y, camera_origin_world_.z);
            for (const auto& corner : corners) {
                fov.points.push_back(origin_point);
                fov.points.push_back(corner);
            }
            for (std::size_t i = 0; i < corners.size(); ++i) {
                fov.points.push_back(corners[i]);
                fov.points.push_back(corners[(i + 1) % corners.size()]);
            }
            output.markers.push_back(std::move(fov));
        }
    }
    if (!output.markers.empty()) static_pub_->publish(std::move(output));
}

void RvizVisualizer::publishPoseFrame(
    const std_msgs::msg::Header& input_header,
    const std::vector<PoseDebugSample>& samples) {
    if (!wantsPoseFrame() || !camera_calibrated_) return;
    auto header = input_header;
    header.frame_id = options_.world_frame;
    MarkerArray output;
    // 先清空上一帧全部 pose 图层，再按开关逐层重建（Marker 在 RViz 不会自动过期）
    output.markers.push_back(delete_all(header));

    if (options_.rays) {
        // 射线层：相机原点 → 每个 sample 的射线终点，检查投影方向是否正确
        auto rays = make_marker(header, "radar/rays", 0, Marker::LINE_LIST);
        rays.scale.x = 0.025;
        rays.color = color(1.0f, 0.75f, 0.05f, 0.85f);
        for (const auto& sample : samples) {
            if (!finite_point(sample.ray_endpoint)) continue;
            rays.points.push_back(point(
                camera_origin_world_.x, camera_origin_world_.y,
                camera_origin_world_.z));
            rays.points.push_back(point(
                sample.ray_endpoint.x, sample.ray_endpoint.y,
                sample.ray_endpoint.z));
        }
        output.markers.push_back(std::move(rays));
    }

    if (options_.ray_hits) {
        // 射线命中层：所有射线终点（含未选中的），用于检查遮挡/筛选逻辑
        auto hits = make_marker(header, "radar/ray_hits", 0, Marker::POINTS);
        hits.scale.x = 0.13;
        hits.scale.y = 0.13;
        for (const auto& sample : samples) {
            if (!finite_point(sample.ray_endpoint)) continue;
            hits.points.push_back(point(
                sample.ray_endpoint.x, sample.ray_endpoint.y,
                sample.ray_endpoint.z));
            hits.colors.push_back(color(1.0f, 0.25f, 0.95f, 1.0f));
        }
        output.markers.push_back(std::move(hits));
    }

    if (options_.measurements) {
        // 测量层：本帧实际选中的落点 + 文本标签；marker_key 复用检测下标保证帧内唯一
        auto measurements = make_marker(
            header, "radar/measurements", 0, Marker::POINTS);
        measurements.scale.x = 0.20;
        measurements.scale.y = 0.20;
        for (const auto& sample : samples) {
            if (!finite_point(sample.measurement)) continue;
            measurements.points.push_back(point(
                sample.measurement.x, sample.measurement.y,
                sample.measurement.z));
            // 灰 = 负样本（死亡装甲板，确认无目标），黄 = 进 Tracker 的测量，
            // 队色 = 其余（如前哨站直通项）
            measurements.colors.push_back(sample.negative_measurement
                ? color(0.35f, 0.35f, 0.35f, 0.9f)
                : sample.tracker_measurement
                    ? color(1.0f, 0.9f, 0.05f, 1.0f)
                    : team_color(sample.team_id));

            auto label = make_marker(
                header, "radar/measurement_text", sample.marker_key,
                Marker::TEXT_VIEW_FACING);
            label.pose.position = point(
                sample.measurement.x, sample.measurement.y + 0.35,
                sample.measurement.z);
            label.scale.z = 0.22;
            label.color = team_color(sample.team_id);
            std::ostringstream text;
            text << robot_label(sample.team_id, sample.class_id)
                 << " raw (" << std::fixed << std::setprecision(2)
                 << sample.measurement.x << ", " << sample.measurement.y
                 << ", " << sample.measurement.z << ")"
                 << " c=" << std::setprecision(2) << sample.confidence;
            if (sample.negative_measurement) text << " NEGATIVE";
            label.text = text.str();
            output.markers.push_back(std::move(label));
        }
        output.markers.push_back(std::move(measurements));
    }
    pose_pub_->publish(std::move(output));
}

void RvizVisualizer::publishStaticScene(
    const std::string& mesh_path,
    const std::string& navgrid_path,
    const std::string& navgrid_role,
    const std::vector<std::string>& blind_zone_paths,
    bool flip_team) {
    if (!options_.observer_layer || !static_pub_) return;
    const auto header = static_header(node_, options_.world_frame);
    MarkerArray output;

    // 场地 Mesh：一次性把三角面顶点灌进 TRIANGLE_LIST，半透明叠在 RViz 地面
    if (options_.mesh && !mesh_path.empty()) {
        open3d::geometry::TriangleMesh mesh;
        if (open3d::io::ReadTriangleMesh(mesh_path, mesh)) {
            auto marker = make_marker(header, "radar/mesh", 0, Marker::TRIANGLE_LIST);
            marker.color = color(0.48f, 0.58f, 0.65f, 0.32f);
            marker.points.reserve(mesh.triangles_.size() * 3);
            for (const auto& triangle : mesh.triangles_) {
                for (int vertex_index : triangle) {
                    const auto& vertex = mesh.vertices_.at(
                        static_cast<std::size_t>(vertex_index));
                    marker.points.push_back(point(vertex.x(), vertex.y(), vertex.z()));
                }
            }
            output.markers.push_back(std::move(marker));
        } else {
            RCLCPP_WARN(node_.get_logger(),
                "RViz 无法读取场地 Mesh: %s", mesh_path.c_str());
        }
    }

    // 场地参考网格：rviz2 自带的 Grid 显示在本机部分会话里会静默不渲染（与
    // 本项目发布的消息无关，rviz2 单独启动即可复现），网格消失后场地失去
    // 透视线参考，视觉上变成"实心"。改为随静态场景发布自己的 1m 网格，
    // 保证每次启动场地观感一致：y 取 0.0025 略高于地面 Mesh（≤0.001）、
    // 低于 NavGrid 薄片（≥0.0125），避免与两者 z-fighting。
    if (options_.field_grid) {
        auto grid = make_marker(header, "radar/field_grid", 0, Marker::LINE_LIST);
        grid.scale.x = 0.03;
        grid.color = color(0.43f, 0.45f, 0.49f, 0.45f);  // 与 rviz Grid 默认灰一致
        constexpr double grid_height = 0.0025;
        // world x ∈ [-7.5, 7.5]、z ∈ [-14, 14] 对应 15×28m 场地；步进 10（0.1m
        // 单位）即 1m 间距，整数步进避免浮点累加漂移。
        for (int x_step = -75; x_step <= 75; x_step += 10) {
            const double x = static_cast<double>(x_step) / 10.0;
            grid.points.push_back(point(x, grid_height, -14.0));
            grid.points.push_back(point(x, grid_height, 14.0));
        }
        for (int z_step = -140; z_step <= 140; z_step += 10) {
            const double z = static_cast<double>(z_step) / 10.0;
            grid.points.push_back(point(-7.5, grid_height, z));
            grid.points.push_back(point(7.5, grid_height, z));
        }
        output.markers.push_back(std::move(grid));
    }

    if (options_.blind_zones) {
        // 每个盲区画三样：闭合边界 LINE_STRIP、贴地三角化面、质心名称标签
        try {
            const auto zones = load_blind_zones(blind_zone_paths);
            int marker_id = 0;
            for (const auto& zone : zones) {
                // 盲区文件是红方 canonical；敌方为蓝方时物理盲区随相机换到对面
                // 半场，先按 x=14 轴左右翻转，再与 PositionPrior 相同的
                // canonical->field->world 顺序渲染。
                BlindZonePolygon effective_zone = zone;
                if (flip_team) {
                    for (auto& vertex : effective_zone.vertices) {
                        vertex[0] = 28.0 - vertex[0];
                    }
                }
                const auto zone_color = effective_zone.engineer_only
                    ? color(0.95f, 0.15f, 0.85f, 0.16f)
                    : color(1.0f, 0.20f, 0.05f, 0.16f);
                auto boundary = make_marker(
                    header, "radar/blind_zones/boundary", marker_id,
                    Marker::LINE_STRIP);
                boundary.scale.x = 0.07;
                boundary.color = effective_zone.engineer_only
                    ? color(0.95f, 0.15f, 0.85f, 0.95f)
                    : color(1.0f, 0.35f, 0.05f, 0.95f);
                for (const auto& vertex : effective_zone.vertices) {
                    boundary.points.push_back(canonical_to_world(
                        vertex[0], vertex[1], flip_team));
                }
                boundary.points.push_back(boundary.points.front());  // 首尾闭合
                output.markers.push_back(std::move(boundary));

                auto area = make_marker(
                    header, "radar/blind_zones/area", marker_id,
                    Marker::TRIANGLE_LIST);
                area.color = zone_color;
                // 耳切三角化后按 y=0.03 贴地渲染，避免与 RViz 地面 z-fighting
                for (const auto& triangle : triangulate_polygon(effective_zone)) {
                    for (const std::size_t vertex_index : triangle) {
                        const auto& vertex = effective_zone.vertices[vertex_index];
                        area.points.push_back(canonical_to_world(
                            vertex[0], vertex[1], flip_team, 0.03));
                    }
                }
                output.markers.push_back(std::move(area));

                auto label = make_marker(
                    header, "radar/blind_zones/text", marker_id,
                    Marker::TEXT_VIEW_FACING);
                double center_x = 0.0;
                double center_y = 0.0;
                // 顶点平均 = 多边形质心近似，够标签定位用
                for (const auto& vertex : effective_zone.vertices) {
                    center_x += vertex[0];
                    center_y += vertex[1];
                }
                center_x /= static_cast<double>(effective_zone.vertices.size());
                center_y /= static_cast<double>(effective_zone.vertices.size());
                label.pose.position = canonical_to_world(
                    center_x, center_y, flip_team, 0.35);
                label.scale.z = 0.22;
                label.color = zone_color;
                label.color.a = 1.0f;
                label.text = effective_zone.engineer_only
                    ? effective_zone.name + " [engineer]" : effective_zone.name;
                output.markers.push_back(std::move(label));
                ++marker_id;
            }
        } catch (const std::exception& error) {
            RCLCPP_WARN(node_.get_logger(),
                "RViz 盲区加载失败: %s", error.what());
        }
    }

    if (options_.nav_grid && !navgrid_path.empty()) {
        // NavGrid 渲染：每个格子一块薄立方体，按 height 与 walkable 分成三类
        try {
            const YAML::Node root = YAML::LoadFile(navgrid_path);
            const auto geometry = root["geometry"];
            const auto surface = root["surface"];
            const auto profiles = root["profiles"];
            const int rows = geometry["rows"].as<int>();
            const int columns = geometry["columns"].as<int>();
            const double resolution = geometry["resolution_m"].as<double>();
            const double field_length = geometry["field_length_m"].as<double>();
            const double field_width = geometry["field_width_m"].as<double>();
            const int invalid_height = surface["invalid_height_mm"].as<int>();
            const auto heights = surface["height_mm"].as<std::vector<int>>();
            YAML::Node role_profile = profiles[navgrid_role];
            if (!role_profile) role_profile = profiles["hero"];  // 未配置兵种回退 hero
            const auto walkable = role_profile["walkable"].as<std::vector<int>>();
            if (rows * columns != static_cast<int>(heights.size()) ||
                heights.size() != walkable.size()) {
                throw std::runtime_error("NavGrid 数组长度与 rows*columns 不一致");
            }
            auto reachable = make_marker(
                header, "radar/nav_grid/reachable", 0, Marker::CUBE_LIST);
            auto unreachable = make_marker(
                header, "radar/nav_grid/unreachable", 0, Marker::CUBE_LIST);
            auto obstacle = make_marker(
                header, "radar/nav_grid/obstacle", 0, Marker::CUBE_LIST);
            // 薄片立方体：x/z 按分辨率略缩留出格缝，y 压扁贴地
            for (Marker* marker : {&reachable, &unreachable, &obstacle}) {
                marker->scale.x = resolution * 0.86;
                marker->scale.y = 0.025;
                marker->scale.z = resolution * 0.86;
            }
            reachable.color = color(0.12f, 0.85f, 0.28f, 0.20f);
            unreachable.color = color(0.95f, 0.18f, 0.08f, 0.26f);
            obstacle.color = color(0.25f, 0.25f, 0.28f, 0.34f);
            for (int row = 0; row < rows; ++row) {
                for (int column = 0; column < columns; ++column) {
                    const std::size_t index = static_cast<std::size_t>(
                        row * columns + column);
                    const double canonical_x = (column + 0.5) * resolution;
                    const double canonical_y = (row + 0.5) * resolution;
                    const bool invalid = heights[index] == invalid_height;
                    // 格子中心 y：无效高度记为障碍薄片，否则按实际高度抬升 25mm
                    const double world_y = invalid
                        ? 0.01 : static_cast<double>(heights[index]) * 0.001 + 0.025;
                    const auto center = canonical_to_world(
                        canonical_x, canonical_y, flip_team, world_y,
                        field_length, field_width);
                    if (invalid) obstacle.points.push_back(center);
                    else if (walkable[index] != 0) reachable.points.push_back(center);
                    else unreachable.points.push_back(center);
                }
            }
            output.markers.push_back(std::move(reachable));
            output.markers.push_back(std::move(unreachable));
            output.markers.push_back(std::move(obstacle));
        } catch (const std::exception& error) {
            RCLCPP_WARN(node_.get_logger(),
                "RViz NavGrid 加载失败: %s", error.what());
        }
    }

    if (!output.markers.empty()) static_pub_->publish(std::move(output));
}

void RvizVisualizer::publishWorldTargets(
    const tensorrt_detect_msgs::msg::WorldTargetArray& message) {
    // 无 RViz 订阅时直接短路，不浪费任何 Marker 组装开销
    if (!tracker_pub_ || tracker_pub_->get_subscription_count() == 0 ||
        !options_.tracks) {
        return;
    }
    auto header = message.header;
    header.frame_id = options_.world_frame;
    MarkerArray output;
    output.markers.push_back(delete_all(header));
    std::unordered_set<int> current_track_ids;  // 本帧存活的 track_id，用于清理轨迹缓存

    for (std::size_t index = 0; index < message.targets.size(); ++index) {
        const auto& target = message.targets[index];
        // 两类目标：Tracker 正式轨迹（track_id>=0 且状态有效），
        // 或未经 Tracker 的直接测量（前哨站直通等，track_id<0 但 valid）
        const bool has_track = target.track_id >= 0 &&
            target.tracking_state != target.TRACKING_INVALID;
        const bool direct_measurement = target.track_id < 0 && target.valid;
        if (!has_track && !direct_measurement) continue;
        if (!std::isfinite(target.world_x) || !std::isfinite(target.world_y) ||
            !std::isfinite(target.world_z)) continue;

        // 有轨迹用 track_id 保证跨帧 id 稳定（RViz 才能平滑移动），
        // 无轨迹用 10000+index 避免与 track_id 冲突
        const int marker_id = has_track
            ? target.track_id
            : 10000 + static_cast<int>(index);
        if (has_track) current_track_ids.insert(target.track_id);
        auto body = make_marker(
            header,
            direct_measurement ? "radar/robots" : "radar/tracks",
            marker_id,
            Marker::SPHERE);
        body.pose.position = point(target.world_x, target.world_y + 0.16, target.world_z);
        body.scale.x = direct_measurement ? 0.30 : 0.36;
        body.scale.y = direct_measurement ? 0.30 : 0.36;
        body.scale.z = direct_measurement ? 0.30 : 0.36;
        body.color = direct_measurement
            ? team_color(target.team_id)
            : tracking_color(target.tracking_state);
        output.markers.push_back(std::move(body));

        auto label = make_marker(
            header, "radar/track_text", marker_id, Marker::TEXT_VIEW_FACING);
        label.pose.position = point(
            target.world_x, target.world_y + 0.55, target.world_z);
        label.scale.z = 0.24;
        label.color = team_color(target.team_id);
        std::ostringstream text;
        text << robot_label(target.team_id, target.class_id);
        if (has_track) {
            text << "  Track=" << target.track_id
                 << "  " << tracking_state_name(target.tracking_state)
                 << "\nobs=" << (target.observed ? "yes" : "no")
                 << " lost=" << std::fixed << std::setprecision(2)
                 << target.lost_duration_s << "s"
                 << "  q=" << target.tracking_confidence
                 << "\nstable=" << target.stable_class_id
                 << " (" << target.stable_class_conf << ")";
        }
        text << "\n(" << std::fixed << std::setprecision(2)
             << target.world_x << ", " << target.world_y << ", "
             << target.world_z << ")";
        label.text = text.str();
        output.markers.push_back(std::move(label));

        if (has_track && options_.velocity) {
            // 速度箭头：按 velocity_scale_seconds 把速度放大到可见长度，过小的画了也没意义
            const double velocity_norm = std::sqrt(
                target.velocity_x * target.velocity_x +
                target.velocity_y * target.velocity_y +
                target.velocity_z * target.velocity_z);
            if (std::isfinite(velocity_norm) && velocity_norm > 1e-3) {
                auto arrow = make_marker(
                    header, "radar/velocity", marker_id, Marker::ARROW);
                arrow.points.push_back(point(
                    target.world_x, target.world_y + 0.18, target.world_z));
                arrow.points.push_back(point(
                    target.world_x + target.velocity_x * options_.velocity_scale_seconds,
                    target.world_y + 0.18 +
                        target.velocity_y * options_.velocity_scale_seconds,
                    target.world_z + target.velocity_z * options_.velocity_scale_seconds));
                arrow.scale.x = 0.045;
                arrow.scale.y = 0.10;
                arrow.scale.z = 0.14;
                arrow.color = color(0.10f, 0.95f, 0.95f, 0.95f);
                output.markers.push_back(std::move(arrow));
            }
        }

        if (has_track && options_.trajectories) {
            auto& history = trajectories_[target.track_id];
            const cv::Point3f current(
                target.world_x, target.world_y + 0.06f, target.world_z);
            // 距上一点 <1mm 不重复记录，并把长度钳在 trajectory_length 以内
            if (history.empty() || cv::norm(current - history.back()) > 1e-3) {
                history.push_back(current);
                while (history.size() > options_.trajectory_length) history.pop_front();
            }
            auto trajectory = make_marker(
                header, "radar/trajectory", marker_id, Marker::LINE_STRIP);
            trajectory.scale.x = 0.045;
            trajectory.color = team_color(target.team_id, 0.82f);
            for (const auto& sample : history) {
                trajectory.points.push_back(point(sample.x, sample.y, sample.z));
            }
            output.markers.push_back(std::move(trajectory));
        }

        if (has_track && options_.covariance && target.covariance_valid) {
            // 协方差椭圆：kf_world 状态为 [x, z, vx, vz]，P 按 4×4 行主序平铺成
            // 16 元数组，取 [0],[1],[4],[5] 即地面 xz 平面 2×2 块，特征分解后
            // 以 2σ 半轴画 LINE_STRIP 椭圆贴地显示滤波不确定度
            add_covariance_ellipse(
                output, header, "radar/covariance", marker_id,
                target.world_x, target.world_y + 0.04, target.world_z,
                target.state_covariance[0],
                0.5 * (target.state_covariance[1] + target.state_covariance[4]),
                target.state_covariance[5],
                color(0.95f, 0.25f, 0.95f, 0.75f), 0.025);
        }

        if (has_track && options_.measurement_covariance &&
            target.measurement_covariance_valid &&
            std::isfinite(target.measurement_x) &&
            std::isfinite(target.measurement_z)) {
            // 测量噪声 R 椭圆：以最近一次被 kf_world 实际采用的原始测量为中心，
            // 白色显示像素误差→5 射线→J→R 传播出的、滤波真实消费的 R；与滤波
            // 后状态处的品红 P 椭圆并排对比，可直接观察一次更新的收敛幅度。
            // 本帧未观测时半透明，表示椭圆来自更早帧的测量。
            const double r_alpha = target.observed ? 0.95 : 0.45;
            add_covariance_ellipse(
                output, header, "radar/measurement_R", marker_id,
                target.measurement_x, target.world_y + 0.04,
                target.measurement_z,
                target.measurement_covariance[0],
                0.5 * (target.measurement_covariance[1] +
                       target.measurement_covariance[2]),
                target.measurement_covariance[3],
                color(1.0f, 1.0f, 1.0f, r_alpha), 0.03);
            // 中心点：R 椭圆彼此靠近时仍能分辨各自的测量位置
            auto center = make_marker(
                header, "radar/measurement_R/center", marker_id,
                Marker::POINTS);
            center.scale.x = 0.10;
            center.scale.y = 0.10;
            center.points.push_back(point(
                target.measurement_x, target.world_y + 0.04,
                target.measurement_z));
            center.color = color(1.0f, 1.0f, 1.0f, r_alpha);
            output.markers.push_back(std::move(center));
        }
    }

    // 轨迹缓存只保留本帧仍存活的 track_id，防止消失目标的历史点无限累积
    for (auto iterator = trajectories_.begin(); iterator != trajectories_.end();) {
        if (current_track_ids.count(iterator->first) == 0) {
            iterator = trajectories_.erase(iterator);
        } else {
            ++iterator;
        }
    }
    tracker_pub_->publish(std::move(output));
}

void RvizVisualizer::publishPriorPredictions(
    const tensorrt_detect_msgs::msg::PriorPredictionArray& message) {
    // 无 RViz 订阅时直接短路，不浪费任何 Marker 组装开销
    if (!guesser_pub_ || guesser_pub_->get_subscription_count() == 0 ||
        !options_.guess_candidates) {
        return;
    }
    auto header = message.header;
    header.frame_id = options_.world_frame;
    MarkerArray output;
    output.markers.push_back(delete_all(header));
    for (const auto& prediction : message.predictions) {
        // id 分段：每个 slot 占 100 号段（slot_idx*100 + rank），
        // 主猜点固定用 +90，不同 slot 之间不会碰撞
        const int base_id = std::max(0, prediction.slot_idx) * 100;
        for (std::size_t rank = 0; rank < prediction.candidates.size(); ++rank) {
            const auto& candidate = prediction.candidates[rank];
            const int marker_id = base_id + static_cast<int>(rank);
            const bool rejected = !candidate.reachable || candidate.blocked;
            // 球体大小随融合概率线性缩放并 clamp，视觉上概率越高球越大
            const double scale = std::clamp(
                0.13 + static_cast<double>(candidate.fused_probability) * 0.65,
                0.13, 0.65);
            auto body = make_marker(
                header,
                rejected ? "radar/guess_candidates/rejected"
                         : "radar/guess_candidates/accepted",
                marker_id,
                Marker::SPHERE);
            body.pose.position = point(
                candidate.world_x, 0.12, candidate.world_z);
            body.scale.x = scale;
            body.scale.y = scale;
            body.scale.z = scale;
            body.color = rejected
                // 红 = 被障碍遮挡，灰 = 不可达，蓝 = 候选接受
                ? candidate.blocked
                    ? color(0.85f, 0.10f, 0.08f, 0.78f)
                    : color(0.45f, 0.45f, 0.45f, 0.70f)
                : color(0.18f, 0.55f, 1.0f, 0.82f);
            output.markers.push_back(std::move(body));

            auto label = make_marker(
                header, "radar/guess_candidates/text", marker_id,
                Marker::TEXT_VIEW_FACING);
            label.pose.position = point(
                candidate.world_x, 0.45 + 0.5 * scale, candidate.world_z);
            label.scale.z = 0.20;
            label.color = rejected
                ? color(0.85f, 0.85f, 0.85f, 0.95f)
                : color(0.20f, 0.75f, 1.0f, 1.0f);
            std::ostringstream text;
            text << "#" << (rank + 1) << " "
                 << robot_label(prediction.team_id, prediction.role_class_id)
                 << "\nPfused=" << std::fixed << std::setprecision(3)
                 << candidate.fused_probability
                 << " prior=" << candidate.prior_probability
                 << "\nd=" << std::setprecision(2)
                 << candidate.distance_from_last_m << "m";
            if (candidate.from_blind_zone) text << " blind";
            if (candidate.stay_anchor) text << " stay";
            if (candidate.blocked) text << " BLOCKED";
            else if (!candidate.reachable) text << " UNREACHABLE";
            label.text = text.str();
            output.markers.push_back(std::move(label));
        }

        if (prediction.valid) {
            // 主猜点：大球 + tracker 位置到猜点的连线 + 置信度文本
            const int marker_id = base_id + 90;
            auto main = make_marker(
                header, "radar/main_guess", marker_id, Marker::SPHERE);
            main.pose.position = point(
                prediction.prior_world_x, 0.20, prediction.prior_world_z);
            main.scale.x = 0.55;
            main.scale.y = 0.55;
            main.scale.z = 0.55;
            main.color = color(1.0f, 0.15f, 0.85f, 0.95f);
            output.markers.push_back(std::move(main));

            auto link = make_marker(
                header, "radar/main_guess/link", marker_id, Marker::ARROW);
            link.points.push_back(point(
                prediction.tracker_world_x, 0.12, prediction.tracker_world_z));
            link.points.push_back(point(
                prediction.prior_world_x, 0.12, prediction.prior_world_z));
            link.scale.x = 0.05;
            link.scale.y = 0.11;
            link.scale.z = 0.15;
            link.color = color(1.0f, 0.15f, 0.85f, 0.85f);
            output.markers.push_back(std::move(link));

            auto label = make_marker(
                header, "radar/main_guess/text", marker_id,
                Marker::TEXT_VIEW_FACING);
            label.pose.position = point(
                prediction.prior_world_x, 0.78, prediction.prior_world_z);
            label.scale.z = 0.27;
            label.color = color(1.0f, 0.25f, 0.90f, 1.0f);
            std::ostringstream text;
            text << "MAIN GUESS "
                 << robot_label(prediction.team_id, prediction.role_class_id)
                 << "\nconfidence=" << std::fixed << std::setprecision(3)
                 << prediction.prior_confidence
                 << " lost=" << std::setprecision(2)
                 << prediction.lost_duration_s << "s"
                 << "\nreachable_mass=" << std::setprecision(3)
                 << prediction.reachable_probability_mass;
            label.text = text.str();
            output.markers.push_back(std::move(label));
        }
    }
    guesser_pub_->publish(std::move(output));
}

}  // namespace tensorrt_detect::debug
