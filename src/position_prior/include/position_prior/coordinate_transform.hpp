/**
 * @file coordinate_transform.hpp
 * @brief world、field 与红方 canonical 三套二维坐标之间的显式转换。
 *
 * world 输入取 WorldTarget 的 (world_x,world_z)；field 是统一场地矩形坐标；
 * canonical 对蓝方目标作中心对称，使模型始终看到“红方视角”的运动。位置与速度
 * 分开转换：速度是向量，不能错误地附加位置平移量。
 */
#pragma once

#include "position_prior/prior_types.hpp"

#include <optional>

namespace position_prior {

/** PositionPriorNode 独占的无状态几何转换器；非法场地范围返回 nullopt。 */
class CoordinateTransform {
public:
    /** 保存场地尺寸和 world-z 朝向约定；不持有 ROS frame 或地图资源。 */
    CoordinateTransform(
        double field_length = 28.0,
        double field_width = 15.0,
        bool world_z_toward_blue = true);

    /** 更新 world z 轴朝向约定，后续位置/速度转换立即使用新值。 */
    void set_world_z_toward_blue(bool value) { world_z_toward_blue_ = value; }
    /** 返回当前 world z 轴是否指向蓝方。 */
    bool world_z_toward_blue() const { return world_z_toward_blue_; }

    /** 返回 field/canonical 的场地长度上界，单位米。 */
    double field_length() const { return field_length_; }
    /** 返回 field/canonical 的场地宽度上界，单位米。 */
    double field_width() const { return field_width_; }

    /** 判断 field 点是否为有限值且落在闭区间场地矩形内。 */
    bool valid_field(const Point2d& field) const;
    /** 判断队伍编码是否为支持 canonical 转换的红方或蓝方。 */
    bool valid_team(int team_id) const;

    /** 将 WorldTarget 的 world(x,z) 映射到统一 field 坐标，非法输入返回 nullopt。 */
    std::optional<Point2d> world_to_field(const Point2d& world) const;
    /** 将 field 坐标逆映射为 world(x,z)，越界返回 nullopt。 */
    std::optional<Point2d> field_to_world(const Point2d& field) const;
    /** 红方保持 field，蓝方做中心对称，得到统一红方 canonical 位置。 */
    std::optional<Point2d> field_to_canonical(int team_id, const Point2d& field) const;
    /** 将指定队伍的 canonical 位置逆变换为 field 坐标。 */
    std::optional<Point2d> canonical_to_field(int team_id, const Point2d& canonical) const;
    /** 组合 world_to_field 与 field_to_canonical 的安全位置转换。 */
    std::optional<Point2d> world_to_canonical(int team_id, const Point2d& world) const;
    /** 组合 canonical_to_field 与 field_to_world 的安全位置逆转换。 */
    std::optional<Point2d> canonical_to_world(int team_id, const Point2d& canonical) const;

    /** 只旋转/换轴 world 速度向量，不施加位置平移。 */
    Point2d world_velocity_to_field(const Point2d& world_velocity) const;
    /** 将 field 速度向量逆换轴为 world(x,z) 速度，不施加平移。 */
    Point2d field_velocity_to_world(const Point2d& field_velocity) const;
    /** 按队伍方向将 field 速度变换到 canonical；队伍无效返回 nullopt。 */
    std::optional<Point2d> field_velocity_to_canonical(
        int team_id, const Point2d& field_velocity) const;
    /** 将 canonical 速度逆变换到 field；队伍无效返回 nullopt。 */
    std::optional<Point2d> canonical_velocity_to_field(
        int team_id, const Point2d& canonical_velocity) const;

private:
    double field_length_;
    double field_width_;
    bool world_z_toward_blue_;
};

}  // namespace position_prior
