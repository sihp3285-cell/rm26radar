/**
 * @file field_geometry.hpp
 * @brief world(x,z) ↔ field ↔ 红方 canonical 坐标变换的唯一实现。
 *
 * 约定（与 position_prior/coordinate_transform 及 rviz 静态场景相同）：
 *  - world 参数为 (world_x, world_z)，地面 X-Z 平面，单位米；
 *  - field 是 28m×15m 场地矩形坐标，field_x ∈ [0,28]、field_y ∈ [0,15]；
 *  - canonical 固定红方视角；蓝方目标按场地中心对称，使模型/盲区/NavGrid
 *    始终使用红方视角数据；
 *  - world_z_toward_blue=true 表示 world_z 正方向指向蓝方基地（field_x 随之
 *    递增）；false 表示 world_z 正方向指向红方基地。
 * 函数为纯算术，不做范围/队伍合法性校验——调用方（CoordinateTransform 等）
 * 保留自己的校验语义。
 */
#pragma once

#include "rm_field/robot_class.hpp"

namespace rm_field {

inline constexpr double kDefaultFieldLength = 28.0;  // field_x 范围（米）
inline constexpr double kDefaultFieldWidth  = 15.0;  // field_y 范围（米）

/** world(x,z) → field(x,y)。 */
inline void world_to_field(
    double world_x, double world_z, bool world_z_toward_blue,
    double field_length, double field_width,
    double& field_x, double& field_y) {
    field_x = world_z_toward_blue
        ? world_z + field_length / 2.0
        : field_length / 2.0 - world_z;
    field_y = world_x + field_width / 2.0;
}

/** field(x,y) → world(x,z)。 */
inline void field_to_world(
    double field_x, double field_y, bool world_z_toward_blue,
    double field_length, double field_width,
    double& world_x, double& world_z) {
    world_x = field_y - field_width / 2.0;
    world_z = world_z_toward_blue
        ? field_x - field_length / 2.0
        : field_length / 2.0 - field_x;
}

/** field → canonical：红方不变，蓝方中心对称。 */
inline void field_to_canonical(
    double field_x, double field_y, int team_id,
    double field_length, double field_width,
    double& canonical_x, double& canonical_y) {
    if (team_id == kTeamBlue) {
        canonical_x = field_length - field_x;
        canonical_y = field_width - field_y;
    } else {
        canonical_x = field_x;
        canonical_y = field_y;
    }
}

/** canonical → field（蓝方中心对称的逆变换，与 field_to_canonical 同构）。 */
inline void canonical_to_field(
    double canonical_x, double canonical_y, int team_id,
    double field_length, double field_width,
    double& field_x, double& field_y) {
    field_to_canonical(canonical_x, canonical_y, team_id,
                       field_length, field_width, field_x, field_y);
}

/** canonical → world(x,z) 组合变换。 */
inline void canonical_to_world(
    double canonical_x, double canonical_y, int team_id,
    bool world_z_toward_blue, double field_length, double field_width,
    double& world_x, double& world_z) {
    double field_x = 0.0, field_y = 0.0;
    canonical_to_field(canonical_x, canonical_y, team_id,
                       field_length, field_width, field_x, field_y);
    field_to_world(field_x, field_y, world_z_toward_blue,
                   field_length, field_width, world_x, world_z);
}

/** world(x,z) → canonical 组合变换。 */
inline void world_to_canonical(
    double world_x, double world_z, int team_id,
    bool world_z_toward_blue, double field_length, double field_width,
    double& canonical_x, double& canonical_y) {
    double field_x = 0.0, field_y = 0.0;
    world_to_field(world_x, world_z, world_z_toward_blue,
                   field_length, field_width, field_x, field_y);
    field_to_canonical(field_x, field_y, team_id,
                       field_length, field_width, canonical_x, canonical_y);
}

/** 按场地中心 (length/2, width/2) 做 180° 点对称（用于镜像盲区/沟区多边形）。 */
inline void mirror_field_point(
    double field_x, double field_y,
    double field_length, double field_width,
    double& mirrored_x, double& mirrored_y) {
    mirrored_x = field_length - field_x;
    mirrored_y = field_width - field_y;
}

/** 只按场地中线翻转 x（敌方为蓝方时盲区随相机换半场的显示变换）。 */
inline double flip_field_x(double field_x, double field_length) {
    return field_length - field_x;
}

// ── 速度（向量换轴，不施加位置平移）──
inline double world_velocity_to_field_x(
    double world_vx, double world_vz, bool world_z_toward_blue) {
    return world_z_toward_blue ? world_vz : -world_vz;
}
inline double world_velocity_to_field_y(double world_vx, double /*world_vz*/) {
    return world_vx;
}
inline double field_velocity_to_world_x(double /*field_vx*/, double field_vy) {
    return field_vy;
}
inline double field_velocity_to_world_z(
    double field_vx, double /*field_vy*/, bool world_z_toward_blue) {
    return world_z_toward_blue ? field_vx : -field_vx;
}

// ── 默认 28×15 场地重载（多数调用方使用标准场地）──
inline void world_to_field(
    double world_x, double world_z, bool world_z_toward_blue,
    double& field_x, double& field_y) {
    world_to_field(world_x, world_z, world_z_toward_blue,
                   kDefaultFieldLength, kDefaultFieldWidth, field_x, field_y);
}
inline void field_to_world(
    double field_x, double field_y, bool world_z_toward_blue,
    double& world_x, double& world_z) {
    field_to_world(field_x, field_y, world_z_toward_blue,
                   kDefaultFieldLength, kDefaultFieldWidth, world_x, world_z);
}
inline void canonical_to_world(
    double canonical_x, double canonical_y, int team_id,
    bool world_z_toward_blue, double& world_x, double& world_z) {
    canonical_to_world(canonical_x, canonical_y, team_id, world_z_toward_blue,
                       kDefaultFieldLength, kDefaultFieldWidth, world_x, world_z);
}
inline void world_to_canonical(
    double world_x, double world_z, int team_id,
    bool world_z_toward_blue, double& canonical_x, double& canonical_y) {
    world_to_canonical(world_x, world_z, team_id, world_z_toward_blue,
                       kDefaultFieldLength, kDefaultFieldWidth,
                       canonical_x, canonical_y);
}

}  // namespace rm_field
