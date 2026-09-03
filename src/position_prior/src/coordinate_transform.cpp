/**
 * @file coordinate_transform.cpp
 * @brief world(x,z) ↔ field ↔ canonical 的位置与速度转换实现。
 *
 * 算术公式统一委托给 rm_field/field_geometry.hpp；本类只保留 PositionPrior
 * 需要的“有限值/边界/队伍合法性”校验语义。canonical 固定红方视角：蓝方目标
 * 通过场地中心对称映射到同一学习方向。
 */
#include "position_prior/coordinate_transform.hpp"

#include <rm_field/field_geometry.hpp>

#include <cmath>

namespace position_prior {

CoordinateTransform::CoordinateTransform(
    double field_length,
    double field_width,
    bool world_z_toward_blue)
    : field_length_(field_length),
      field_width_(field_width),
      world_z_toward_blue_(world_z_toward_blue) {}

bool CoordinateTransform::valid_field(const Point2d& field) const {
    return std::isfinite(field.x) && std::isfinite(field.y) &&
           field.x >= 0.0 && field.x <= field_length_ &&
           field.y >= 0.0 && field.y <= field_width_;
}

bool CoordinateTransform::valid_team(int team_id) const {
    return team_id == rm_field::kTeamRed || team_id == rm_field::kTeamBlue;
}

std::optional<Point2d> CoordinateTransform::world_to_field(const Point2d& world) const {
    if (!std::isfinite(world.x) || !std::isfinite(world.y)) {
        return std::nullopt;
    }
    // world 参数约定为 (world_x, world_z)；field.x 由 world_z 平移得到，
    // world_z_toward_blue_ 决定正方向；field.y 始终由 world_x 平移得到。
    Point2d field;
    rm_field::world_to_field(
        world.x, world.y, world_z_toward_blue_,
        field_length_, field_width_, field.x, field.y);
    if (!valid_field(field)) {
        return std::nullopt;
    }
    return field;
}

std::optional<Point2d> CoordinateTransform::field_to_world(const Point2d& field) const {
    if (!valid_field(field)) {
        return std::nullopt;
    }
    Point2d world;
    rm_field::field_to_world(
        field.x, field.y, world_z_toward_blue_,
        field_length_, field_width_, world.x, world.y);
    return world;
}

std::optional<Point2d> CoordinateTransform::field_to_canonical(
    int team_id,
    const Point2d& field) const {
    if (!valid_team(team_id) || !valid_field(field)) {
        return std::nullopt;
    }
    // 离线模型固定 canonical_team=red：红方 field 保持不变，蓝方按场地中心
    // 对称，使两方相同战术方向落入同一模型坐标语义。
    Point2d canonical;
    rm_field::field_to_canonical(
        field.x, field.y, team_id,
        field_length_, field_width_, canonical.x, canonical.y);
    return canonical;
}

std::optional<Point2d> CoordinateTransform::canonical_to_field(
    int team_id,
    const Point2d& canonical) const {
    return field_to_canonical(team_id, canonical);
}

std::optional<Point2d> CoordinateTransform::world_to_canonical(
    int team_id,
    const Point2d& world) const {
    const auto field = world_to_field(world);
    return field ? field_to_canonical(team_id, *field) : std::nullopt;
}

std::optional<Point2d> CoordinateTransform::canonical_to_world(
    int team_id,
    const Point2d& canonical) const {
    const auto field = canonical_to_field(team_id, canonical);
    return field ? field_to_world(*field) : std::nullopt;
}

Point2d CoordinateTransform::world_velocity_to_field(
    const Point2d& world_velocity) const {
    return Point2d{
        rm_field::world_velocity_to_field_x(
            world_velocity.x, world_velocity.y, world_z_toward_blue_),
        rm_field::world_velocity_to_field_y(world_velocity.x, world_velocity.y)};
}

Point2d CoordinateTransform::field_velocity_to_world(
    const Point2d& field_velocity) const {
    return Point2d{
        rm_field::field_velocity_to_world_x(field_velocity.x, field_velocity.y),
        rm_field::field_velocity_to_world_z(
            field_velocity.x, field_velocity.y, world_z_toward_blue_)};
}

std::optional<Point2d> CoordinateTransform::field_velocity_to_canonical(
    int team_id,
    const Point2d& field_velocity) const {
    if (!valid_team(team_id) || !std::isfinite(field_velocity.x) ||
        !std::isfinite(field_velocity.y)) {
        return std::nullopt;
    }
    if (team_id == rm_field::kTeamBlue) {
        return Point2d{-field_velocity.x, -field_velocity.y};
    }
    return field_velocity;
}

std::optional<Point2d> CoordinateTransform::canonical_velocity_to_field(
    int team_id,
    const Point2d& canonical_velocity) const {
    return field_velocity_to_canonical(team_id, canonical_velocity);
}

}  // namespace position_prior
