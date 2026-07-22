#include "position_prior/coordinate_transform.hpp"

#include <cmath>

namespace position_prior {

namespace {
constexpr int TEAM_RED = 1;
constexpr int TEAM_BLUE = 2;
}

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
    return team_id == TEAM_RED || team_id == TEAM_BLUE;
}

std::optional<Point2d> CoordinateTransform::world_to_field(const Point2d& world) const {
    if (!std::isfinite(world.x) || !std::isfinite(world.y)) {
        return std::nullopt;
    }
    Point2d field;
    field.x = world_z_toward_blue_
        ? world.y + field_length_ / 2.0
        : field_length_ / 2.0 - world.y;
    field.y = world.x + field_width_ / 2.0;
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
    world.x = field.y - field_width_ / 2.0;
    world.y = world_z_toward_blue_
        ? field.x - field_length_ / 2.0
        : field_length_ / 2.0 - field.x;
    return world;
}

std::optional<Point2d> CoordinateTransform::field_to_canonical(
    int team_id,
    const Point2d& field) const {
    if (!valid_team(team_id) || !valid_field(field)) {
        return std::nullopt;
    }
    if (team_id == TEAM_BLUE) {
        return Point2d{field_length_ - field.x, field_width_ - field.y};
    }
    return field;
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
        world_z_toward_blue_ ? world_velocity.y : -world_velocity.y,
        world_velocity.x};
}

Point2d CoordinateTransform::field_velocity_to_world(
    const Point2d& field_velocity) const {
    return Point2d{
        field_velocity.y,
        world_z_toward_blue_ ? field_velocity.x : -field_velocity.x};
}

std::optional<Point2d> CoordinateTransform::field_velocity_to_canonical(
    int team_id,
    const Point2d& field_velocity) const {
    if (!valid_team(team_id) || !std::isfinite(field_velocity.x) ||
        !std::isfinite(field_velocity.y)) {
        return std::nullopt;
    }
    if (team_id == TEAM_BLUE) {
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
