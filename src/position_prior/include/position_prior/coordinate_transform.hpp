#pragma once

#include "position_prior/prior_types.hpp"

#include <optional>

namespace position_prior {

class CoordinateTransform {
public:
    CoordinateTransform(
        double field_length = 28.0,
        double field_width = 15.0,
        bool world_z_toward_blue = true);

    void set_world_z_toward_blue(bool value) { world_z_toward_blue_ = value; }
    bool world_z_toward_blue() const { return world_z_toward_blue_; }

    double field_length() const { return field_length_; }
    double field_width() const { return field_width_; }

    bool valid_field(const Point2d& field) const;
    bool valid_team(int team_id) const;

    std::optional<Point2d> world_to_field(const Point2d& world) const;
    std::optional<Point2d> field_to_world(const Point2d& field) const;
    std::optional<Point2d> field_to_canonical(int team_id, const Point2d& field) const;
    std::optional<Point2d> canonical_to_field(int team_id, const Point2d& canonical) const;
    std::optional<Point2d> world_to_canonical(int team_id, const Point2d& world) const;
    std::optional<Point2d> canonical_to_world(int team_id, const Point2d& canonical) const;

    Point2d world_velocity_to_field(const Point2d& world_velocity) const;
    Point2d field_velocity_to_world(const Point2d& field_velocity) const;
    std::optional<Point2d> field_velocity_to_canonical(
        int team_id, const Point2d& field_velocity) const;
    std::optional<Point2d> canonical_velocity_to_field(
        int team_id, const Point2d& canonical_velocity) const;

private:
    double field_length_;
    double field_width_;
    bool world_z_toward_blue_;
};

}  // namespace position_prior
