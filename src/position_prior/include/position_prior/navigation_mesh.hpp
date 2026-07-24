#pragma once

#include "position_prior/prior_types.hpp"

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

namespace position_prior {

struct NavigationSnap {
    bool valid = false;
    int cell_index = -1;
    int component_id = -1;
    Point2d canonical;
    double snap_distance_m = 0.0;
};

struct NavigationRouteMap {
    bool valid = false;
    std::string reason;
    std::string role;
    int start_cell = -1;
    int component_id = -1;
    NavigationSnap start_snap;
    std::vector<double> distances_m;
};

class NavigationMesh {
public:
    void load(const std::string& path);

    bool loaded() const { return loaded_; }
    const std::string& path() const { return path_; }
    double field_length() const { return field_length_; }
    double field_width() const { return field_width_; }
    double resolution() const { return resolution_; }
    int rows() const { return rows_; }
    int columns() const { return columns_; }

    bool supports_role(const std::string& role) const;
    NavigationSnap snap_to_walkable(
        const std::string& role,
        const Point2d& canonical,
        double maximum_snap_distance_m,
        int required_component_id = -1) const;
    NavigationRouteMap route_map(
        const std::string& role,
        const Point2d& canonical_start,
        double maximum_snap_distance_m) const;
    double path_distance(
        const NavigationRouteMap& routes,
        const Point2d& canonical_goal,
        double maximum_snap_distance_m,
        NavigationSnap* goal_snap = nullptr) const;

    Point2d cell_center(int cell_index) const;
    double cell_height_m(int cell_index) const;
    std::vector<int> walkable_cells_in_polygon(
        const std::string& role,
        const std::vector<Point2d>& polygon,
        int required_component_id = -1) const;

private:
    struct Profile {
        double maximum_step_height_m = 0.0;
        std::vector<std::uint8_t> walkable;
        std::vector<int> component_id;
    };

    bool valid_cell(int row, int column) const;
    bool edge_is_walkable(
        const Profile& profile,
        int row,
        int column,
        int next_row,
        int next_column) const;
    static bool point_in_polygon(
        const Point2d& point,
        const std::vector<Point2d>& polygon);

    bool loaded_ = false;
    std::string path_;
    double field_length_ = 0.0;
    double field_width_ = 0.0;
    double resolution_ = 0.0;
    int rows_ = 0;
    int columns_ = 0;
    std::vector<int> height_mm_;
    std::unordered_map<std::string, Profile> profiles_;
};

}  // namespace position_prior
