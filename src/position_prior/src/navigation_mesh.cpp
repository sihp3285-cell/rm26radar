#include "position_prior/navigation_mesh.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>

namespace position_prior {

namespace {

constexpr double SQRT_TWO = 1.4142135623730950488;

}  // namespace

void NavigationMesh::load(const std::string& path) {
    loaded_ = false;
    profiles_.clear();
    height_mm_.clear();

    const YAML::Node root = YAML::LoadFile(path);
    const auto metadata = root["metadata"];
    const auto geometry = root["geometry"];
    if (!metadata || metadata["asset_version"].as<int>() != 1 ||
        metadata["map_id"].as<std::string>() != "RMUC2026" ||
        !geometry ||
        geometry["canonical_team"].as<std::string>() != "red") {
        throw std::runtime_error("不支持的 navgrid 版本、地图或 canonical 阵营");
    }

    field_length_ = geometry["field_length_m"].as<double>();
    field_width_ = geometry["field_width_m"].as<double>();
    resolution_ = geometry["resolution_m"].as<double>();
    columns_ = geometry["columns"].as<int>();
    rows_ = geometry["rows"].as<int>();
    const std::size_t cell_count =
        static_cast<std::size_t>(rows_) * static_cast<std::size_t>(columns_);
    if (std::abs(field_length_ - 28.0) > 1e-9 ||
        std::abs(field_width_ - 15.0) > 1e-9 ||
        resolution_ <= 0.0 || columns_ <= 0 || rows_ <= 0 ||
        std::abs(columns_ * resolution_ - field_length_) > 1e-9 ||
        std::abs(rows_ * resolution_ - field_width_) > 1e-9) {
        throw std::runtime_error("navgrid geometry 与 RMUC2026 在线约定不一致");
    }

    const auto heights = root["surface"]["height_mm"];
    if (!heights || !heights.IsSequence() || heights.size() != cell_count) {
        throw std::runtime_error("navgrid surface.height_mm 长度无效");
    }
    height_mm_.reserve(cell_count);
    for (const auto& value : heights) {
        height_mm_.push_back(value.as<int>());
    }

    const auto profiles = root["profiles"];
    for (const std::string& role :
         {"hero", "engineer", "infantry3", "infantry4", "sentry"}) {
        const auto node = profiles[role];
        if (!node) {
            throw std::runtime_error("navgrid 缺少兵种 profile: " + role);
        }
        const auto walkable = node["walkable"];
        const auto components = node["component_id"];
        if (!walkable || !components ||
            walkable.size() != cell_count || components.size() != cell_count) {
            throw std::runtime_error("navgrid profile 数组长度无效: " + role);
        }
        Profile profile;
        profile.maximum_step_height_m =
            node["parameters"]["max_step_height_m"].as<double>();
        profile.walkable.reserve(cell_count);
        profile.component_id.reserve(cell_count);
        for (std::size_t index = 0; index < cell_count; ++index) {
            profile.walkable.push_back(
                walkable[index].as<int>() == 0 ? std::uint8_t{0} : std::uint8_t{1});
            profile.component_id.push_back(components[index].as<int>());
        }
        profiles_.emplace(role, std::move(profile));
    }

    path_ = path;
    loaded_ = true;
}

bool NavigationMesh::supports_role(const std::string& role) const {
    return loaded_ && profiles_.find(role) != profiles_.end();
}

bool NavigationMesh::valid_cell(int row, int column) const {
    return row >= 0 && row < rows_ && column >= 0 && column < columns_;
}

Point2d NavigationMesh::cell_center(int cell_index) const {
    if (cell_index < 0 ||
        cell_index >= rows_ * columns_) {
        return {};
    }
    const int row = cell_index / columns_;
    const int column = cell_index % columns_;
    return Point2d{
        (static_cast<double>(column) + 0.5) * resolution_,
        (static_cast<double>(row) + 0.5) * resolution_};
}

double NavigationMesh::cell_height_m(int cell_index) const {
    if (cell_index < 0 ||
        static_cast<std::size_t>(cell_index) >= height_mm_.size()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return static_cast<double>(height_mm_[cell_index]) * 1e-3;
}

NavigationSnap NavigationMesh::snap_to_walkable(
    const std::string& role,
    const Point2d& canonical,
    double maximum_snap_distance_m,
    int required_component_id) const {
    NavigationSnap result;
    const auto profile_it = profiles_.find(role);
    if (!loaded_ || profile_it == profiles_.end() ||
        !std::isfinite(canonical.x) || !std::isfinite(canonical.y) ||
        maximum_snap_distance_m < 0.0) {
        return result;
    }

    const auto& profile = profile_it->second;
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < profile.walkable.size(); ++index) {
        if (!profile.walkable[index] ||
            (required_component_id >= 0 &&
             profile.component_id[index] != required_component_id)) {
            continue;
        }
        const Point2d center = cell_center(static_cast<int>(index));
        const double distance = std::hypot(
            center.x - canonical.x, center.y - canonical.y);
        if (distance <= maximum_snap_distance_m + 1e-12 &&
            (distance < best_distance - 1e-12 ||
             (std::abs(distance - best_distance) <= 1e-12 &&
              static_cast<int>(index) < result.cell_index))) {
            best_distance = distance;
            result.valid = true;
            result.cell_index = static_cast<int>(index);
            result.component_id = profile.component_id[index];
            result.canonical = center;
            result.snap_distance_m = distance;
        }
    }
    return result;
}

bool NavigationMesh::edge_is_walkable(
    const Profile& profile,
    int row,
    int column,
    int next_row,
    int next_column) const {
    if (!valid_cell(next_row, next_column)) {
        return false;
    }
    const int current = row * columns_ + column;
    const int next = next_row * columns_ + next_column;
    const int maximum_step_mm = static_cast<int>(
        std::lround(profile.maximum_step_height_m * 1000.0));
    const auto can_step = [&](int from, int to) {
        return profile.walkable[to] &&
            std::abs(height_mm_[to] - height_mm_[from]) <= maximum_step_mm;
    };
    if (!can_step(current, next)) {
        return false;
    }

    const int delta_row = next_row - row;
    const int delta_column = next_column - column;
    if (delta_row == 0 || delta_column == 0) {
        return true;
    }

    // 对角移动必须能沿任一相邻正交格安全通过。除防止穿墙角外，还要检查
    // 两个正交格到目标格的台阶高度；这是 navgrid golden 的边连接约定。
    const int horizontal = row * columns_ + next_column;
    const int vertical = next_row * columns_ + column;
    return can_step(current, horizontal) &&
           can_step(current, vertical) &&
           can_step(horizontal, next) &&
           can_step(vertical, next);
}

NavigationRouteMap NavigationMesh::route_map(
    const std::string& role,
    const Point2d& canonical_start,
    double maximum_snap_distance_m) const {
    NavigationRouteMap result;
    result.role = role;
    if (!supports_role(role)) {
        result.reason = "unsupported_navgrid_role";
        return result;
    }
    result.start_snap = snap_to_walkable(
        role, canonical_start, maximum_snap_distance_m);
    if (!result.start_snap.valid) {
        result.reason = "start_not_walkable";
        return result;
    }

    const auto& profile = profiles_.at(role);
    const std::size_t cell_count = profile.walkable.size();
    result.distances_m.assign(
        cell_count, std::numeric_limits<double>::infinity());
    result.start_cell = result.start_snap.cell_index;
    result.component_id = result.start_snap.component_id;
    result.distances_m[result.start_cell] = 0.0;

    using QueueItem = std::pair<double, int>;
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> queue;
    queue.emplace(0.0, result.start_cell);
    constexpr std::array<std::pair<int, int>, 8> offsets{{
        {-1, -1}, {-1, 0}, {-1, 1}, {0, -1},
        {0, 1}, {1, -1}, {1, 0}, {1, 1}}};

    while (!queue.empty()) {
        const auto [distance, cell] = queue.top();
        queue.pop();
        if (distance > result.distances_m[cell] + 1e-12) {
            continue;
        }
        const int row = cell / columns_;
        const int column = cell % columns_;
        for (const auto& [delta_row, delta_column] : offsets) {
            const int next_row = row + delta_row;
            const int next_column = column + delta_column;
            if (!edge_is_walkable(
                    profile, row, column, next_row, next_column)) {
                continue;
            }
            const int next = next_row * columns_ + next_column;
            const double step_distance =
                (delta_row != 0 && delta_column != 0)
                    ? resolution_ * SQRT_TWO : resolution_;
            const double next_distance = distance + step_distance;
            if (next_distance + 1e-12 < result.distances_m[next]) {
                result.distances_m[next] = next_distance;
                queue.emplace(next_distance, next);
            }
        }
    }

    result.valid = true;
    result.reason = "ok";
    return result;
}

double NavigationMesh::path_distance(
    const NavigationRouteMap& routes,
    const Point2d& canonical_goal,
    double maximum_snap_distance_m,
    NavigationSnap* goal_snap) const {
    const double infinity = std::numeric_limits<double>::infinity();
    if (!routes.valid || routes.distances_m.empty() ||
        !supports_role(routes.role)) {
        return infinity;
    }
    const NavigationSnap snapped = snap_to_walkable(
        routes.role, canonical_goal, maximum_snap_distance_m);
    if (goal_snap) {
        *goal_snap = snapped;
    }
    if (!snapped.valid || snapped.component_id != routes.component_id ||
        snapped.cell_index < 0 ||
        static_cast<std::size_t>(snapped.cell_index) >= routes.distances_m.size()) {
        return infinity;
    }
    return routes.distances_m[snapped.cell_index];
}

bool NavigationMesh::point_in_polygon(
    const Point2d& point,
    const std::vector<Point2d>& polygon) {
    if (polygon.size() < 3) {
        return false;
    }
    bool inside = false;
    for (std::size_t i = 0, j = polygon.size() - 1;
         i < polygon.size(); j = i++) {
        const Point2d& a = polygon[i];
        const Point2d& b = polygon[j];
        const bool crosses = (a.y > point.y) != (b.y > point.y);
        if (crosses) {
            const double intersection_x =
                (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x;
            if (point.x < intersection_x) {
                inside = !inside;
            }
        }
    }
    return inside;
}

std::vector<int> NavigationMesh::walkable_cells_in_polygon(
    const std::string& role,
    const std::vector<Point2d>& polygon,
    int required_component_id) const {
    std::vector<int> result;
    const auto profile_it = profiles_.find(role);
    if (!loaded_ || profile_it == profiles_.end() || polygon.size() < 3) {
        return result;
    }
    const auto& profile = profile_it->second;
    for (std::size_t index = 0; index < profile.walkable.size(); ++index) {
        if (!profile.walkable[index] ||
            (required_component_id >= 0 &&
             profile.component_id[index] != required_component_id)) {
            continue;
        }
        if (point_in_polygon(cell_center(static_cast<int>(index)), polygon)) {
            result.push_back(static_cast<int>(index));
        }
    }
    return result;
}

}  // namespace position_prior
