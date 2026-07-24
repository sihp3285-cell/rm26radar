#include "position_prior/blind_zone_prior.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace position_prior {

BlindZonePrior::BlindZonePrior(BlindZoneBiasConfig config)
    : config_(std::move(config)) {
    config_.trigger_distance_m = std::max(0.0, config_.trigger_distance_m);
    config_.maximum_probability_mass =
        std::clamp(config_.maximum_probability_mass, 0.0, 0.95);
    config_.candidates_per_zone =
        std::max<std::size_t>(1, config_.candidates_per_zone);
    config_.minimum_candidate_separation_m =
        std::max(0.0, config_.minimum_candidate_separation_m);
}

void BlindZonePrior::load(
    const std::vector<std::string>& common_zone_paths,
    const std::string& engineer_zone_path) {
    loaded_ = false;
    zones_.clear();
    for (const auto& path : common_zone_paths) {
        if (!path.empty()) {
            load_file(path, false);
        }
    }
    if (!engineer_zone_path.empty()) {
        load_file(engineer_zone_path, true);
    }
    if (zones_.empty()) {
        throw std::runtime_error("未加载到任何盲区多边形");
    }
    loaded_ = true;
}

void BlindZonePrior::load_file(
    const std::string& path,
    bool engineer_only) {
    const YAML::Node root = YAML::LoadFile(path);
    const auto blind_zones = root["blind_zones"];
    if (!blind_zones || !blind_zones.IsSequence()) {
        throw std::runtime_error("盲区文件缺少 blind_zones: " + path);
    }
    for (const auto& node : blind_zones) {
        const auto polygon_node = node["polygon"];
        if (!polygon_node || !polygon_node.IsSequence() ||
            polygon_node.size() < 3) {
            throw std::runtime_error("盲区 polygon 顶点不足: " + path);
        }
        Zone zone;
        zone.name = node["name"] ? node["name"].as<std::string>() : path;
        zone.engineer_only = engineer_only;
        zone.mirror_centrally = node["mirror_centrally"]
            ? node["mirror_centrally"].as<bool>() : false;
        zone.prefer_lowest_elevation = node["prefer_lowest_elevation"]
            ? node["prefer_lowest_elevation"].as<bool>() : false;
        zone.maximum_height_above_lowest_m =
            node["max_height_above_lowest_m"]
                ? node["max_height_above_lowest_m"].as<double>() : 0.0;
        if (!std::isfinite(zone.maximum_height_above_lowest_m) ||
            zone.maximum_height_above_lowest_m < 0.0) {
            throw std::runtime_error(
                "盲区最低高度层容差无效: " + path);
        }
        zone.same_elevation_as_last = node["same_elevation_as_last"]
            ? node["same_elevation_as_last"].as<bool>() : false;
        zone.maximum_height_delta_from_last_m =
            node["max_height_delta_from_last_m"]
                ? node["max_height_delta_from_last_m"].as<double>() : 0.0;
        if (!std::isfinite(zone.maximum_height_delta_from_last_m) ||
            zone.maximum_height_delta_from_last_m < 0.0) {
            throw std::runtime_error(
                "盲区最后观测高度层容差无效: " + path);
        }
        for (const auto& vertex : polygon_node) {
            if (!vertex.IsSequence() || vertex.size() != 2) {
                throw std::runtime_error("盲区 polygon 顶点格式无效: " + path);
            }
            Point2d point{vertex[0].as<double>(), vertex[1].as<double>()};
            if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                point.x < 0.0 || point.x > 28.0 ||
                point.y < 0.0 || point.y > 15.0) {
                throw std::runtime_error("盲区 polygon 顶点越界: " + path);
            }
            zone.centroid.x += point.x;
            zone.centroid.y += point.y;
            zone.polygon.push_back(point);
        }
        zone.centroid.x /= static_cast<double>(zone.polygon.size());
        zone.centroid.y /= static_cast<double>(zone.polygon.size());
        zones_.push_back(zone);
        if (zone.mirror_centrally) {
            Zone mirrored = zone;
            mirrored.name += "_mirrored";
            mirrored.mirror_centrally = false;
            mirrored.centroid = {};
            for (auto& point : mirrored.polygon) {
                point.x = 28.0 - point.x;
                point.y = 15.0 - point.y;
                mirrored.centroid.x += point.x;
                mirrored.centroid.y += point.y;
            }
            mirrored.centroid.x /=
                static_cast<double>(mirrored.polygon.size());
            mirrored.centroid.y /=
                static_cast<double>(mirrored.polygon.size());
            zones_.push_back(std::move(mirrored));
        }
    }
}

bool BlindZonePrior::point_in_polygon(
    const Point2d& point,
    const std::vector<Point2d>& polygon) {
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

double BlindZonePrior::distance_to_polygon(
    const Point2d& point,
    const std::vector<Point2d>& polygon) {
    if (point_in_polygon(point, polygon)) {
        return 0.0;
    }
    double result = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < polygon.size(); ++index) {
        const Point2d& start = polygon[index];
        const Point2d& end = polygon[(index + 1) % polygon.size()];
        const double dx = end.x - start.x;
        const double dy = end.y - start.y;
        const double length_squared = dx * dx + dy * dy;
        const double projection = length_squared > 0.0
            ? std::clamp(
                ((point.x - start.x) * dx + (point.y - start.y) * dy) /
                    length_squared,
                0.0, 1.0)
            : 0.0;
        result = std::min(result, std::hypot(
            point.x - (start.x + projection * dx),
            point.y - (start.y + projection * dy)));
    }
    return result;
}

double BlindZonePrior::normalized_entropy(
    const std::vector<PriorCandidate>& candidates) {
    if (candidates.size() <= 1) {
        return 0.0;
    }
    double total = 0.0;
    for (const auto& candidate : candidates) {
        total += std::max(0.0, candidate.probability);
    }
    if (total <= 0.0) {
        return 1.0;
    }
    double entropy = 0.0;
    for (const auto& candidate : candidates) {
        const double probability =
            std::max(0.0, candidate.probability) / total;
        if (probability > 0.0) {
            entropy -= probability * std::log(probability);
        }
    }
    return std::clamp(
        entropy / std::log(static_cast<double>(candidates.size())), 0.0, 1.0);
}

BlindZoneBiasResult BlindZonePrior::apply(
    const std::string& role,
    const Point2d& last_canonical,
    const Point2d& motion_prediction_canonical,
    double maximum_path_distance_m,
    const NavigationMesh& navigation_mesh,
    const NavigationRouteMap& routes,
    PriorDistribution& distribution) const {
    BlindZoneBiasResult result;
    if (!loaded_ || !routes.valid || !distribution.valid ||
        distribution.candidates.empty() ||
        maximum_path_distance_m < 0.0) {
        return result;
    }

    struct ActiveZone {
        const Zone* zone = nullptr;
        double proximity_strength = 0.0;
        double maximum_allowed_height_m =
            std::numeric_limits<double>::infinity();
        double minimum_allowed_height_m =
            -std::numeric_limits<double>::infinity();
        std::vector<int> selected_cells;
    };
    std::vector<ActiveZone> active;
    for (const auto& zone : zones_) {
        if (zone.engineer_only && role != "engineer") {
            continue;
        }
        const bool inside = point_in_polygon(last_canonical, zone.polygon);
        const double distance = distance_to_polygon(
            last_canonical, zone.polygon);
        if (!inside &&
            (config_.trigger_distance_m <= 0.0 ||
             distance > config_.trigger_distance_m)) {
            continue;
        }

        ActiveZone candidate_zone;
        candidate_zone.zone = &zone;
        const double linear_strength = inside ? 1.0 :
            std::clamp(
                1.0 - distance / config_.trigger_distance_m, 0.0, 1.0);
        // 刚进入“靠近”范围也保留一部分偏置，避免在阈值边缘完全没有效果。
        candidate_zone.proximity_strength =
            inside ? 1.0 : 0.35 + 0.65 * linear_strength;

        auto cells = navigation_mesh.walkable_cells_in_polygon(
            role, zone.polygon, routes.component_id);
        if (zone.same_elevation_as_last && !cells.empty()) {
            const double last_height =
                navigation_mesh.cell_height_m(routes.start_cell);
            candidate_zone.minimum_allowed_height_m =
                last_height - zone.maximum_height_delta_from_last_m;
            candidate_zone.maximum_allowed_height_m =
                last_height + zone.maximum_height_delta_from_last_m;
            cells.erase(std::remove_if(cells.begin(), cells.end(),
                [&](int cell) {
                    const double height =
                        navigation_mesh.cell_height_m(cell);
                    return !std::isfinite(height) ||
                        height <
                            candidate_zone.minimum_allowed_height_m - 1e-9 ||
                        height >
                            candidate_zone.maximum_allowed_height_m + 1e-9;
                }), cells.end());
        }
        if (zone.prefer_lowest_elevation && !cells.empty()) {
            const auto lowest = std::min_element(
                cells.begin(), cells.end(),
                [&](int lhs, int rhs) {
                    return navigation_mesh.cell_height_m(lhs) <
                        navigation_mesh.cell_height_m(rhs);
                });
            const double maximum_height =
                navigation_mesh.cell_height_m(*lowest) +
                zone.maximum_height_above_lowest_m;
            candidate_zone.maximum_allowed_height_m = maximum_height;
            cells.erase(std::remove_if(cells.begin(), cells.end(),
                [&](int cell) {
                    const double height =
                        navigation_mesh.cell_height_m(cell);
                    return !std::isfinite(height) ||
                        height > maximum_height + 1e-9;
                }), cells.end());
        }
        cells.erase(std::remove_if(cells.begin(), cells.end(),
            [&](int cell) {
                return cell < 0 ||
                    static_cast<std::size_t>(cell) >= routes.distances_m.size() ||
                    !std::isfinite(routes.distances_m[cell]) ||
                    routes.distances_m[cell] >
                        maximum_path_distance_m + 1e-9;
            }), cells.end());
        std::sort(cells.begin(), cells.end(),
            [&](int lhs, int rhs) {
                const auto score = [&](int cell) {
                    const Point2d point = navigation_mesh.cell_center(cell);
                    const double motion_distance = std::hypot(
                        point.x - motion_prediction_canonical.x,
                        point.y - motion_prediction_canonical.y);
                    const double center_distance = std::hypot(
                        point.x - zone.centroid.x,
                        point.y - zone.centroid.y);
                    return 0.45 * motion_distance + 0.55 * center_distance;
                };
                const double lhs_score = score(lhs);
                const double rhs_score = score(rhs);
                return lhs_score == rhs_score ? lhs < rhs : lhs_score < rhs_score;
            });

        for (const int cell : cells) {
            const Point2d point = navigation_mesh.cell_center(cell);
            const bool separated = std::all_of(
                candidate_zone.selected_cells.begin(),
                candidate_zone.selected_cells.end(),
                [&](int selected) {
                    const Point2d other = navigation_mesh.cell_center(selected);
                    return std::hypot(point.x - other.x, point.y - other.y) >=
                        config_.minimum_candidate_separation_m - 1e-12;
                });
            if (separated || candidate_zone.selected_cells.empty()) {
                candidate_zone.selected_cells.push_back(cell);
                if (candidate_zone.selected_cells.size() >=
                    config_.candidates_per_zone) {
                    break;
                }
            }
        }
        if (!candidate_zone.selected_cells.empty()) {
            active.push_back(std::move(candidate_zone));
        }
    }
    if (active.empty()) {
        return result;
    }

    double maximum_strength = 0.0;
    std::size_t injected_count = 0;
    for (const auto& zone : active) {
        maximum_strength = std::max(maximum_strength, zone.proximity_strength);
        injected_count += zone.selected_cells.size();
    }
    if (injected_count == 0) {
        return result;
    }

    const double injected_mass =
        config_.maximum_probability_mass * maximum_strength;
    // 同一盲区内若原模型候选落在被排除的高层（例如 gully 中心岛），也将其
    // 概率清零。否则它仍可能作为 Top-K 空心圆显示在高台上，造成误导。
    for (auto& candidate : distribution.candidates) {
        for (const auto& zone : active) {
            if (!std::isfinite(zone.maximum_allowed_height_m) ||
                !point_in_polygon(candidate.canonical, zone.zone->polygon)) {
                continue;
            }
            const auto snap = navigation_mesh.snap_to_walkable(
                role, candidate.canonical,
                navigation_mesh.resolution() * 1.5, routes.component_id);
            if (!snap.valid ||
                navigation_mesh.cell_height_m(snap.cell_index) <
                    zone.minimum_allowed_height_m - 1e-9 ||
                navigation_mesh.cell_height_m(snap.cell_index) >
                    zone.maximum_allowed_height_m + 1e-9) {
                candidate.probability = 0.0;
                break;
            }
        }
    }
    double original_total = 0.0;
    for (const auto& candidate : distribution.candidates) {
        original_total += std::max(0.0, candidate.probability);
    }
    const double original_scale = original_total > 0.0
        ? (1.0 - injected_mass) / original_total : 0.0;
    for (auto& candidate : distribution.candidates) {
        candidate.probability =
            std::max(0.0, candidate.probability) * original_scale;
    }

    double strength_total = 0.0;
    for (const auto& zone : active) {
        strength_total += zone.proximity_strength;
    }
    for (const auto& zone : active) {
        const double zone_mass = injected_mass *
            zone.proximity_strength / strength_total;
        const double candidate_mass =
            zone_mass / static_cast<double>(zone.selected_cells.size());
        for (const int cell : zone.selected_cells) {
            const Point2d point = navigation_mesh.cell_center(cell);
            auto duplicate = std::find_if(
                distribution.candidates.begin(),
                distribution.candidates.end(),
                [&](const PriorCandidate& candidate) {
                    return std::hypot(
                        candidate.canonical.x - point.x,
                        candidate.canonical.y - point.y) <=
                        navigation_mesh.resolution() * 0.5;
                });
            if (duplicate != distribution.candidates.end()) {
                duplicate->probability += candidate_mass;
                duplicate->from_blind_zone = true;
            } else {
                PriorCandidate candidate;
                candidate.grid_index = 1000000 + cell;
                candidate.canonical = point;
                candidate.probability = candidate_mass;
                candidate.from_blind_zone = true;
                distribution.candidates.push_back(candidate);
            }
        }
    }

    distribution.normalized_entropy =
        normalized_entropy(distribution.candidates);
    distribution.retained_probability_mass = 1.0;
    result.applied = true;
    result.active_zone_count = active.size();
    result.injected_candidate_count = injected_count;
    result.injected_probability_mass = injected_mass;
    return result;
}

}  // namespace position_prior
