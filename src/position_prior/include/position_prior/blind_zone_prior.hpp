#pragma once

#include "position_prior/navigation_mesh.hpp"
#include "position_prior/prior_types.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace position_prior {

struct BlindZoneBiasConfig {
    double trigger_distance_m = 1.2;
    double maximum_probability_mass = 0.65;
    std::size_t candidates_per_zone = 4;
    double minimum_candidate_separation_m = 0.6;
};

struct BlindZoneBiasResult {
    bool applied = false;
    std::size_t active_zone_count = 0;
    std::size_t injected_candidate_count = 0;
    double injected_probability_mass = 0.0;
};

class BlindZonePrior {
public:
    explicit BlindZonePrior(BlindZoneBiasConfig config = {});

    void load(
        const std::vector<std::string>& common_zone_paths,
        const std::string& engineer_zone_path);

    bool loaded() const { return loaded_; }
    std::size_t zone_count() const { return zones_.size(); }
    const BlindZoneBiasConfig& config() const { return config_; }

    BlindZoneBiasResult apply(
        const std::string& role,
        const Point2d& last_canonical,
        const Point2d& motion_prediction_canonical,
        double maximum_path_distance_m,
        const NavigationMesh& navigation_mesh,
        const NavigationRouteMap& routes,
        PriorDistribution& distribution) const;

private:
    struct Zone {
        std::string name;
        bool engineer_only = false;
        bool mirror_centrally = false;
        bool prefer_lowest_elevation = false;
        double maximum_height_above_lowest_m = 0.0;
        bool same_elevation_as_last = false;
        double maximum_height_delta_from_last_m = 0.0;
        std::vector<Point2d> polygon;
        Point2d centroid;
    };

    void load_file(const std::string& path, bool engineer_only);
    static bool point_in_polygon(
        const Point2d& point,
        const std::vector<Point2d>& polygon);
    static double distance_to_polygon(
        const Point2d& point,
        const std::vector<Point2d>& polygon);
    static double normalized_entropy(
        const std::vector<PriorCandidate>& candidates);

    BlindZoneBiasConfig config_;
    bool loaded_ = false;
    std::vector<Zone> zones_;
};

}  // namespace position_prior
