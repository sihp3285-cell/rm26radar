#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace position_prior {

struct Point2d {
    double x = 0.0;
    double y = 0.0;
};

struct Rect2d {
    double min_x = 0.0;
    double min_y = 0.0;
    double max_x = 0.0;
    double max_y = 0.0;

    bool contains(const Point2d& point) const {
        return point.x >= min_x && point.x <= max_x &&
               point.y >= min_y && point.y <= max_y;
    }
};

enum class FallbackLevel : std::uint8_t {
    LOCAL_ZONE = 0,
    GLOBAL_ROLE = 1,
};

struct PriorCandidate {
    int grid_index = -1;
    Point2d canonical;
    double probability = 0.0;
};

struct PriorDistribution {
    bool valid = false;
    std::string error;
    std::string role;
    std::string context;
    int horizon_seconds = 0;
    int zone_index = -1;
    FallbackLevel fallback_level = FallbackLevel::GLOBAL_ROLE;
    std::uint32_t sample_count = 0;
    double local_weight = 0.0;
    double stay_probability = 0.0;
    double retained_probability_mass = 0.0;
    double normalized_entropy = 1.0;
    std::vector<PriorCandidate> candidates;
};

struct GatedCandidate {
    PriorCandidate prior;
    double fused_probability = 0.0;
    bool reachable = false;
    bool blocked = false;
    double distance_from_last_m = 0.0;
};

struct GateResult {
    bool valid = false;
    std::string rejection_reason;
    Point2d predicted_canonical;
    Point2d motion_prediction_canonical;
    double confidence = 0.0;
    double reachable_probability_mass = 0.0;
    bool motion_gated = false;
    std::vector<GatedCandidate> candidates;
};

}  // namespace position_prior
