#pragma once

#include "position_prior/prior_types.hpp"

#include <unordered_map>
#include <vector>

namespace position_prior {

struct PriorGateConfig {
    double field_length = 28.0;
    double field_width = 15.0;
    double max_speed_mps = 5.0;
    double reachability_margin_m = 0.5;
    // 相对最后可靠观测点的绝对距离上限。<= 0 表示只使用速度可达约束。
    double max_guess_distance_m = 6.0;
    // 阈值内的近距离高斯偏好尺度。<= 0 表示不施加软距离偏好。
    double distance_preference_sigma_m = 3.0;
    double minimum_confidence = 0.05;
    std::size_t output_top_k = 5;
    std::unordered_map<int, double> motion_gate_mps{{2, 0.8}, {5, 0.4}, {10, 0.4}};
    std::unordered_map<int, double> motion_sigma_m{{2, 0.5}, {5, 3.0}, {10, 4.0}};
    std::vector<Rect2d> blocked_regions_canonical;
};

class PriorGate {
public:
    explicit PriorGate(PriorGateConfig config = {});

    const PriorGateConfig& config() const { return config_; }

    GateResult apply(
        const PriorDistribution& distribution,
        const Point2d& last_canonical,
        const Point2d& canonical_velocity,
        double lost_duration_s,
        double last_tracking_confidence) const;

private:
    bool in_field(const Point2d& point) const;
    bool blocked(const Point2d& point) const;

    PriorGateConfig config_;
};

}  // namespace position_prior
