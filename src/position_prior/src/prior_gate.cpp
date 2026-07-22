#include "position_prior/prior_gate.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace position_prior {

PriorGate::PriorGate(PriorGateConfig config) : config_(std::move(config)) {}

bool PriorGate::in_field(const Point2d& point) const {
    return std::isfinite(point.x) && std::isfinite(point.y) &&
           point.x >= 0.0 && point.x <= config_.field_length &&
           point.y >= 0.0 && point.y <= config_.field_width;
}

bool PriorGate::blocked(const Point2d& point) const {
    return std::any_of(
        config_.blocked_regions_canonical.begin(),
        config_.blocked_regions_canonical.end(),
        [&point](const Rect2d& region) { return region.contains(point); });
}

GateResult PriorGate::apply(
    const PriorDistribution& distribution,
    const Point2d& last_canonical,
    const Point2d& canonical_velocity,
    double lost_duration_s,
    double last_tracking_confidence) const {
    GateResult result;
    if (!distribution.valid || distribution.candidates.empty()) {
        result.rejection_reason = "no_distribution";
        return result;
    }
    if (!in_field(last_canonical)) {
        result.rejection_reason = "last_position_out_of_field";
        return result;
    }

    const int horizon = distribution.horizon_seconds;
    const double raw_speed = std::hypot(canonical_velocity.x, canonical_velocity.y);
    Point2d capped_velocity = canonical_velocity;
    if (raw_speed > config_.max_speed_mps && raw_speed > 0.0) {
        const double scale = config_.max_speed_mps / raw_speed;
        capped_velocity.x *= scale;
        capped_velocity.y *= scale;
    }
    const double elapsed_s = std::max(0.0, lost_duration_s);
    result.motion_prediction_canonical = Point2d{
        last_canonical.x + capped_velocity.x * elapsed_s,
        last_canonical.y + capped_velocity.y * elapsed_s};
    result.motion_prediction_canonical.x = std::clamp(
        result.motion_prediction_canonical.x, 0.0, config_.field_length);
    result.motion_prediction_canonical.y = std::clamp(
        result.motion_prediction_canonical.y, 0.0, config_.field_width);

    const auto threshold_it = config_.motion_gate_mps.find(horizon);
    const double motion_threshold = threshold_it == config_.motion_gate_mps.end()
        ? 0.4 : threshold_it->second;
    const auto sigma_it = config_.motion_sigma_m.find(horizon);
    const double sigma = std::max(1e-3,
        sigma_it == config_.motion_sigma_m.end() ? 3.0 : sigma_it->second);
    result.motion_gated = raw_speed >= motion_threshold;

    const double physical_maximum_distance =
        config_.max_speed_mps * elapsed_s + config_.reachability_margin_m;
    const double configured_maximum_distance = config_.max_guess_distance_m > 0.0
        ? config_.max_guess_distance_m
        : std::numeric_limits<double>::infinity();
    // 同时满足“按时间和速度可达”与“不可离最后观测点过远”。较小者生效。
    const double maximum_distance = std::min(
        physical_maximum_distance, configured_maximum_distance);
    const double distance_sigma = config_.distance_preference_sigma_m;
    result.candidates.reserve(distribution.candidates.size());
    double raw_fused_total = 0.0;

    for (const auto& candidate : distribution.candidates) {
        GatedCandidate gated;
        gated.prior = candidate;
        gated.distance_from_last_m = std::hypot(
            candidate.canonical.x - last_canonical.x,
            candidate.canonical.y - last_canonical.y);
        gated.blocked = blocked(candidate.canonical);
        gated.reachable = in_field(candidate.canonical) && !gated.blocked &&
            gated.distance_from_last_m <= maximum_distance;
        if (gated.reachable) {
            result.reachable_probability_mass += candidate.probability;
            // 在硬阈值内继续保留连续偏好，避免高概率远点压过同样合理的近点。
            // 该权重只改变候选间相对排序，不会让阈值外候选重新变为可达。
            const double distance_weight = distance_sigma > 0.0
                ? std::exp(-gated.distance_from_last_m * gated.distance_from_last_m /
                    (2.0 * distance_sigma * distance_sigma))
                : 1.0;
            double motion_weight = 1.0;
            if (result.motion_gated) {
                const double dx = candidate.canonical.x - result.motion_prediction_canonical.x;
                const double dy = candidate.canonical.y - result.motion_prediction_canonical.y;
                motion_weight = std::exp(-(dx * dx + dy * dy) / (2.0 * sigma * sigma));
            }
            gated.fused_probability =
                candidate.probability * motion_weight * distance_weight;
            raw_fused_total += gated.fused_probability;
        }
        result.candidates.push_back(gated);
    }

    if (raw_fused_total <= std::numeric_limits<double>::epsilon()) {
        result.rejection_reason = "no_reachable_candidate";
        return result;
    }
    for (auto& candidate : result.candidates) {
        candidate.fused_probability /= raw_fused_total;
    }

    if (result.motion_gated) {
        for (const auto& candidate : result.candidates) {
            result.predicted_canonical.x +=
                candidate.fused_probability * candidate.prior.canonical.x;
            result.predicted_canonical.y +=
                candidate.fused_probability * candidate.prior.canonical.y;
        }
    } else {
        // 离线评测中的 motion gate：低速目标保持最后可靠位置。
        result.predicted_canonical = last_canonical;
    }

    if (config_.max_guess_distance_m > 0.0) {
        const double prediction_distance = std::hypot(
            result.predicted_canonical.x - last_canonical.x,
            result.predicted_canonical.y - last_canonical.y);
        if (prediction_distance > config_.max_guess_distance_m + 1e-9) {
            result.rejection_reason = "prediction_beyond_distance_threshold";
            return result;
        }
    }

    std::sort(result.candidates.begin(), result.candidates.end(),
        [](const GatedCandidate& lhs, const GatedCandidate& rhs) {
            if (lhs.fused_probability != rhs.fused_probability) {
                return lhs.fused_probability > rhs.fused_probability;
            }
            return lhs.prior.grid_index < rhs.prior.grid_index;
        });
    if (config_.output_top_k > 0 && result.candidates.size() > config_.output_top_k) {
        result.candidates.resize(config_.output_top_k);
    }

    const double sample_reliability =
        distribution.fallback_level == FallbackLevel::LOCAL_ZONE
            ? std::clamp(distribution.local_weight, 0.0, 1.0)
            : 0.45;
    const double entropy_reliability =
        0.25 + 0.75 * (1.0 - std::clamp(distribution.normalized_entropy, 0.0, 1.0));
    // 丢失时长只用于选择模型 horizon 和运动可达性，不再让置信度随时间
    // 线性归零。否则即使缓存保留，长时间丢失后也会被低置信度门控停猜。
    result.confidence =
        std::clamp(last_tracking_confidence, 0.0, 1.0) *
        sample_reliability * entropy_reliability *
        std::clamp(result.reachable_probability_mass, 0.0, 1.0);

    if (result.confidence < config_.minimum_confidence) {
        result.rejection_reason = "confidence_below_threshold";
        return result;
    }
    result.valid = true;
    return result;
}

}  // namespace position_prior
