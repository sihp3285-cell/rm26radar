#pragma once

#include <radar27_interfaces/msg/world_target_array.hpp>
#include <radar27_interfaces/msg/prior_prediction_array.hpp>
#include <radar27_interfaces/msg/fused_target_array.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace target_fusion {
inline std::int64_t stamp_ns(const builtin_interfaces::msg::Time& t) {
    return static_cast<std::int64_t>(t.sec) * 1000000000LL + t.nanosec;
}

// Source selection only: no coordinate averaging, extrapolation, or Kalman feedback.
inline radar27_interfaces::msg::FusedTargetArray fuse(
    const radar27_interfaces::msg::WorldTargetArray& world,
    const radar27_interfaces::msg::PriorPredictionArray* prior,
    double max_prior_age_s) {
    using World = radar27_interfaces::msg::WorldTarget;
    using Fused = radar27_interfaces::msg::FusedTarget;
    radar27_interfaces::msg::FusedTargetArray output;
    output.header = world.header;
    output.targets.resize(10);
    const double prior_age = prior
        ? (stamp_ns(world.header.stamp) - stamp_ns(prior->header.stamp)) * 1e-9 : -1.0;
    const bool prior_fresh = prior && prior->model_enabled && prior_age >= 0.0 &&
        prior_age <= max_prior_age_s;
    for (std::size_t slot = 0; slot < output.targets.size(); ++slot) {
        auto& out = output.targets[slot];
        out.slot_idx = static_cast<int>(slot);
        out.track_id = -1;
        if (slot >= world.targets.size()) continue;
        const auto& target = world.targets[slot];
        out.track_id = target.track_id;
        out.team_id = target.team_id;
        out.class_id = target.class_id;
        out.lost_duration_s = target.lost_duration_s;
        if (target.is_dead) continue;
        if (target.valid && target.observed &&
            (target.position_source == World::POSITION_MEASURED ||
             target.position_source == World::POSITION_TRACKED) &&
            std::isfinite(target.world_x) && std::isfinite(target.world_z) &&
            std::isfinite(target.tracking_confidence)) {
            out.valid = true;
            out.source = Fused::SOURCE_TRACKED;
            out.world_x = target.world_x;
            out.world_z = target.world_z;
            out.confidence = target.tracking_confidence;
            out.source_stamp = world.header.stamp;
            continue;
        }
        // A malformed observation must not resurrect an older prediction.
        if (target.observed || !prior_fresh) continue;
        for (const auto& prediction : prior->predictions) {
            if (prediction.slot_idx != static_cast<int>(slot) || !prediction.valid ||
                prediction.team_id != target.team_id ||
                prediction.role_class_id != target.class_id ||
                (target.track_id >= 0 && prediction.track_id != target.track_id) ||
                stamp_ns(prediction.last_observed_time) < stamp_ns(target.last_observed_time) ||
                !std::isfinite(prediction.prior_world_x) ||
                !std::isfinite(prediction.prior_world_z) ||
                !std::isfinite(prediction.prior_confidence)) continue;
            out.valid = true;
            out.track_id = prediction.track_id;
            out.source = Fused::SOURCE_PRIOR;
            out.world_x = prediction.prior_world_x;
            out.world_z = prediction.prior_world_z;
            out.confidence = prediction.prior_confidence;
            out.lost_duration_s = prediction.lost_duration_s;
            out.source_stamp = prior->header.stamp;
            break;
        }
    }
    return output;
}
} // namespace target_fusion
