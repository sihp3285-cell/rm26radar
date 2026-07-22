#include "position_prior/observation_confirmation.hpp"

#include <algorithm>
#include <cmath>

namespace position_prior {
namespace {

constexpr double NS_TO_SECONDS = 1e-9;

double distance(const Point2d& lhs, const Point2d& rhs) {
    return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

bool finite(const Point2d& point) {
    return std::isfinite(point.x) && std::isfinite(point.y);
}

}  // namespace

ObservationConfirmation::ObservationConfirmation(ObservationConfirmationConfig config)
    : config_(config) {
    config_.required_count = std::max<std::size_t>(1, config_.required_count);
    config_.cluster_radius_m = std::max(0.0, config_.cluster_radius_m);
    config_.maximum_gap_s = std::max(0.0, config_.maximum_gap_s);
}

void ObservationConfirmation::reset() {
    stream_confirmed_ = false;
    last_sample_ns_ = 0;
    pending_count_ = 0;
    last_accepted_ = {};
    pending_center_ = {};
}

bool ObservationConfirmation::observe(
    const Point2d& world_position,
    std::int64_t timestamp_ns) {
    if (!finite(world_position) || timestamp_ns <= 0) {
        reset();
        return false;
    }

    if (last_sample_ns_ > 0) {
        const double gap_s = static_cast<double>(timestamp_ns - last_sample_ns_) *
            NS_TO_SECONDS;
        if (gap_s <= 0.0 ||
            (config_.maximum_gap_s > 0.0 && gap_s > config_.maximum_gap_s)) {
            stream_confirmed_ = false;
            pending_count_ = 0;
        }
    }
    last_sample_ns_ = timestamp_ns;

    if (stream_confirmed_ &&
        distance(world_position, last_accepted_) <= config_.cluster_radius_m) {
        last_accepted_ = world_position;
        pending_count_ = 0;
        return true;
    }

    // 已确认流中的突然跳点不能直接成为新的可靠观测；从该点重新累计。
    if (stream_confirmed_) {
        stream_confirmed_ = false;
        pending_count_ = 0;
    }

    if (pending_count_ == 0 ||
        distance(world_position, pending_center_) > config_.cluster_radius_m) {
        pending_center_ = world_position;
        pending_count_ = 1;
    } else {
        const double count = static_cast<double>(pending_count_);
        pending_center_.x = (pending_center_.x * count + world_position.x) /
            (count + 1.0);
        pending_center_.y = (pending_center_.y * count + world_position.y) /
            (count + 1.0);
        ++pending_count_;
    }

    if (pending_count_ < config_.required_count) {
        return false;
    }

    stream_confirmed_ = true;
    last_accepted_ = pending_center_;
    pending_count_ = 0;
    return true;
}

}  // namespace position_prior
