#include "tracker_message.hpp"

#include <algorithm>
#include <cstdint>

namespace {

builtin_interfaces::msg::Time to_builtin_time(std::int64_t timestamp_ns) {
    builtin_interfaces::msg::Time result;
    if (timestamp_ns <= 0) {
        return result;
    }

    constexpr std::int64_t ns_per_second = 1000000000LL;
    result.sec = static_cast<std::int32_t>(timestamp_ns / ns_per_second);
    result.nanosec = static_cast<std::uint32_t>(timestamp_ns % ns_per_second);
    return result;
}

}  // namespace

namespace tracker_message {

void fill_world_target(
    int slot_idx,
    const Tracker::SlotOutput& slot,
    tensorrt_detect_msgs::msg::WorldTarget& target) {
    target.idx = slot_idx;
    target.class_id = slot.class_id;
    target.team_id = slot.team_id;
    target.is_dead = slot.is_dead;
    target.score = slot.score;
    target.valid = slot.valid;
    target.bbox_x = slot.smoothed_box.x;
    target.bbox_y = slot.smoothed_box.y;
    target.bbox_w = slot.smoothed_box.width;
    target.bbox_h = slot.smoothed_box.height;
    target.world_x = slot.smoothed_world.x;
    target.world_y = 0.0f;
    target.world_z = slot.smoothed_world.y;
    target.stable_class_id = slot.stable_class_id;
    target.stable_class_conf = slot.stable_class_conf;

    target.track_id = slot.track_id;
    target.tracking_state = static_cast<std::uint8_t>(slot.state);
    target.position_source = static_cast<std::uint8_t>(slot.position_source);
    target.observed = slot.observed;
    target.velocity_x = slot.velocity.x;
    target.velocity_y = 0.0f;
    target.velocity_z = slot.velocity.y;
    target.state_covariance = slot.state_covariance;
    target.covariance_valid = slot.covariance_valid;
    target.detection_confidence = slot.detection_confidence;
    target.tracking_confidence = slot.tracking_confidence;
    target.last_observed_time = to_builtin_time(slot.last_observed_time_ns);
    target.lost_duration_s = std::max(0.0f, slot.lost_duration_s);
}

void mark_direct_measurement(
    tensorrt_detect_msgs::msg::WorldTarget& target,
    bool observed) {
    target.track_id = -1;
    target.tracking_state = observed
        ? tensorrt_detect_msgs::msg::WorldTarget::TRACKING_ACTIVE
        : tensorrt_detect_msgs::msg::WorldTarget::TRACKING_INVALID;
    target.position_source = observed
        ? tensorrt_detect_msgs::msg::WorldTarget::POSITION_MEASURED
        : tensorrt_detect_msgs::msg::WorldTarget::POSITION_INVALID;
    target.observed = observed;
    target.covariance_valid = false;
    target.detection_confidence = target.score;
    target.tracking_confidence = observed ? target.score : 0.0f;
    target.lost_duration_s = 0.0f;
}

}  // namespace tracker_message
