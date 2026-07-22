#pragma once

#include "tracker.hpp"
#include "tensorrt_detect_msgs/msg/world_target.hpp"

namespace tracker_message {

// 将 official slot 输出转换成 ROS2 消息。保留旧字段语义，并补齐 tracker 状态字段。
void fill_world_target(
    int slot_idx,
    const Tracker::SlotOutput& slot,
    tensorrt_detect_msgs::msg::WorldTarget& target);

// 为不经过 Tracker 的直接测量（前哨站、死亡装甲板）补齐来源字段。
void mark_direct_measurement(
    tensorrt_detect_msgs::msg::WorldTarget& target,
    bool observed);

}  // namespace tracker_message
