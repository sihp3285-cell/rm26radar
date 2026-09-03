/**
 * @file tracker_message.hpp
 * @brief Tracker SlotOutput 到 ROS WorldTarget 数据契约的集中转换。
 * 这里统一展开 covariance、时间、PositionSource 与 tracking state 字段语义。
 */
#pragma once

#include "tracker.hpp"
#include "tensorrt_detect_msgs/msg/world_target.hpp"

namespace tracker_message {

/** 将 official slot 输出转换成 ROS2 消息，补齐 tracker 状态、时间和协方差字段。 */
void fill_world_target(
    int slot_idx,
    const Tracker::SlotOutput& slot,
    tensorrt_detect_msgs::msg::WorldTarget& target);

/** 为不经过 Tracker 的前哨站/死亡装甲板直接测量补齐来源和观测状态字段。
 *  detection_score 是该直通项的检测置信度；observed=false 时记 0。 */
void mark_direct_measurement(
    tensorrt_detect_msgs::msg::WorldTarget& target,
    bool observed,
    float detection_score);

}  // namespace tracker_message
