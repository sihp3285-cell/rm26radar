/**
 * @file prior_lifecycle.hpp
 * @brief 判定 TargetCache 在 Tracker 状态变化时应保留、更新还是清除。
 *
 * Tracker DEAD/INVALID 可能只是短时跟踪生命周期结束，不等价于机器人被确认死亡；
 * 因而最后可靠观测通常必须继续保存，直到重观测、明确死亡或输入时间非法。
 */
#pragma once

namespace position_prior {

enum class PriorCacheAction {
    KEEP,
    UPDATE_OBSERVATION,
    CLEAR_CONFIRMED_DEAD,
    CLEAR_INVALID_TIME,
};

struct PriorLifecycleSample {
    // 真实死亡语义，与 tracker 因漏检结束生命周期不同。
    bool robot_confirmed_dead = false;
    bool reliable_observation = false;
    bool tracker_lifecycle_ended = false;
    double lost_duration_s = 0.0;
};

/**
 * 根据真实死亡、可靠重观测、Tracker 生命周期和时间合法性返回缓存动作。
 * Tracker DEAD/INVALID 本身只返回 KEEP，不会误删最后可靠位置。
 */
PriorCacheAction decide_prior_cache_action(const PriorLifecycleSample& sample);

}  // namespace position_prior
