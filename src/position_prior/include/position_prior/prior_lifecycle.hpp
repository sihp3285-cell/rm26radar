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

// tracker 进入 DEAD/INVALID 只表示短期跟踪结束，不能清除最后一次可靠观测。
// 缓存持续保留，直到重新真实观测、确认死亡或输入时间非法。
PriorCacheAction decide_prior_cache_action(const PriorLifecycleSample& sample);

}  // namespace position_prior
