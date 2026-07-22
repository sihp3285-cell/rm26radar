#include "position_prior/prior_lifecycle.hpp"

#include <cmath>

namespace position_prior {

PriorCacheAction decide_prior_cache_action(const PriorLifecycleSample& sample) {
    if (sample.robot_confirmed_dead) {
        return PriorCacheAction::CLEAR_CONFIRMED_DEAD;
    }
    if (sample.reliable_observation) {
        return PriorCacheAction::UPDATE_OBSERVATION;
    }

    if (!std::isfinite(sample.lost_duration_s) || sample.lost_duration_s < 0.0) {
        return PriorCacheAction::CLEAR_INVALID_TIME;
    }

    // tracker_lifecycle_ended 刻意不参与清理条件。先验从最后一次可靠观测
    // 独立计时，并在 tracker 释放物理轨迹后持续工作，直到重新观测。
    return PriorCacheAction::KEEP;
}

}  // namespace position_prior
