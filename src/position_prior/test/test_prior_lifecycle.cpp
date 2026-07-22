#include "position_prior/prior_lifecycle.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace position_prior {
namespace {

TEST(PriorLifecycleTest, TrackerDeadKeepsCacheUntilReobserved) {
    PriorLifecycleSample sample;
    sample.tracker_lifecycle_ended = true;

    sample.lost_duration_s = 0.31;
    EXPECT_EQ(
        decide_prior_cache_action(sample),
        PriorCacheAction::KEEP);

    sample.lost_duration_s = 2.0;
    EXPECT_EQ(
        decide_prior_cache_action(sample),
        PriorCacheAction::KEEP);

    sample.lost_duration_s = 12.0;
    EXPECT_EQ(
        decide_prior_cache_action(sample),
        PriorCacheAction::KEEP);

    sample.lost_duration_s = 3600.0;
    EXPECT_EQ(
        decide_prior_cache_action(sample),
        PriorCacheAction::KEEP);
}

TEST(PriorLifecycleTest, ConfirmedRobotDeathClearsImmediately) {
    PriorLifecycleSample sample;
    sample.robot_confirmed_dead = true;
    sample.lost_duration_s = 0.1;
    EXPECT_EQ(
        decide_prior_cache_action(sample),
        PriorCacheAction::CLEAR_CONFIRMED_DEAD);
}

TEST(PriorLifecycleTest, ReliableObservationRefreshesLongLivedCache) {
    PriorLifecycleSample sample;
    sample.reliable_observation = true;
    sample.tracker_lifecycle_ended = false;
    sample.lost_duration_s = 20.0;
    EXPECT_EQ(
        decide_prior_cache_action(sample),
        PriorCacheAction::UPDATE_OBSERVATION);
}

TEST(PriorLifecycleTest, InvalidElapsedTimeFailsClosed) {
    PriorLifecycleSample sample;
    sample.lost_duration_s = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(
        decide_prior_cache_action(sample),
        PriorCacheAction::CLEAR_INVALID_TIME);
}

}  // namespace
}  // namespace position_prior
