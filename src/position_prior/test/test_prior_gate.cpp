#include "position_prior/prior_gate.hpp"

#include <gtest/gtest.h>

namespace position_prior {
namespace {

PriorDistribution distribution(std::initializer_list<PriorCandidate> candidates) {
    PriorDistribution value;
    value.valid = true;
    value.horizon_seconds = 2;
    value.fallback_level = FallbackLevel::LOCAL_ZONE;
    value.local_weight = 1.0;
    value.normalized_entropy = 0.0;
    value.candidates.assign(candidates);
    return value;
}

TEST(PriorGateTest, RejectsCandidatesBeyondPhysicalReach) {
    PriorGateConfig config;
    config.max_speed_mps = 1.0;
    config.reachability_margin_m = 0.0;
    config.minimum_confidence = 0.0;
    PriorGate gate(config);
    const auto result = gate.apply(
        distribution({{1, {10.0, 10.0}, 1.0}}),
        {0.0, 0.0}, {0.0, 0.0}, 2.0, 1.0);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.rejection_reason, "no_reachable_candidate");
}

TEST(PriorGateTest, ReachabilityUsesActualLostDurationNotModelHorizon) {
    PriorGateConfig config;
    config.max_speed_mps = 1.0;
    config.reachability_margin_m = 0.0;
    config.minimum_confidence = 0.0;
    PriorGate gate(config);
    // 分布是 2 秒档，但实际只丢失 0.5 秒，1 米候选不可达。
    const auto result = gate.apply(
        distribution({{1, {1.0, 0.0}, 1.0}}),
        {0.0, 0.0}, {0.0, 0.0}, 0.5, 1.0);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.rejection_reason, "no_reachable_candidate");
}

TEST(PriorGateTest, HardDistanceThresholdRejectsFarCandidateEvenWhenSpeedReachable) {
    PriorGateConfig config;
    config.max_speed_mps = 10.0;
    config.reachability_margin_m = 0.0;
    config.max_guess_distance_m = 3.0;
    config.minimum_confidence = 0.0;
    PriorGate gate(config);
    // 2 秒按速度可达 20 米，但绝对距离阈值只允许 3 米。
    const auto result = gate.apply(
        distribution({{1, {4.0, 0.0}, 1.0}}),
        {0.0, 0.0}, {1.0, 0.0}, 2.0, 1.0);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.rejection_reason, "no_reachable_candidate");
    ASSERT_EQ(result.candidates.size(), 1u);
    EXPECT_FALSE(result.candidates.front().reachable);
    EXPECT_DOUBLE_EQ(result.candidates.front().distance_from_last_m, 4.0);
}

TEST(PriorGateTest, HardDistanceThresholdKeepsNearCandidateAndDisablesFarCandidate) {
    PriorGateConfig config;
    config.max_speed_mps = 10.0;
    config.max_guess_distance_m = 3.0;
    config.minimum_confidence = 0.0;
    config.motion_gate_mps[2] = 0.1;
    PriorGate gate(config);
    const auto result = gate.apply(
        distribution({{1, {2.0, 0.0}, 0.2}, {2, {4.0, 0.0}, 0.8}}),
        {0.0, 0.0}, {1.0, 0.0}, 2.0, 1.0);
    ASSERT_TRUE(result.valid);
    ASSERT_EQ(result.candidates.size(), 2u);
    EXPECT_EQ(result.candidates.front().prior.grid_index, 1);
    EXPECT_TRUE(result.candidates.front().reachable);
    EXPECT_FALSE(result.candidates.back().reachable);
    EXPECT_NEAR(result.predicted_canonical.x, 2.0, 1e-9);
    EXPECT_NEAR(result.predicted_canonical.y, 0.0, 1e-9);
}

TEST(PriorGateTest, SoftDistancePreferenceFavorsNearCandidateWithinThreshold) {
    PriorGateConfig config;
    config.max_guess_distance_m = 6.0;
    config.distance_preference_sigma_m = 2.0;
    config.minimum_confidence = 0.0;
    PriorGate gate(config);
    const auto result = gate.apply(
        distribution({{1, {1.0, 0.0}, 0.5}, {2, {4.0, 0.0}, 0.5}}),
        {0.0, 0.0}, {0.0, 0.0}, 2.0, 1.0);
    ASSERT_TRUE(result.valid);
    ASSERT_EQ(result.candidates.size(), 2u);
    EXPECT_EQ(result.candidates.front().prior.grid_index, 1);
    EXPECT_GT(result.candidates.front().fused_probability,
              result.candidates.back().fused_probability);
    EXPECT_TRUE(result.candidates.front().reachable);
    EXPECT_TRUE(result.candidates.back().reachable);
}

TEST(PriorGateTest, RejectsConfiguredBlockedRegion) {
    PriorGateConfig config;
    config.minimum_confidence = 0.0;
    config.blocked_regions_canonical.push_back({1.0, 1.0, 3.0, 3.0});
    PriorGate gate(config);
    const auto result = gate.apply(
        distribution({{1, {2.0, 2.0}, 1.0}}),
        {2.0, 2.0}, {0.0, 0.0}, 2.0, 1.0);
    EXPECT_FALSE(result.valid);
    ASSERT_EQ(result.candidates.size(), 1u);
    EXPECT_TRUE(result.candidates.front().blocked);
}

TEST(PriorGateTest, LowSpeedUsesLastReliablePosition) {
    PriorGateConfig config;
    config.minimum_confidence = 0.0;
    PriorGate gate(config);
    const auto result = gate.apply(
        distribution({{1, {4.0, 4.0}, 0.6}, {2, {5.0, 5.0}, 0.4}}),
        {3.0, 3.0}, {0.0, 0.0}, 2.0, 1.0);
    ASSERT_TRUE(result.valid);
    EXPECT_FALSE(result.motion_gated);
    EXPECT_DOUBLE_EQ(result.predicted_canonical.x, 3.0);
    EXPECT_DOUBLE_EQ(result.predicted_canonical.y, 3.0);
}

TEST(PriorGateTest, VelocityGateBiasesReachableCandidates) {
    PriorGateConfig config;
    config.minimum_confidence = 0.0;
    config.motion_gate_mps[2] = 0.1;
    config.motion_sigma_m[2] = 0.25;
    PriorGate gate(config);
    const auto result = gate.apply(
        distribution({{1, {2.0, 0.0}, 0.5}, {2, {0.0, 2.0}, 0.5}}),
        {0.0, 0.0}, {1.0, 0.0}, 2.0, 1.0);
    ASSERT_TRUE(result.valid);
    EXPECT_TRUE(result.motion_gated);
    EXPECT_GT(result.candidates[0].fused_probability, 0.99);
    EXPECT_EQ(result.candidates[0].prior.grid_index, 1);
}

TEST(PriorGateTest, ConfidenceCanRejectUncertainPrior) {
    PriorGateConfig config;
    config.minimum_confidence = 0.2;
    PriorGate gate(config);
    auto input = distribution({{1, {1.0, 1.0}, 1.0}});
    input.local_weight = 0.1;
    const auto result = gate.apply(input, {1.0, 1.0}, {0.0, 0.0}, 10.0, 0.5);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.rejection_reason, "confidence_below_threshold");
}

TEST(PriorGateTest, ConfidenceDoesNotExpireWhileTargetRemainsLost) {
    PriorGateConfig config;
    config.max_speed_mps = 1.0;
    config.max_guess_distance_m = 4.0;
    config.minimum_confidence = 0.1;
    PriorGate gate(config);
    const auto input = distribution({{1, {1.0, 1.0}, 1.0}});

    const auto early = gate.apply(
        input, {1.0, 1.0}, {0.0, 0.0}, 2.0, 0.8);
    const auto long_lost = gate.apply(
        input, {1.0, 1.0}, {0.0, 0.0}, 3600.0, 0.8);

    ASSERT_TRUE(early.valid);
    ASSERT_TRUE(long_lost.valid);
    EXPECT_DOUBLE_EQ(long_lost.confidence, early.confidence);
    EXPECT_EQ(long_lost.predicted_canonical.x, early.predicted_canonical.x);
    EXPECT_EQ(long_lost.predicted_canonical.y, early.predicted_canonical.y);
}

}  // namespace
}  // namespace position_prior
