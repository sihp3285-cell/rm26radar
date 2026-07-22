#include <gtest/gtest.h>

#include "robot_id.hpp"
#include "tracker.hpp"
#include "tracker_message.hpp"

#include <cmath>
#include <cstdint>
#include <vector>

namespace {

TrackerParams test_params() {
    TrackerParams params;
    params.max_predict_time_s = 0.20f;
    params.max_lost_time_s = 0.50f;
    params.dead_retention_time_s = 2.0f;
    params.min_hit = 1;
    params.max_tracks = 4;
    params.max_gate_box = 10000.0f;
    params.max_gate_world = 100.0f;
    params.kalman_gate_box = -1.0f;
    params.kalman_gate_world = -1.0f;
    params.identity_confirm_observations = 1;
    params.identity_switch_confirm_observations = 1;
    params.slot_bind_min_conf = 0.0f;
    params.slot_lease_time_s = 2.0f;
    params.slot_min_stability = 0.0f;
    params.slot_max_switch_rate = 1.0f;
    params.max_slot_jump_dist = -1.0f;
    params.botIdentity.maxHistory = 10;
    params.botIdentity.minHistoryForStable = 1;
    params.botIdentity.purgeAfterLostTimeS = 1.0f;
    return params;
}

WorldMeasurement measurement(float x, float z, float score = 0.8f) {
    WorldMeasurement result;
    result.class_id = robot_id::R1;
    result.team_id = robot_id::RED;
    result.score = score;
    result.class_conf = 1.0f;
    result.class_margin = 1.0f;
    result.box = cv::Rect(100, 100, 40, 30);
    result.world = cv::Point2f(x, z);
    return result;
}

constexpr std::int64_t seconds_to_ns(double seconds) {
    return static_cast<std::int64_t>(seconds * 1000000000.0);
}

}  // namespace

TEST(TrackerState, LifecycleUsesElapsedTimeAndKeepsMetadata) {
    Tracker tracker(test_params());
    tracker.update({measurement(1.0f, 2.0f)}, 0.05f, seconds_to_ns(1.0));

    auto slot = tracker.get_slot(Tracker::SLOT_RED_R1);
    ASSERT_TRUE(slot.valid);
    EXPECT_EQ(slot.state, TrackState::ACTIVE);
    EXPECT_TRUE(slot.observed);
    EXPECT_EQ(slot.position_source, PositionSource::TRACKED);
    EXPECT_FLOAT_EQ(slot.lost_duration_s, 0.0f);
    EXPECT_FLOAT_EQ(slot.detection_confidence, 0.8f);
    EXPECT_FLOAT_EQ(slot.tracking_confidence, 0.8f);

    tracker.update({}, 0.10f, seconds_to_ns(1.10));
    slot = tracker.get_slot(Tracker::SLOT_RED_R1);
    EXPECT_TRUE(slot.valid);
    EXPECT_EQ(slot.state, TrackState::PREDICTED);
    EXPECT_FALSE(slot.observed);
    EXPECT_EQ(slot.position_source, PositionSource::PREDICTED);
    EXPECT_NEAR(slot.lost_duration_s, 0.10f, 1e-5f);
    EXPECT_LT(slot.tracking_confidence, slot.detection_confidence);

    tracker.update({}, 0.16f, seconds_to_ns(1.26));
    slot = tracker.get_slot(Tracker::SLOT_RED_R1);
    EXPECT_FALSE(slot.valid);
    EXPECT_EQ(slot.state, TrackState::LOST);
    EXPECT_GE(slot.track_id, 0);
    EXPECT_NEAR(slot.lost_duration_s, 0.26f, 1e-5f);

    tensorrt_detect_msgs::msg::WorldTarget lost_message;
    tracker_message::fill_world_target(
        Tracker::SLOT_RED_R1, slot, lost_message);
    EXPECT_FALSE(lost_message.valid);
    EXPECT_EQ(lost_message.tracking_state,
              tensorrt_detect_msgs::msg::WorldTarget::TRACKING_LOST);
    EXPECT_EQ(lost_message.position_source,
              tensorrt_detect_msgs::msg::WorldTarget::POSITION_PREDICTED);
    EXPECT_EQ(lost_message.last_observed_time.sec, 1);

    tracker.update({}, 0.30f, seconds_to_ns(1.56));
    slot = tracker.get_slot(Tracker::SLOT_RED_R1);
    EXPECT_FALSE(slot.valid);
    EXPECT_EQ(slot.state, TrackState::DEAD);
    EXPECT_GE(slot.track_id, 0);
    EXPECT_FLOAT_EQ(slot.tracking_confidence, 0.0f);
}

TEST(TrackerState, LifecycleIsIndependentOfCallbackCount) {
    Tracker low_rate(test_params());
    Tracker high_rate(test_params());
    const auto initial = measurement(3.0f, 4.0f);
    low_rate.update({initial}, 0.05f, seconds_to_ns(2.0));
    high_rate.update({initial}, 0.05f, seconds_to_ns(2.0));

    low_rate.update({}, 0.26f, seconds_to_ns(2.26));
    for (int i = 1; i <= 13; ++i) {
        high_rate.update({}, 0.02f, seconds_to_ns(2.0 + i * 0.02));
    }

    const auto low_slot = low_rate.get_slot(Tracker::SLOT_RED_R1);
    const auto high_slot = high_rate.get_slot(Tracker::SLOT_RED_R1);
    EXPECT_EQ(low_slot.state, TrackState::LOST);
    EXPECT_EQ(high_slot.state, TrackState::LOST);
    EXPECT_NEAR(low_slot.lost_duration_s, high_slot.lost_duration_s, 1e-5f);
}

TEST(TrackerState, ReacquisitionAndClockRollbackResetTemporalState) {
    Tracker tracker(test_params());
    tracker.update({measurement(1.0f, 1.0f)}, 0.05f, seconds_to_ns(5.0));
    tracker.update({}, 0.10f, seconds_to_ns(5.10));
    tracker.update({measurement(1.1f, 1.0f)}, 0.05f, seconds_to_ns(5.15));

    auto slot = tracker.get_slot(Tracker::SLOT_RED_R1);
    ASSERT_TRUE(slot.valid);
    EXPECT_EQ(slot.state, TrackState::ACTIVE);
    EXPECT_TRUE(slot.observed);
    EXPECT_FLOAT_EQ(slot.lost_duration_s, 0.0f);
    EXPECT_EQ(slot.last_observed_time_ns, seconds_to_ns(5.15));

    tracker.update({measurement(7.0f, 8.0f)}, 0.05f, seconds_to_ns(4.0));
    slot = tracker.get_slot(Tracker::SLOT_RED_R1);
    ASSERT_TRUE(slot.valid);
    EXPECT_EQ(slot.state, TrackState::ACTIVE);
    EXPECT_TRUE(slot.observed);
    EXPECT_FLOAT_EQ(slot.lost_duration_s, 0.0f);
    EXPECT_EQ(slot.last_observed_time_ns, seconds_to_ns(4.0));
}

TEST(TrackerState, DuplicateTimestampDoesNotAdvancePrediction) {
    Tracker tracker(test_params());
    tracker.update({measurement(1.0f, 2.0f)}, 0.10f, seconds_to_ns(8.0));
    tracker.update({measurement(1.5f, 2.0f)}, 0.10f, seconds_to_ns(8.1));
    const auto before = tracker.get_slot(Tracker::SLOT_RED_R1);

    tracker.update({}, 0.0f, seconds_to_ns(8.1));
    const auto after = tracker.get_slot(Tracker::SLOT_RED_R1);
    EXPECT_EQ(after.state, TrackState::PREDICTED);
    EXPECT_FLOAT_EQ(after.lost_duration_s, 0.0f);
    EXPECT_NEAR(after.smoothed_world.x, before.smoothed_world.x, 1e-6f);
    EXPECT_NEAR(after.smoothed_world.y, before.smoothed_world.y, 1e-6f);
}

TEST(TrackerState, VelocityCovarianceAndRosMessageAreComplete) {
    Tracker tracker(test_params());
    tracker.update({measurement(1.0f, 2.0f)}, 0.10f, seconds_to_ns(10.0));
    tracker.update({measurement(1.2f, 2.1f)}, 0.10f, seconds_to_ns(10.1));

    const auto slot = tracker.get_slot(Tracker::SLOT_RED_R1);
    ASSERT_TRUE(slot.valid);
    ASSERT_TRUE(slot.covariance_valid);
    for (double value : slot.state_covariance) {
        EXPECT_TRUE(std::isfinite(value));
    }
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            EXPECT_NEAR(
                slot.state_covariance[row * 4 + col],
                slot.state_covariance[col * 4 + row],
                1e-5);
        }
        EXPECT_GE(slot.state_covariance[row * 4 + row], 0.0);
    }

    tensorrt_detect_msgs::msg::WorldTarget message;
    tracker_message::fill_world_target(Tracker::SLOT_RED_R1, slot, message);
    EXPECT_EQ(message.idx, Tracker::SLOT_RED_R1);
    EXPECT_EQ(message.class_id, robot_id::R1);
    EXPECT_EQ(message.team_id, robot_id::RED);
    EXPECT_EQ(message.track_id, slot.track_id);
    EXPECT_EQ(message.tracking_state,
              tensorrt_detect_msgs::msg::WorldTarget::TRACKING_ACTIVE);
    EXPECT_EQ(message.position_source,
              tensorrt_detect_msgs::msg::WorldTarget::POSITION_TRACKED);
    EXPECT_TRUE(message.observed);
    EXPECT_TRUE(message.covariance_valid);
    EXPECT_FLOAT_EQ(message.score, message.detection_confidence);
    EXPECT_EQ(message.last_observed_time.sec, 10);
    EXPECT_EQ(message.last_observed_time.nanosec, 100000000u);
}
