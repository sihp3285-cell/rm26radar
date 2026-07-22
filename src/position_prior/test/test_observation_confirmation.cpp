#include "position_prior/observation_confirmation.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace position_prior {
namespace {

TEST(ObservationConfirmationTest, RequiresRepeatedNearbyDetections) {
    ObservationConfirmation filter({3, 0.8, 0.35});
    EXPECT_FALSE(filter.observe({1.0, 2.0}, 1000000000LL));
    EXPECT_FALSE(filter.observe({1.2, 2.1}, 1100000000LL));
    EXPECT_TRUE(filter.observe({0.9, 2.2}, 1200000000LL));
    EXPECT_TRUE(filter.stream_confirmed());
}

TEST(ObservationConfirmationTest, SuddenJumpCannotReplaceConfirmedRegion) {
    ObservationConfirmation filter({3, 0.8, 0.35});
    EXPECT_FALSE(filter.observe({1.0, 1.0}, 1000000000LL));
    EXPECT_FALSE(filter.observe({1.1, 1.0}, 1100000000LL));
    EXPECT_TRUE(filter.observe({1.0, 1.1}, 1200000000LL));

    EXPECT_FALSE(filter.observe({8.0, 8.0}, 1300000000LL));
    EXPECT_EQ(filter.pending_count(), 1u);
    EXPECT_FALSE(filter.observe({1.0, 1.0}, 1400000000LL));
    EXPECT_FALSE(filter.observe({1.1, 1.0}, 1500000000LL));
    EXPECT_TRUE(filter.observe({1.0, 1.1}, 1600000000LL));
}

TEST(ObservationConfirmationTest, LongGapStartsNewConfirmationSequence) {
    ObservationConfirmation filter({3, 0.8, 0.35});
    EXPECT_FALSE(filter.observe({1.0, 1.0}, 1000000000LL));
    EXPECT_FALSE(filter.observe({1.0, 1.0}, 1100000000LL));
    EXPECT_TRUE(filter.observe({1.0, 1.0}, 1200000000LL));

    EXPECT_FALSE(filter.observe({1.0, 1.0}, 2000000000LL));
    EXPECT_FALSE(filter.observe({1.0, 1.0}, 2100000000LL));
    EXPECT_TRUE(filter.observe({1.0, 1.0}, 2200000000LL));
}

TEST(ObservationConfirmationTest, InvalidPointFailsClosedAndResets) {
    ObservationConfirmation filter({2, 0.8, 0.35});
    EXPECT_FALSE(filter.observe({1.0, 1.0}, 1000000000LL));
    EXPECT_FALSE(filter.observe(
        {std::numeric_limits<double>::quiet_NaN(), 1.0}, 1100000000LL));
    EXPECT_FALSE(filter.observe({1.0, 1.0}, 1200000000LL));
    EXPECT_TRUE(filter.observe({1.0, 1.0}, 1300000000LL));
}

}  // namespace
}  // namespace position_prior
