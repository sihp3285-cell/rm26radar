#include "position_prior/team_selection.hpp"

#include <gtest/gtest.h>

namespace position_prior {
namespace {

TEST(TeamSelectionTest, BlueViewOnlySelectsRedAsOpponent) {
    EXPECT_EQ(own_team_for_view(false), TEAM_BLUE);
    EXPECT_EQ(opponent_team_for_view(false), TEAM_RED);
    EXPECT_TRUE(is_opponent_team(TEAM_RED, false));
    EXPECT_FALSE(is_opponent_team(TEAM_BLUE, false));
    EXPECT_FALSE(is_opponent_team(TEAM_UNKNOWN, false));
}

TEST(TeamSelectionTest, RedViewOnlySelectsBlueAsOpponent) {
    EXPECT_EQ(own_team_for_view(true), TEAM_RED);
    EXPECT_EQ(opponent_team_for_view(true), TEAM_BLUE);
    EXPECT_TRUE(is_opponent_team(TEAM_BLUE, true));
    EXPECT_FALSE(is_opponent_team(TEAM_RED, true));
    EXPECT_FALSE(is_opponent_team(TEAM_UNKNOWN, true));
}

}  // namespace
}  // namespace position_prior
