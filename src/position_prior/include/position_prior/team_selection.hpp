#pragma once

namespace position_prior {

constexpr int TEAM_UNKNOWN = 0;
constexpr int TEAM_RED = 1;
constexpr int TEAM_BLUE = 2;

// 与地图节点的视角定义保持一致：false 为蓝方视角，true 为红方视角。
constexpr int own_team_for_view(bool flip_team) {
    return flip_team ? TEAM_RED : TEAM_BLUE;
}

constexpr int opponent_team_for_view(bool flip_team) {
    return flip_team ? TEAM_BLUE : TEAM_RED;
}

constexpr bool is_opponent_team(int team_id, bool flip_team) {
    return team_id == opponent_team_for_view(flip_team);
}

}  // namespace position_prior
