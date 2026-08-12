/**
 * @file team_selection.hpp
 * @brief /flip_team 视角下的敌我队伍选择规则。
 * false 表示我方蓝、预测红方；true 表示我方红、预测蓝方，与 MapNode 保持一致。
 */
#pragma once

namespace position_prior {

constexpr int TEAM_UNKNOWN = 0;
constexpr int TEAM_RED = 1;
constexpr int TEAM_BLUE = 2;

/** 按地图视角返回我方 team_id：false 为蓝方、true 为红方。 */
constexpr int own_team_for_view(bool flip_team) {
    return flip_team ? TEAM_RED : TEAM_BLUE;
}

/** 按地图视角返回需要位置先验预测的敌方 team_id。 */
constexpr int opponent_team_for_view(bool flip_team) {
    return flip_team ? TEAM_BLUE : TEAM_RED;
}

/** 判断输入 team_id 是否为当前视角的敌方；未知队伍恒为 false。 */
constexpr bool is_opponent_team(int team_id, bool flip_team) {
    return team_id == opponent_team_for_view(flip_team);
}

}  // namespace position_prior
