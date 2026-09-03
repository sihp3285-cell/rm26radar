/**
 * @file team_selection.hpp
 * @brief /flip_team 视角下的敌我队伍选择规则（薄封装）。
 *
 * 数值与公式的唯一来源是 rm_field/robot_class.hpp：false 表示我方蓝、预测红方；
 * true 表示我方红、预测蓝方，与 MapNode 保持一致。本文件仅为 position_prior
 * 旧调用点保留 position_prior::* 名称，不再各自维护一份逻辑。
 */
#pragma once

#include <rm_field/robot_class.hpp>

namespace position_prior {

inline constexpr int TEAM_UNKNOWN = rm_field::kTeamUnknown;
inline constexpr int TEAM_RED = rm_field::kTeamRed;
inline constexpr int TEAM_BLUE = rm_field::kTeamBlue;

/** 按地图视角返回我方 team_id：false 为蓝方、true 为红方。 */
inline constexpr int own_team_for_view(bool flip_team) {
    return rm_field::own_team_for_view(flip_team);
}

/** 按地图视角返回需要位置先验预测的敌方 team_id。 */
inline constexpr int opponent_team_for_view(bool flip_team) {
    return rm_field::opponent_team_for_view(flip_team);
}

/** 判断输入 team_id 是否为当前视角的敌方；未知队伍恒为 false。 */
inline constexpr bool is_opponent_team(int team_id, bool flip_team) {
    return rm_field::is_opponent_team(team_id, flip_team);
}

}  // namespace position_prior
