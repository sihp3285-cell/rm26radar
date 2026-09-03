/**
 * @file robot_class.hpp
 * @brief RM 场地角色/队伍 ID 与 class_id→role key 的唯一契约。
 *
 * 全仓库（tensorrt_detect 的 robot_id.hpp、position_prior 的 role 查询、
 * model.yaml 的 classNames 下标、离线工具 role 字符串）都以此文件为准。
 * 这些值是与检测模型输出、ROS 消息和 NavGrid profile 对齐的整数契约，
 * 不是 track_id / slot_idx / grid_index。
 */
#pragma once

namespace rm_field {

// ── team_id（与 armor_color 对齐）──
inline constexpr int kTeamUnknown = 0;
inline constexpr int kTeamRed = 1;
inline constexpr int kTeamBlue = 2;

// ── class_id（与 DetectionBox.idx / robot_id 兼容）──
inline constexpr int kClassCar = 0;        // 车辆
inline constexpr int kClassArmor = 1;      // 未分类装甲板
inline constexpr int kClassHero = 2;       // 1 号机器人
inline constexpr int kClassEngineer = 3;   // 2 号机器人
inline constexpr int kClassInfantry3 = 4;  // 3 号机器人
inline constexpr int kClassInfantry4 = 5;  // 4 号机器人
inline constexpr int kClassSentry = 6;     // 哨兵
inline constexpr int kClassOutpost = 7;    // 前哨站
inline constexpr int kClassAirplane = 8;   // 无人机

/** 按地图视角返回我方 team_id：false=蓝方视角、true=红方视角。 */
inline constexpr int own_team_for_view(bool flip_team) {
    return flip_team ? kTeamRed : kTeamBlue;
}

/** 按地图视角返回敌方 team_id（先验只预测敌方）。 */
inline constexpr int opponent_team_for_view(bool flip_team) {
    return flip_team ? kTeamBlue : kTeamRed;
}

/** 判断输入 team_id 是否为当前视角的敌方；未知队伍恒为 false。 */
inline constexpr bool is_opponent_team(int team_id, bool flip_team) {
    return team_id == opponent_team_for_view(flip_team);
}

/** 在线模型/NavGrid 支持的兵种 role key（唯一字符串来源）。 */
inline const char* const kRoleKeys[] = {
    "hero", "engineer", "infantry3", "infantry4", "sentry",
};

/** class_id→Position Prior 模型/NavGrid 的 role key；非兵种返回空串。 */
inline const char* role_key_for_class(int class_id) {
    switch (class_id) {
        case kClassHero:       return kRoleKeys[0];
        case kClassEngineer:   return kRoleKeys[1];
        case kClassInfantry3:  return kRoleKeys[2];
        case kClassInfantry4:  return kRoleKeys[3];
        case kClassSentry:     return kRoleKeys[4];
        default:               return "";
    }
}

/** 判断字符串是否属于 kRoleKeys 中的合法兵种 role key。 */
inline bool is_role_key(const char* role) {
    if (role == nullptr) return false;
    for (const char* const key : kRoleKeys) {
        bool same = true;
        for (int i = 0; ; ++i) {
            if (role[i] != key[i]) { same = false; break; }
            if (role[i] == '\0') break;
        }
        if (same) return true;
    }
    return false;
}

}  // namespace rm_field
