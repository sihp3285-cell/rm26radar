/**
 * @file robot_id.hpp
 * @brief 检测、跟踪、地图和消息层共享的类别/队伍整数编码。
 *
 * 整数数值的唯一来源是 rm_field/robot_class.hpp（rm_field 包），本文件只做
 * 兼容重导出与显示/绘制辅助，避免在多个模块里各自维护一份 class_id/team_id
 * 常量表。数值不要与 track_id、slot_idx 或 prior grid_index 混用。
 */
#pragma once

#include <rm_field/robot_class.hpp>

#include <opencv2/opencv.hpp>
#include <string>

namespace robot_id {

using TeamId = int;
using ClassId = int;

// team_id 语义（与 armor_color 对齐）
inline constexpr int UNKNOWN = rm_field::kTeamUnknown;
inline constexpr int RED     = rm_field::kTeamRed;
inline constexpr int BLUE    = rm_field::kTeamBlue;

// class_id 语义（与 model.yaml classNames 对齐）
inline constexpr int CAR      = rm_field::kClassCar;
inline constexpr int ARMOR    = rm_field::kClassArmor;
inline constexpr int R1       = rm_field::kClassHero;       // 1号机器人
inline constexpr int R2       = rm_field::kClassEngineer;   // 2号机器人
inline constexpr int R3       = rm_field::kClassInfantry3;  // 3号机器人
inline constexpr int R4       = rm_field::kClassInfantry4;  // 4号机器人
inline constexpr int S        = rm_field::kClassSentry;     // 哨兵
inline constexpr int OUTPOST  = rm_field::kClassOutpost;
inline constexpr int AIRPLANE = rm_field::kClassAirplane;

/** 按地图视角返回我方 team_id：false=蓝方视角、true=红方视角。 */
inline constexpr int own_team_for_view(bool flip_team) {
    return rm_field::own_team_for_view(flip_team);
}

/** 按地图视角返回敌方 team_id（先验只预测敌方）。 */
inline constexpr int opponent_team_for_view(bool flip_team) {
    return rm_field::opponent_team_for_view(flip_team);
}

/** 将 team_id 转成调试标签用英文队伍名，未知值返回问号。 */
inline std::string getTeamName(int team_id) {
    switch (team_id) {
        case RED:  return "red";
        case BLUE: return "blue";
        default:   return "?";
    }
}

/** 将兵种 class_id 转成官方编号字符；非 R1-R4/S 返回空串。 */
inline std::string getRobotNumber(int class_id) {
    switch (class_id) {
        case R1: return "1";
        case R2: return "2";
        case R3: return "3";
        case R4: return "4";
        case S:  return "S";
        default: return "";
    }
}

/** 组合队伍与编号生成完整调试标签；任一 ID 无效返回问号。 */
inline std::string getRobotLabel(int team_id, int class_id) {
    if (team_id == UNKNOWN) {
        return "?";
    }
    std::string num = getRobotNumber(class_id);
    if (num.empty()) {
        return "?";
    }
    return getTeamName(team_id) + num;
}

/** 返回 OpenCV BGR 绘制颜色；未知队伍使用黄色以突出数据异常。 */
inline cv::Scalar getTeamColor(int team_id) {
    switch (team_id) {
        case RED:  return cv::Scalar(0, 0, 255);    // 红色
        case BLUE: return cv::Scalar(255, 0, 0);    // 蓝色
        default:   return cv::Scalar(0, 255, 255);  // 黄色（未知）
    }
}

} // namespace robot_id
