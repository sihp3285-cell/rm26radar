/**
 * @file robot_id.hpp
 * @brief 检测、跟踪、地图和消息层共享的类别/队伍整数编码。
 * 这些是 class/team ID；不要与 track_id、slot_idx 或 prior grid_index 混用。
 */
#pragma once

#include <opencv2/opencv.hpp>
#include <string>

namespace robot_id {

// team_id 语义（与 armor_color 对齐）
enum TeamId {
    UNKNOWN = 0,  // 未知/无效
    RED     = 1,  // 红方
    BLUE    = 2,  // 蓝方
};

// class_id 语义（与 classNames 对齐）
enum ClassId {
    CAR   = 0,  // 车辆
    ARMOR = 1,  // 未分类装甲板
    R1    = 2,  // 1号机器人
    R2    = 3,  // 2号机器人
    R3    = 4,  // 3号机器人
    R4    = 5,  // 4号机器人
    S     = 6,  // 哨兵
    OUTPOST = 7,  // 前哨站
    AIRPLANE = 8, // 无人机
};

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
