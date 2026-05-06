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
};

// 队伍名称
inline std::string getTeamName(int team_id) {
    switch (team_id) {
        case RED:  return "red";
        case BLUE: return "blue";
        default:   return "?";
    }
}

// 机器人编号字符
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

// 完整标签：红3、蓝S、?
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

// 队伍绘制颜色（BGR）
inline cv::Scalar getTeamColor(int team_id) {
    switch (team_id) {
        case RED:  return cv::Scalar(0, 0, 255);    // 红色
        case BLUE: return cv::Scalar(255, 0, 0);    // 蓝色
        default:   return cv::Scalar(0, 255, 255);  // 黄色（未知）
    }
}

} // namespace robot_id
