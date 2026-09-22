#pragma once
#include <rm_field/robot_id.hpp>
#include <opencv2/core.hpp>
namespace robot_id {
inline cv::Scalar getTeamColor(int team_id) {
    switch (team_id) {
        case RED:  return cv::Scalar(0, 0, 255);    // 红色
        case BLUE: return cv::Scalar(255, 0, 0);    // 蓝色
        default:   return cv::Scalar(0, 255, 255);  // 黄色（未知）
    }
}
}
