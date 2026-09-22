#pragma once
#include <opencv2/core.hpp>
struct Result
{
    int idx = 0;                 // robot_id 语义类别，不是 Tracker track_id。
    float confidence = 0.0f;    // 当前检测分支的目标置信度。
    cv::Rect box;                // 当前目标像素框；最终通常为装甲板框。
    int armorColor = 0;          // 装甲板队伍颜色，后续成为 WorldMeasurement.team_id。
    cv::Rect car_box{};          // 所属车辆框，PoseNode 可用其底边估计落地点。
    bool isDead = false;
    float class_conf = -1.0f;
    float class_margin = -1.0f;
};
