#pragma once
#include <opencv2/opencv.hpp>

// ==========================================
// Tracker 公共数据结构
// ==========================================

enum class TrackState {
    ACTIVE = 0,    // 正常跟踪中
    PREDICTED = 1, // 短暂丢失，卡尔曼外推中（仍显示在地图上）
    LOST = 2,      // 丢失较久，不再对外输出
    DEAD = 3       // 超过最大丢失帧，已删除
};

// 单帧观测输入（像素框 + 世界坐标）
struct WorldMeasurement {
    int class_id = 0;
    int team_id = 0;
    float score = 0.0f;
    bool is_dead = false;
    cv::Rect box;          // 像素框 [x, y, w, h]
    cv::Point2f world;     // world.x = world_x, world.y = world_z (world_y 始终为 0)
};

// 跟踪目标输出
struct TrackedTarget {
    int track_id = 0;      // 全局唯一跟踪 ID
    int team_id = 0;
    int class_id = 0;
    int hit_count = 0;     // 连续命中次数
    int miss_count = 0;    // 连续丢失次数
    TrackState state = TrackState::ACTIVE;

    cv::Rect smoothed_box;       // 平滑后的像素框
    cv::Point2f smoothed_world;  // 平滑后的世界坐标 (x=world_x, y=world_z)
    bool is_dead = false;
    float score = 0.0f;
};
