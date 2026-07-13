#pragma once

#include <opencv2/opencv.hpp>

// ==========================================
// Tracker 公共数据结构
// ==========================================

enum class TrackState {
    ACTIVE = 0,     // 正常跟踪中
    PREDICTED = 1,  // 短暂丢失，卡尔曼外推中（仍可显示在地图上）
    LOST = 2,       // 丢失较久，不再对外输出，但 track 还没彻底清理
    DEAD = 3        // 超过最大丢失帧，等待删除或已经无效
};

// 单帧观测输入（像素框 + 世界坐标）
struct WorldMeasurement {
    int class_id = 0;
    int team_id = 0;

    // 检测框置信度；身份更新使用下方独立的 class_conf。
    float score = 0.0f;

    bool is_dead = false;

    // 负观测只用于抑制附近的正观测与轨迹输出，不参与 Kalman 修正、
    // BotIdentity 投票或新轨迹创建。当前由死亡 ARMOR 观测设置。
    bool is_negative = false;

    cv::Rect box;          // 像素框 [x, y, w, h]
    cv::Point2f world;     // world.x = world_x, world.y = world_z，world_y 始终为 0

    // class_conf: 分类器 top1 置信度
    // class_margin: top1 - top2
    float class_conf = -1.0f;
    float class_margin = -1.0f;
};

// 跟踪目标输出
struct TrackedTarget {
    int track_id = 0;      // 全局唯一跟踪 ID
    int team_id = 0;
    int class_id = 0;
    int hit_count = 0;     // 命中次数
    int miss_count = 0;    // 连续丢失次数
    TrackState state = TrackState::ACTIVE;

    cv::Rect smoothed_box;       // 平滑后的像素框
    cv::Point2f smoothed_world;  // 平滑后的世界坐标，x=world_x, y=world_z
    bool is_dead = false;
    float score = 0.0f;

    // 可选：如果后续想把 BotIdentity 的稳定身份透传到旧接口，可以使用这两个字段。
    int stable_class_id = -1;
    float stable_class_conf = 0.0f;
};
