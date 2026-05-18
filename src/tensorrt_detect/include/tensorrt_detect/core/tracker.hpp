#pragma once
#include "tracker_types.hpp"
#include "kalman.hpp"
#include "robot_id.hpp"
#include <vector>

// ==========================================
// Tracker - 固定槽位多目标跟踪器
//   每个兵种预分配一个永久槽位（座位），class/team 构造时焊死
//   匈牙利匹配只做一件事：把检测分配给对应类别的固定槽位
//   从根源上消除“同一目标多个 track_id / 旧坐标残留”
// ==========================================
struct TrackerParams {
    int max_miss = 4;               // 连续丢失多少帧后槽位标记为 DEAD（不再输出）
    int min_hit = 2;                // 最少命中次数才对外输出
    float max_gate_box = 200.0f;     // 像素框中心距离门限
};

class Tracker {
public:
    // 固定槽位索引：Red R1~R4,S | Blue R1~R4,S
    // 注：Outpost 不走 Tracker，由 pose_node 直接透传
    static constexpr int NUM_SLOTS = 10;
    static constexpr int SLOT_RED_R1   = 0;
    static constexpr int SLOT_RED_R2   = 1;
    static constexpr int SLOT_RED_R3   = 2;
    static constexpr int SLOT_RED_R4   = 3;
    static constexpr int SLOT_RED_S    = 4;
    static constexpr int SLOT_BLUE_R1  = 5;
    static constexpr int SLOT_BLUE_R2  = 6;
    static constexpr int SLOT_BLUE_R3  = 7;
    static constexpr int SLOT_BLUE_R4  = 8;
    static constexpr int SLOT_BLUE_S   = 9;

    explicit Tracker(const TrackerParams& params = TrackerParams());

    // 输入新一帧观测，更新所有槽位状态
    void update(const std::vector<WorldMeasurement>& detections);

    // 单个槽位的对外输出状态
    struct SlotOutput {
        int slot_idx = 0;
        int team_id = 0;
        int class_id = 0;
        bool valid = false;          // 当前是否有效（非 DEAD 且满足 min_hit）
        TrackState state = TrackState::LOST;
        cv::Rect smoothed_box;
        cv::Point2f smoothed_world;
        bool is_dead = false;
        float score = 0.0f;
    };

    // 获取指定槽位的当前状态（用于 pose_node 固定数组输出）
    SlotOutput get_slot(int idx) const;

    void reset();

private:
    struct Slot {
        int slot_idx = 0;
        int team_id = 0;
        int class_id = 0;
        int hit_count = 0;
        int miss_count = 0;
        TrackState state = TrackState::DEAD;  // 初始未激活
        bool initialized = false;             // Kalman 是否已初始化

        KalmanFilterBox kf_box;   // 8维 [cx, cy, w, h, vx, vy, vw, vh]
        KalmanFilter2d kf_world;  // 4维 [x, z, vx, vz]

        cv::Rect last_box;
        cv::Point2f last_world;
        float last_score = 0.0f;
        bool last_is_dead = false;
    };

    TrackerParams params_;
    std::vector<Slot> slots_;

    static float box_center_distance(const cv::Rect& a, const cv::Rect& b);
};
