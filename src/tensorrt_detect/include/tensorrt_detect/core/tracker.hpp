#pragma once
#include "tracker_types.hpp"
#include "kalman.hpp"
#include "bot_identity.hpp"
#include "robot_id.hpp"
#include <vector>

// BotIdentityConfig 定义在 bot_identity.hpp 中，此处直接复用

// ==========================================
// Tracker - PhysicalTrack 多目标跟踪器
//   内部用动态 PhysicalTrack 按物理连续性跟踪目标
//   BotIdentity 逐步发现兵种身份，不再使用固定槽位
//   输出层通过仲裁映射回 10 个 official slot，保持外部接口兼容
// ==========================================
struct TrackerParams {
    int max_miss = 4;               // 连续丢失多少帧后 track 标记为 DEAD
    int max_predict = 2;            // 连续丢失多少帧内保持 PREDICTED（卡尔曼外推仍显示）
    int min_hit = 2;                // 最少命中次数才对外输出
    float max_gate_box = 200.0f;     // 像素框中心距离门限
    float class_mismatch_penalty = 300.0f;  // 跨兵种匹配惩罚（像素等效距离）
    int max_tracks = 20;            // 最大同时跟踪的 PhysicalTrack 数量
    BotIdentityConfig botIdentity;   // BotIdentity 身份稳定器参数
};

class Tracker {
public:
    // 固定槽位索引：Red R1~R4,S | Blue R1~R4,S（仅用于输出映射，内部不再有固定槽位）
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

    // 输入新一帧观测，更新所有 track 状态
    void update(const std::vector<WorldMeasurement>& detections);

    // 单个槽位的对外输出状态
    struct SlotOutput {
        int slot_idx = 0;
        int team_id = 0;
        int class_id = 0;
        bool valid = false;          // 当前是否有效（ACTIVE / PREDICTED 且满足 min_hit）
        TrackState state = TrackState::LOST;
        cv::Rect smoothed_box;
        cv::Point2f smoothed_world;
        bool is_dead = false;
        float score = 0.0f;

        // BotIdentity 稳定身份输出
        int stable_class_id = -1;
        float stable_class_conf = 0.0f;
    };

    // 获取指定 official slot 的当前状态（兼容旧接口，委托给 get_outputs()）
    SlotOutput get_slot(int idx) const;

    // 批量获取 10 个 official slot 输出（含仲裁），推荐使用
    std::vector<SlotOutput> get_outputs() const;

    void reset();

private:
    struct PhysicalTrack {
        int track_id = -1;              // 唯一 ID（调试用）
        int team_id = robot_id::UNKNOWN; // 首次匹配时设定，之后不变
        int hit_count = 0;
        int miss_count = 0;
        TrackState state = TrackState::DEAD;
        bool initialized = false;       // Kalman 是否已初始化

        KalmanFilterBox kf_box;   // 8维 [cx, cy, w, h, vx, vy, vw, vh]
        KalmanFilter2d kf_world;  // 4维 [x, z, vx, vz]
        BotIdentity bot_id;       // 身份轨迹池：跨帧持久化 class 历史

        cv::Rect last_box;
        cv::Point2f last_world;
        cv::Point2f detected_world;  // 本帧实际检测到的世界坐标（ACTIVE 时优先使用）
        float last_score = 0.0f;
        bool last_is_dead = false;
    };

    TrackerParams params_;
    std::vector<PhysicalTrack> tracks_;
    int next_track_id_ = 0;

    static float box_center_distance(const cv::Rect& a, const cv::Rect& b);

    // 将 (team, stable_class) 映射到 official slot 索引 0-9，无法映射返回 -1
    static int slot_for(int team, int class_id);

    // 同队同兵种仲裁：每组只保留最高置信度，其余 suppressed
    static void arbitrate_outputs(std::vector<SlotOutput>& outputs);
};
