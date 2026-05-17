#pragma once
#include "tracker_types.hpp"
#include "kalman.hpp"
#include <vector>
#include <memory>

// ==========================================
// Tracker - 多目标跟踪器
//   按 (team_id, class_id) 维护目标
//   使用匈牙利算法做数据关联（按 team_id + class_id 分组）
//   支持 Kalman 平滑像素框与世界坐标
//   支持短时间 LOST 预测
// ==========================================
struct TrackerParams {
    int max_miss = 3;            // 最大允许连续丢失帧数，超过则标记为 DEAD
    int min_hit = 1;             // 最少命中次数才对外输出
    float max_gate_box = 80.0f; // 像素框中心距离门限（预留 gating 扩展）
    float max_gate_world = 3.0f; // 世界坐标距离门限（预留 gating 扩展）
};

class Tracker {
public:
    explicit Tracker(const TrackerParams& params = TrackerParams());

    // 输入新一帧观测，更新所有跟踪器状态
    void update(const std::vector<WorldMeasurement>& detections);

    // 获取所有未 DEAD 的跟踪结果（含 LOST 预测值）
    std::vector<TrackedTarget> get_tracks() const;

    // 获取对外输出：ACTIVE + 短时间 LOST，且满足 min_hit
    std::vector<TrackedTarget> get_outputs() const;

    // 重置所有跟踪状态
    void reset();

private:
    struct Track {
        int track_id = 0;
        int team_id = 0;
        int class_id = 0;
        int hit_count = 0;
        int miss_count = 0;
        TrackState state = TrackState::ACTIVE;

        KalmanFilterBox kf_box;   // 8维 [cx, cy, w, h, vx, vy, vw, vh]
        KalmanFilter2d kf_world;  // 4维 [x, z, vx, vz]

        // 上次更新后的平滑结果（方便外部读取，避免重复计算）
        cv::Rect last_box;
        cv::Point2f last_world;
        float last_score = 0.0f;
        bool last_is_dead = false;
    };

    TrackerParams params_;
    int next_track_id_ = 1;
    std::vector<std::unique_ptr<Track>> tracks_;

    // 匈牙利关联：按 (team_id, class_id) 分组构建代价矩阵，全局最优匹配
    void associate_and_update(const std::vector<WorldMeasurement>& detections);

    static float box_center_distance(const cv::Rect& a, const cv::Rect& b);
    static float world_distance(const cv::Point2f& a, const cv::Point2f& b);

    static TrackedTarget track_to_target(const Track& t);
};
