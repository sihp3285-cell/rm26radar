#include "tracker.hpp"
#include "hungarian.hpp"
#include <cmath>
#include <algorithm>
#include <map>

// ==========================================
// Tracker 实现
// ==========================================

Tracker::Tracker(const TrackerParams& params) : params_(params) {}

void Tracker::reset() {
    tracks_.clear();
    next_track_id_ = 1;
}

float Tracker::box_center_distance(const cv::Rect& a, const cv::Rect& b) {
    float cx1 = a.x + a.width  / 2.0f;
    float cy1 = a.y + a.height / 2.0f;
    float cx2 = b.x + b.width  / 2.0f;
    float cy2 = b.y + b.height / 2.0f;
    return std::hypot(cx1 - cx2, cy1 - cy2);
}

float Tracker::world_distance(const cv::Point2f& a, const cv::Point2f& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

TrackedTarget Tracker::track_to_target(const Track& t) {
    TrackedTarget tgt;
    tgt.track_id = t.track_id;
    tgt.team_id = t.team_id;
    tgt.class_id = t.class_id;
    tgt.hit_count = t.hit_count;
    tgt.miss_count = t.miss_count;
    tgt.state = t.state;
    tgt.smoothed_box = t.last_box;
    tgt.smoothed_world = t.last_world;
    tgt.is_dead = t.last_is_dead;
    tgt.score = t.last_score;
    return tgt;
}

void Tracker::associate_and_update(const std::vector<WorldMeasurement>& detections) {
    std::vector<bool> det_matched(detections.size(), false);

    // 按 (team_id, class_id) 对非 DEAD tracks 分组
    std::map<std::pair<int, int>, std::vector<size_t>> group_tracks;
    for (size_t i = 0; i < tracks_.size(); ++i) {
        if (tracks_[i]->state == TrackState::DEAD) continue;
        group_tracks[{tracks_[i]->team_id, tracks_[i]->class_id}].push_back(i);
    }

    // 按 (team_id, class_id) 对 detections 分组
    std::map<std::pair<int, int>, std::vector<size_t>> group_dets;
    for (size_t i = 0; i < detections.size(); ++i) {
        group_dets[{detections[i].team_id, detections[i].class_id}].push_back(i);
    }

    radar_core::tracker::HungarianAlgorithm hungarian;

    // 对每个类别组分别执行匈牙利匹配
    for (const auto& [key, track_indices] : group_tracks) {
        auto it = group_dets.find(key);
        if (it == group_dets.end()) {
            // 该组无任何观测，所有 track 标记为丢失
            for (size_t ti : track_indices) {
                auto& track = tracks_[ti];
                track->miss_count++;
                if (track->miss_count > params_.max_miss) {
                    track->state = TrackState::DEAD;
                } else {
                    track->state = TrackState::LOST;
                }
            }
            continue;
        }

        const auto& det_indices = it->second;
        int n_rows = static_cast<int>(track_indices.size());
        int n_cols = static_cast<int>(det_indices.size());

        // 构建代价矩阵：像素框中心欧氏距离
        std::vector<std::vector<float>> cost_matrix(
            n_rows, std::vector<float>(n_cols, 1e6f));
        for (int r = 0; r < n_rows; ++r) {
            const auto& track = *tracks_[track_indices[r]];
            for (int c = 0; c < n_cols; ++c) {
                const auto& det = detections[det_indices[c]];
                cost_matrix[r][c] = box_center_distance(track.last_box, det.box);
            }
        }

        std::vector<int> assignment;
        hungarian.Solve(cost_matrix, assignment);

        // 处理匹配结果
        for (int r = 0; r < n_rows; ++r) {
            auto& track = tracks_[track_indices[r]];
            int c = assignment[r];

            if (c >= 0 && c < n_cols && cost_matrix[r][c] < params_.max_gate_box) {
                const auto& det = detections[det_indices[c]];

                // ---- 更新 Kalman 像素框 ----
                std::vector<float> box_meas = {
                    det.box.x + det.box.width  / 2.0f,
                    det.box.y + det.box.height / 2.0f,
                    static_cast<float>(det.box.width),
                    static_cast<float>(det.box.height)
                };
                auto box_upd = track->kf_box.update(box_meas);
                track->last_box = cv::Rect(
                    static_cast<int>(box_upd[0] - box_upd[2] / 2.0f),
                    static_cast<int>(box_upd[1] - box_upd[3] / 2.0f),
                    static_cast<int>(box_upd[2]),
                    static_cast<int>(box_upd[3])
                );

                // ---- 更新 Kalman 世界坐标 ----
                auto world_upd = track->kf_world.update({det.world.x, det.world.y});
                track->last_world = cv::Point2f(world_upd[0], world_upd[1]);

                // ---- 更新统计量 ----
                track->hit_count++;
                track->miss_count = 0;
                track->state = TrackState::ACTIVE;
                track->last_score = det.score;
                track->last_is_dead = det.is_dead;

                det_matched[det_indices[c]] = true;
            } else {
                // 未匹配到观测
                track->miss_count++;
                if (track->miss_count > params_.max_miss) {
                    track->state = TrackState::DEAD;
                } else {
                    track->state = TrackState::LOST;
                }
            }
        }
    }

    // 未匹配的 detection 初始化新 track
    for (size_t i = 0; i < detections.size(); ++i) {
        if (det_matched[i]) continue;
        const auto& det = detections[i];

        auto track = std::make_unique<Track>();
        track->track_id = next_track_id_++;
        track->team_id = det.team_id;
        track->class_id = det.class_id;
        track->hit_count = 1;
        track->miss_count = 0;
        track->state = TrackState::ACTIVE;
        track->last_score = det.score;
        track->last_is_dead = det.is_dead;

        // 初始化像素框 Kalman
        track->kf_box.reset({
            det.box.x + det.box.width  / 2.0f,
            det.box.y + det.box.height / 2.0f,
            static_cast<float>(det.box.width),
            static_cast<float>(det.box.height)
        });
        track->last_box = det.box;

        // 初始化世界坐标 Kalman
        track->kf_world.reset({det.world.x, det.world.y});
        track->last_world = det.world;

        tracks_.push_back(std::move(track));
    }
}

void Tracker::update(const std::vector<WorldMeasurement>& detections) {
    // Step 1: 所有现有 track 先执行 predict（LOST 时提供预测值）
    for (auto& track : tracks_) {
        if (track->state == TrackState::DEAD) continue;

        auto box_pred = track->kf_box.predict();
        track->last_box = cv::Rect(
            static_cast<int>(box_pred[0] - box_pred[2] / 2.0f),
            static_cast<int>(box_pred[1] - box_pred[3] / 2.0f),
            static_cast<int>(box_pred[2]),
            static_cast<int>(box_pred[3])
        );

        auto world_pred = track->kf_world.predict();
        track->last_world = cv::Point2f(world_pred[0], world_pred[1]);
    }

    // Step 2: 匈牙利关联并更新
    associate_and_update(detections);
}

std::vector<TrackedTarget> Tracker::get_tracks() const {
    std::vector<TrackedTarget> out;
    for (const auto& track : tracks_) {
        if (track->state != TrackState::DEAD) {
            out.push_back(track_to_target(*track));
        }
    }
    return out;
}

std::vector<TrackedTarget> Tracker::get_outputs() const {
    std::vector<TrackedTarget> out;
    for (const auto& track : tracks_) {
        if (track->state == TrackState::DEAD) continue;
        if (track->hit_count < params_.min_hit) continue;
        out.push_back(track_to_target(*track));
    }
    return out;
}
