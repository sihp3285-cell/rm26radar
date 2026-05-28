#include "tracker.hpp"
#include "hungarian.hpp"
#include <cmath>
#include <algorithm>
#include <unordered_map>

// ==========================================
// Tracker 实现 - PhysicalTrack 动态跟踪版
// ==========================================

Tracker::Tracker(const TrackerParams& params) : params_(params) {
    // tracks_ 初始为空，按需创建
}

void Tracker::reset() {
    tracks_.clear();
    next_track_id_ = 0;
}

float Tracker::box_center_distance(const cv::Rect& a, const cv::Rect& b) {
    float cx1 = a.x + a.width  / 2.0f;
    float cy1 = a.y + a.height / 2.0f;
    float cx2 = b.x + b.width  / 2.0f;
    float cy2 = b.y + b.height / 2.0f;
    return std::hypot(cx1 - cx2, cy1 - cy2);
}

// ---- official slot 映射 ----
int Tracker::slot_for(int team, int class_id) {
    if (team == robot_id::RED) {
        if (class_id == robot_id::R1) return Tracker::SLOT_RED_R1;
        if (class_id == robot_id::R2) return Tracker::SLOT_RED_R2;
        if (class_id == robot_id::R3) return Tracker::SLOT_RED_R3;
        if (class_id == robot_id::R4) return Tracker::SLOT_RED_R4;
        if (class_id == robot_id::S)  return Tracker::SLOT_RED_S;
    } else if (team == robot_id::BLUE) {
        if (class_id == robot_id::R1) return Tracker::SLOT_BLUE_R1;
        if (class_id == robot_id::R2) return Tracker::SLOT_BLUE_R2;
        if (class_id == robot_id::R3) return Tracker::SLOT_BLUE_R3;
        if (class_id == robot_id::R4) return Tracker::SLOT_BLUE_R4;
        if (class_id == robot_id::S)  return Tracker::SLOT_BLUE_S;
    }
    return -1;
}

// ---- 同队同兵种仲裁 ----
void Tracker::arbitrate_outputs(std::vector<SlotOutput>& outputs) {
    struct SlotRef { int idx; float conf; };
    std::unordered_map<uint64_t, std::vector<SlotRef>> groups;
    for (int i = 0; i < static_cast<int>(outputs.size()); ++i) {
        const auto& out = outputs[i];
        if (!out.valid) continue;
        if (out.stable_class_id < 0) continue;
        uint64_t key = (static_cast<uint64_t>(out.team_id) << 32)
                     | static_cast<uint64_t>(out.stable_class_id);
        groups[key].push_back({i, out.stable_class_conf});
    }
    for (auto& [key, refs] : groups) {
        if (refs.size() <= 1) continue;
        std::sort(refs.begin(), refs.end(),
                  [](const SlotRef& a, const SlotRef& b) {
                      if (a.conf != b.conf) return a.conf > b.conf;
                      return a.idx < b.idx;
                  });
        for (size_t j = 1; j < refs.size(); ++j) {
            auto& sup = outputs[refs[j].idx];
            sup.stable_class_id = -1;
            sup.stable_class_conf = 0.0f;
            sup.valid = false;
        }
    }
}

// ---- 批量输出（含 track->slot 映射 + 仲裁） ----
std::vector<Tracker::SlotOutput> Tracker::get_outputs() const {
    std::vector<SlotOutput> results(NUM_SLOTS);

    // 初始化 10 个 official slot，带上 nominal (team, class)
    auto init_slot = [&results](int idx, int team, int cls) {
        results[idx].slot_idx = idx;
        results[idx].team_id  = team;
        results[idx].class_id = cls;
        results[idx].valid    = false;
        results[idx].state    = TrackState::DEAD;
    };
    init_slot(SLOT_RED_R1,  robot_id::RED,  robot_id::R1);
    init_slot(SLOT_RED_R2,  robot_id::RED,  robot_id::R2);
    init_slot(SLOT_RED_R3,  robot_id::RED,  robot_id::R3);
    init_slot(SLOT_RED_R4,  robot_id::RED,  robot_id::R4);
    init_slot(SLOT_RED_S,   robot_id::RED,  robot_id::S);
    init_slot(SLOT_BLUE_R1, robot_id::BLUE, robot_id::R1);
    init_slot(SLOT_BLUE_R2, robot_id::BLUE, robot_id::R2);
    init_slot(SLOT_BLUE_R3, robot_id::BLUE, robot_id::R3);
    init_slot(SLOT_BLUE_R4, robot_id::BLUE, robot_id::R4);
    init_slot(SLOT_BLUE_S,  robot_id::BLUE, robot_id::S);

    // 遍历所有 track，按 (team, stable_class) 映射到 official slot
    for (const auto& track : tracks_) {
        if (track.state != TrackState::ACTIVE && track.state != TrackState::PREDICTED)
            continue;
        if (track.hit_count < params_.min_hit)
            continue;

        auto [stable_cls, stable_conf] = track.bot_id.getStableClass();
        if (stable_cls < 0 || stable_conf <= 0.0f)
            continue;

        int slot_idx = slot_for(track.team_id, stable_cls);
        if (slot_idx < 0) continue;

        auto& slot = results[slot_idx];
        // 如果已有 track 占据此 slot，取置信度更高的
        if (slot.valid && slot.stable_class_conf >= stable_conf)
            continue;

        slot.valid             = true;
        slot.state             = track.state;
        slot.smoothed_box      = track.last_box;
        slot.smoothed_world    = (track.state == TrackState::ACTIVE)
                                 ? track.detected_world : track.last_world;
        slot.is_dead           = track.last_is_dead;
        slot.score             = track.last_score;
        slot.stable_class_id   = stable_cls;
        slot.stable_class_conf = stable_conf;
    }

    // 仲裁：确保每个 (team, stable_class) 只出现一次
    arbitrate_outputs(results);

    return results;
}

// ---- 兼容旧接口 ----
Tracker::SlotOutput Tracker::get_slot(int idx) const {
    if (idx < 0 || idx >= NUM_SLOTS) return SlotOutput{};
    auto all = get_outputs();
    return all[idx];
}

// ---- 核心更新 ----
void Tracker::update(const std::vector<WorldMeasurement>& detections) {
    int n_cols = static_cast<int>(detections.size());

    // ---- Step 1: Kalman predict ----
    for (auto& track : tracks_) {
        if (!track.initialized || track.state == TrackState::DEAD) continue;
        if (track.last_is_dead) continue;  // 死亡车辆冻结

        auto box_pred = track.kf_box.predict();
        track.last_box = cv::Rect(
            static_cast<int>(box_pred[0] - box_pred[2] / 2.0f),
            static_cast<int>(box_pred[1] - box_pred[3] / 2.0f),
            static_cast<int>(box_pred[2]),
            static_cast<int>(box_pred[3])
        );
        auto world_pred = track.kf_world.predict();
        track.last_world = cv::Point2f(world_pred[0], world_pred[1]);
    }

    // ---- Step 2: 构建代价矩阵 (tracks × detections) ----
    int n_rows = static_cast<int>(tracks_.size());
    std::vector<bool> det_matched(n_cols, false);

    if (n_rows > 0 && n_cols > 0) {
        radar_core::tracker::HungarianAlgorithm hungarian;
        std::vector<std::vector<float>> cost_matrix(
            n_rows, std::vector<float>(n_cols, 1e6f));

        for (int r = 0; r < n_rows; ++r) {
            const auto& track = tracks_[r];
            auto [track_stable_cls, _] = track.bot_id.getStableClass();

            for (int c = 0; c < n_cols; ++c) {
                const auto& det = detections[c];
                // team_id 硬过滤
                if (det.team_id != track.team_id) {
                    cost_matrix[r][c] = 1e6f;
                    continue;
                }
                float gate = (track.initialized && track.state != TrackState::DEAD)
                                 ? params_.max_gate_box : 1e6f;
                float d = box_center_distance(track.last_box, det.box);
                float cost = d;
                // class mismatch penalty：用 track 的 stable_class 作为参考
                if (track_stable_cls >= 0 && det.class_id != track_stable_cls) {
                    cost += params_.class_mismatch_penalty;
                }
                if (cost >= gate) {
                    cost_matrix[r][c] = 1e6f;
                    continue;
                }
                cost_matrix[r][c] = cost;
            }
        }

        std::vector<int> assignment;
        hungarian.Solve(cost_matrix, assignment);

        // ---- Step 3: 处理匹配结果 ----
        for (int r = 0; r < n_rows; ++r) {
            auto& track = tracks_[r];
            int c = assignment[r];
            float gate_check = (track.initialized && track.state != TrackState::DEAD)
                                   ? params_.max_gate_box : 1e6f;
            bool matched = (c >= 0 && c < n_cols && cost_matrix[r][c] < gate_check);

            if (matched) {
                const auto& det = detections[c];

                std::vector<float> box_meas = {
                    det.box.x + det.box.width  / 2.0f,
                    det.box.y + det.box.height / 2.0f,
                    static_cast<float>(det.box.width),
                    static_cast<float>(det.box.height)
                };
                if (!track.initialized || track.state == TrackState::DEAD) {
                    track.kf_box.reset(box_meas);
                    track.kf_world.reset({det.world.x, det.world.y});
                    track.bot_id.reset();
                    track.initialized = true;
                    track.last_box = det.box;
                    track.last_world = det.world;
                    track.detected_world = det.world;
                } else {
                    track.detected_world = det.world;
                    auto box_upd = track.kf_box.update(box_meas);
                    track.last_box = cv::Rect(
                        static_cast<int>(box_upd[0] - box_upd[2] / 2.0f),
                        static_cast<int>(box_upd[1] - box_upd[3] / 2.0f),
                        static_cast<int>(box_upd[2]),
                        static_cast<int>(box_upd[3])
                    );
                    auto world_upd = track.kf_world.update({det.world.x, det.world.y});
                    track.last_world = cv::Point2f(world_upd[0], world_upd[1]);
                    track.detected_world = det.world;
                }
                if (track.hit_count == 0) {
                    track.last_box = det.box;
                    track.last_world = det.world;
                    track.detected_world = det.world;
                }

                track.bot_id.update(det.class_id, det.score);

                track.hit_count++;
                track.miss_count = 0;
                track.state = TrackState::ACTIVE;
                track.last_score = det.score;
                track.last_is_dead = det.is_dead;
                det_matched[c] = true;
            } else {
                // 未匹配到观测
                if (track.last_is_dead) {
                    // 死亡车辆冻结
                } else {
                    track.miss_count++;
                    track.bot_id.markLost();
                    if (track.miss_count > params_.max_miss) {
                        track.state = TrackState::DEAD;
                    } else if (track.miss_count <= params_.max_predict) {
                        track.state = TrackState::PREDICTED;
                    } else {
                        track.state = TrackState::LOST;
                    }
                }
            }
        }
    } else if (n_cols == 0) {
        // 无 detection，所有已初始化且非死亡 track 标记为丢失
        for (auto& track : tracks_) {
            if (!track.initialized || track.last_is_dead) continue;
            track.miss_count++;
            track.bot_id.markLost();
            if (track.miss_count > params_.max_miss) {
                track.state = TrackState::DEAD;
            } else if (track.miss_count <= params_.max_predict) {
                track.state = TrackState::PREDICTED;
            } else {
                track.state = TrackState::LOST;
            }
        }
    }

    // ---- Step 4: 未匹配的 detection → 创建新 PhysicalTrack ----
    for (int c = 0; c < n_cols; ++c) {
        if (det_matched[c]) continue;
        if (static_cast<int>(tracks_.size()) >= params_.max_tracks) break;

        const auto& det = detections[c];

        PhysicalTrack track;
        track.track_id = next_track_id_++;
        track.team_id  = det.team_id;
        track.bot_id.configure(params_.botIdentity);
        track.bot_id.update(det.class_id, det.score);
        track.initialized = true;
        track.state       = TrackState::ACTIVE;
        track.hit_count   = 1;
        track.miss_count  = 0;
        track.last_box    = det.box;
        track.last_world  = det.world;
        track.detected_world = det.world;
        track.last_score  = det.score;
        track.last_is_dead = det.is_dead;

        std::vector<float> box_meas = {
            det.box.x + det.box.width  / 2.0f,
            det.box.y + det.box.height / 2.0f,
            static_cast<float>(det.box.width),
            static_cast<float>(det.box.height)
        };
        track.kf_box.reset(box_meas);
        track.kf_world.reset({det.world.x, det.world.y});

        tracks_.push_back(std::move(track));
    }

    // ---- Step 5: 清理 DEAD 超时 track ----
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
        [this](const PhysicalTrack& t) {
            return t.state == TrackState::DEAD && t.miss_count > params_.max_miss + 2;
        }), tracks_.end());
}
