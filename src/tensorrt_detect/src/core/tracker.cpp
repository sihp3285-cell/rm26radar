#include "tracker.hpp"
#include "hungarian.hpp"

#include <cmath>
#include <algorithm>
#include <unordered_set>
#include <limits>

// ==========================================
// Tracker 实现 - PhysicalTrack + SlotOwner 稳定槽位版
// ==========================================

namespace {
constexpr float INF_COST = 1e6f;
constexpr std::int64_t NS_PER_SECOND = 1000000000LL;
}

// ---------------- 构造 / reset ----------------

Tracker::Tracker(const TrackerParams& params) : params_(params) {
    // 防御直接通过 TrackerParams 构造的调用方：端点先限制为非负，顺序颠倒时交换。
    params_.class_mismatch_min_penalty =
        std::max(0.0f, params_.class_mismatch_min_penalty);
    params_.class_mismatch_penalty =
        std::max(0.0f, params_.class_mismatch_penalty);
    if (params_.class_mismatch_min_penalty > params_.class_mismatch_penalty) {
        std::swap(params_.class_mismatch_min_penalty, params_.class_mismatch_penalty);
    }
    params_.max_predict_time_s = std::max(0.0f, params_.max_predict_time_s);
    params_.max_lost_time_s = std::max(
        params_.max_predict_time_s, params_.max_lost_time_s);
    params_.dead_retention_time_s = std::max(0.0f, params_.dead_retention_time_s);
    params_.slot_lease_time_s = std::max(0.0f, params_.slot_lease_time_s);
    last_outputs_ = make_empty_outputs();
}

void Tracker::reset() {
    tracks_.clear();
    next_track_id_ = 0;
    current_time_ns_ = 0;
    time_initialized_ = false;

    for (auto& owner : slot_owners_) {
        owner = SlotOwner{};
    }

    last_outputs_ = make_empty_outputs();
}

// ---------------- 基础工具 ----------------

float Tracker::box_center_distance(const cv::Rect& a, const cv::Rect& b) {
    float cx1 = a.x + a.width  / 2.0f;
    float cy1 = a.y + a.height / 2.0f;
    float cx2 = b.x + b.width  / 2.0f;
    float cy2 = b.y + b.height / 2.0f;
    return std::hypot(cx1 - cx2, cy1 - cy2);
}

float Tracker::point_distance(const cv::Point2f& a, const cv::Point2f& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

float Tracker::calculate_class_mismatch_penalty(
    const PhysicalTrack& track,
    const WorldMeasurement& detection
) const {
    // 保持类别约束为软约束：未提交身份或类别一致时不增加代价。
    if (track.committed_class < 0 || detection.class_id == track.committed_class) {
        return 0.0f;
    }

    const auto stats = track.bot_id.getStats();
    const float track_stability = std::clamp(stats.stability, 0.0f, 1.0f);
    const float track_switch_reliability =
        1.0f - std::clamp(stats.switch_rate, 0.0f, 1.0f);
    const float track_reliability = std::clamp(
        track_stability * track_switch_reliability,
        0.0f,
        1.0f
    );
    const float detection_reliability =
        std::clamp(detection.class_conf, 0.0f, 1.0f);
    const float combined_reliability = std::clamp(
        track_reliability * detection_reliability,
        0.0f,
        1.0f
    );

    return params_.class_mismatch_min_penalty +
        (params_.class_mismatch_penalty - params_.class_mismatch_min_penalty) *
            combined_reliability;
}

// ---------------- official slot 映射 ----------------

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

// ---------------- 空输出初始化 ----------------

std::vector<Tracker::SlotOutput> Tracker::make_empty_outputs() const {
    std::vector<SlotOutput> results(NUM_SLOTS);

    auto init_slot = [&results](int idx, int team, int cls) {
        results[idx].slot_idx = idx;
        results[idx].team_id  = team;
        results[idx].class_id = cls;
        results[idx].track_id = -1;
        results[idx].valid    = false;
        results[idx].state    = TrackState::INVALID;
        results[idx].position_source = PositionSource::INVALID;
        results[idx].stable_class_id = -1;
        results[idx].stable_class_conf = 0.0f;
        results[idx].owner_lost_duration_s = 0.0f;
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

    return results;
}

// ---------------- track 查找 ----------------

Tracker::PhysicalTrack* Tracker::find_track_by_id(int track_id) {
    for (auto& track : tracks_) {
        if (track.track_id == track_id) {
            return &track;
        }
    }
    return nullptr;
}

const Tracker::PhysicalTrack* Tracker::find_track_by_id(int track_id) const {
    for (const auto& track : tracks_) {
        if (track.track_id == track_id) {
            return &track;
        }
    }
    return nullptr;
}

bool Tracker::is_output_state(TrackState state) const {
    return state == TrackState::ACTIVE || state == TrackState::PREDICTED;
}

void Tracker::reserve_owner(SlotOwner& owner) {
    if (owner.state == SlotOwnerState::OWNED) {
        owner.state = SlotOwnerState::RESERVED;
        owner.reserved_since_time_ns = current_time_ns_;
    }
}

void Tracker::update_lifecycle(PhysicalTrack& track) {
    if (track.last_observed_time_ns <= 0 || current_time_ns_ <= 0) {
        track.lost_duration_s = 0.0f;
    } else {
        track.lost_duration_s = static_cast<float>(
            std::max<std::int64_t>(0, current_time_ns_ - track.last_observed_time_ns)) * 1e-9f;
    }

    if (track.lost_duration_s > params_.max_lost_time_s) {
        if (track.state != TrackState::DEAD) {
            track.dead_since_time_ns = current_time_ns_;
        }
        track.state = TrackState::DEAD;
    } else if (track.lost_duration_s <= params_.max_predict_time_s) {
        track.state = TrackState::PREDICTED;
    } else {
        track.state = TrackState::LOST;
    }
}

float Tracker::calculate_tracking_confidence(const PhysicalTrack& track) const {
    const float detection_confidence = std::clamp(track.last_score, 0.0f, 1.0f);
    if (track.observed_this_update) {
        return detection_confidence;
    }
    if (params_.max_lost_time_s <= 0.0f) {
        return 0.0f;
    }
    const float remaining = std::clamp(
        1.0f - track.lost_duration_s / params_.max_lost_time_s,
        0.0f,
        1.0f);
    return detection_confidence * remaining;
}

void Tracker::update_identity_state(
    PhysicalTrack& track,
    const WorldMeasurement& detection) {
    const bool valid =
        detection.class_conf >= params_.min_identity_update_conf &&
        slot_for(track.team_id, detection.class_id) >= 0;
    if (!valid) {
        track.pending_class = -1;
        track.pending_class_observations = 0;
        return;
    }

    if (detection.class_id == track.committed_class) {
        track.pending_class = -1;
        track.pending_class_observations = 0;
        return;
    }

    if (detection.class_id == track.pending_class) {
        track.pending_class_observations++;
    } else {
        track.pending_class = detection.class_id;
        track.pending_class_observations = 1;
    }

    const int confirm_observations = track.committed_class < 0
        ? params_.identity_confirm_observations
        : params_.identity_switch_confirm_observations;
    const auto stats = track.bot_id.getStats();
    if (track.pending_class_observations >= confirm_observations &&
        stats.class_id == track.pending_class &&
        stats.confidence >= params_.slot_bind_min_conf &&
        stats.stability >= params_.slot_min_stability &&
        stats.switch_rate <= params_.slot_max_switch_rate) {
        track.committed_class = track.pending_class;
        track.pending_class = -1;
        track.pending_class_observations = 0;
    }
}

// ---------------- 填充 SlotOutput ----------------

void Tracker::fill_slot_output(
    SlotOutput& out,
    int slot_idx,
    const PhysicalTrack& track,
    int stable_class_id,
    float stable_class_conf,
    float owner_lost_duration_s,
    bool valid
) const {
    out.valid = valid;
    out.slot_idx = slot_idx;

    // nominal class / team 已经在 make_empty_outputs() 初始化过
    out.track_id = track.track_id;
    out.state = track.state;
    out.observed = track.observed_this_update;
    out.position_source = track.observed_this_update
        ? PositionSource::TRACKED
        : PositionSource::PREDICTED;

    out.smoothed_box = track.last_box;

    // 关键改动：
    // 统一输出 Kalman 后的 last_world，而不是 ACTIVE 时输出 detected_world。
    // 你原来 ACTIVE 用 detected_world，会让地图点更抖。
    out.smoothed_world = track.last_world;
    const auto velocity = track.kf_world.get_velocity();
    if (velocity.size() >= 2) {
        out.velocity = cv::Point2f(velocity[0], velocity[1]);
    }

    out.covariance_valid = track.kf_world.P.allFinite();
    if (out.covariance_valid) {
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                out.state_covariance[row * 4 + col] =
                    static_cast<double>(track.kf_world.P(row, col));
            }
        }
    }

    out.is_dead = track.last_is_dead;
    out.score = track.last_score;
    out.detection_confidence = track.last_score;
    out.tracking_confidence = calculate_tracking_confidence(track);
    out.last_observed_time_ns = track.last_observed_time_ns;
    out.lost_duration_s = track.lost_duration_s;

    out.stable_class_id = stable_class_id;
    out.stable_class_conf = stable_class_conf;

    out.owner_lost_duration_s = owner_lost_duration_s;
}

// ==========================================================
// 核心：构建输出 + slot owner 锁定
// ==========================================================

void Tracker::build_outputs_with_slot_owner() {
    auto results = make_empty_outputs();
    std::unordered_set<int> used_tracks;

    // Phase 1: 已有 owner 优先；不可输出时转 RESERVED 并独立倒计时。
    for (int slot_idx = 0; slot_idx < NUM_SLOTS; ++slot_idx) {
        auto& owner = slot_owners_[slot_idx];
        if (owner.state == SlotOwnerState::FREE) {
            continue;
        }

        PhysicalTrack* owner_track = find_track_by_id(owner.track_id);

        // 同一身份出现多个物理轨迹时，不让已绑定关系压过更可靠的近期观测。
        if (owner_track &&
            slot_for(owner_track->team_id, owner_track->committed_class) == slot_idx) {
            float best_recent_conf = owner_track->bot_id.getRecentConfidence(
                owner_track->committed_class);
            for (auto& track : tracks_) {
                if (track.track_id == owner.track_id ||
                    track.state != TrackState::ACTIVE ||
                    track.negative_suppressed ||
                    track.hit_count < params_.min_hit ||
                    track.pending_class >= 0 ||
                    slot_for(track.team_id, track.committed_class) != slot_idx) {
                    continue;
                }

                const auto challenger_stats = track.bot_id.getStats();
                if (challenger_stats.class_id != track.committed_class ||
                    challenger_stats.confidence < params_.slot_bind_min_conf ||
                    challenger_stats.stability < params_.slot_min_stability ||
                    challenger_stats.switch_rate > params_.slot_max_switch_rate) {
                    continue;
                }

                if (owner.has_last_world && params_.max_slot_jump_dist > 0.0f &&
                    point_distance(track.last_world, owner.last_world) >
                        params_.max_slot_jump_dist) {
                    continue;
                }

                const float recent_conf = track.bot_id.getRecentConfidence(
                    track.committed_class);
                if (recent_conf > best_recent_conf) {
                    owner.track_id = track.track_id;
                    owner.state = SlotOwnerState::OWNED;
                    owner_track = &track;
                    best_recent_conf = recent_conf;
                }
            }
        }

        const bool can_reclaim = owner_track &&
            is_output_state(owner_track->state) &&
            !owner_track->negative_suppressed &&
            owner_track->hit_count >= params_.min_hit &&
            slot_for(owner_track->team_id, owner_track->committed_class) == slot_idx;

        if (owner.state == SlotOwnerState::RESERVED) {
            if (can_reclaim) {
                owner.state = SlotOwnerState::OWNED;
                owner.reserved_since_time_ns = 0;
            } else {
                if (owner_track &&
                    slot_for(owner_track->team_id, owner_track->committed_class) == slot_idx) {
                    fill_slot_output(
                        results[slot_idx], slot_idx, *owner_track,
                        owner_track->committed_class, owner.last_conf,
                        owner_track->lost_duration_s, false);
                }
                const float reserved_duration_s = owner.reserved_since_time_ns > 0
                    ? static_cast<float>(std::max<std::int64_t>(
                          0, current_time_ns_ - owner.reserved_since_time_ns)) * 1e-9f
                    : 0.0f;
                if (reserved_duration_s >= params_.slot_lease_time_s) {
                    owner.state = SlotOwnerState::FREE;
                    owner.track_id = -1;
                    owner.reserved_since_time_ns = 0;
                }
                continue;
            }
        }

        if (owner.state != SlotOwnerState::OWNED) {
            continue;
        }

        if (!can_reclaim) {
            if (owner_track &&
                slot_for(owner_track->team_id, owner_track->committed_class) == slot_idx) {
                fill_slot_output(
                    results[slot_idx], slot_idx, *owner_track,
                    owner_track->committed_class, owner.last_conf,
                    owner_track->lost_duration_s, false);
            }
            reserve_owner(owner);
            continue;
        }

        const auto stats = owner_track->bot_id.getStats();
        const float output_conf = stats.class_id == owner_track->committed_class
            ? stats.confidence : owner.last_conf;
        fill_slot_output(results[slot_idx], slot_idx, *owner_track,
                         owner_track->committed_class, output_conf,
                         owner_track->lost_duration_s, true);

        owner.reserved_since_time_ns = 0;
        owner.last_world = owner_track->last_world;
        owner.has_last_world = true;
        owner.last_conf = output_conf;
        used_tracks.insert(owner_track->track_id);
    }

    // Phase 2: 只有身份已提交且稳定的 track 才能竞争 FREE slot。
    std::array<std::vector<SlotCandidate>, NUM_SLOTS> candidates;

    for (int i = 0; i < static_cast<int>(tracks_.size()); ++i) {
        const auto& track = tracks_[i];

        if (!is_output_state(track.state)) {
            continue;
        }

        if (track.negative_suppressed) {
            continue;
        }

        if (track.hit_count < params_.min_hit) {
            continue;
        }

        if (used_tracks.count(track.track_id) > 0) {
            continue;
        }

        const auto stats = track.bot_id.getStats();
        if (track.committed_class < 0 || track.pending_class >= 0 ||
            stats.class_id != track.committed_class ||
            stats.confidence < params_.slot_bind_min_conf ||
            stats.stability < params_.slot_min_stability ||
            stats.switch_rate > params_.slot_max_switch_rate) {
            continue;
        }

        int slot_idx = slot_for(track.team_id, track.committed_class);
        if (slot_idx < 0) {
            continue;
        }

        auto& owner = slot_owners_[slot_idx];
        if (owner.state != SlotOwnerState::FREE) {
            continue;
        }

        // 防止 slot 瞬移。
        // 例如 R2 原来在右边，某个左边的 track 突然误识别成 R2，距离过大则拒绝。
        if (owner.has_last_world && params_.max_slot_jump_dist > 0.0f) {
            float jump = point_distance(track.last_world, owner.last_world);

            if (jump > params_.max_slot_jump_dist) {
                continue;
            }
        }

        float state_bonus = (track.state == TrackState::ACTIVE) ? 1.0f : 0.5f;
        float age_bonus = std::min(track.hit_count, 30) * 0.01f;

        SlotCandidate cand;
        cand.track_index = i;
        cand.slot_idx = slot_idx;
        cand.stable_class_id = track.committed_class;
        cand.stable_class_conf = stats.confidence;
        cand.priority = state_bonus + stats.confidence + stats.stability +
                        stats.margin - stats.switch_rate + age_bonus;

        candidates[slot_idx].push_back(cand);
    }

    // ------------------------------------------------------
    // Phase 3: 对空 slot 选择最佳候选者并绑定 owner
    // ------------------------------------------------------
    for (int slot_idx = 0; slot_idx < NUM_SLOTS; ++slot_idx) {
        if (results[slot_idx].valid) {
            continue;
        }

        auto& list = candidates[slot_idx];
        if (list.empty()) {
            continue;
        }

        std::sort(list.begin(), list.end(),
            [](const SlotCandidate& a, const SlotCandidate& b) {
                if (a.priority != b.priority) {
                    return a.priority > b.priority;
                }
                return a.track_index < b.track_index;
            }
        );

        for (const auto& cand : list) {
            const auto& track = tracks_[cand.track_index];

            if (used_tracks.count(track.track_id) > 0) {
                continue;
            }

            auto& owner = slot_owners_[slot_idx];

            // 绑定新的 owner，并在每个有效输出帧续租。
            owner.state = SlotOwnerState::OWNED;
            owner.track_id = track.track_id;
            owner.reserved_since_time_ns = 0;
            owner.last_world = track.last_world;
            owner.has_last_world = true;
            owner.last_conf = cand.stable_class_conf;

            fill_slot_output(
                results[slot_idx],
                slot_idx,
                track,
                cand.stable_class_id,
                cand.stable_class_conf,
                track.lost_duration_s,
                true
            );

            used_tracks.insert(track.track_id);
            break;
        }
    }

    last_outputs_ = std::move(results);
}

// ---------------- 输出接口 ----------------

std::vector<Tracker::SlotOutput> Tracker::get_outputs() const {
    return last_outputs_;
}

Tracker::SlotOutput Tracker::get_slot(int idx) const {
    if (idx < 0 || idx >= NUM_SLOTS) {
        return SlotOutput{};
    }

    if (last_outputs_.empty()) {
        return SlotOutput{};
    }

    return last_outputs_[idx];
}

// ==========================================================
// 核心更新：物理匹配优先，class 只做轻量软惩罚
// ==========================================================

void Tracker::update(
    const std::vector<WorldMeasurement>& detections,
    float dt,
    std::int64_t timestamp_ns) {
    const float effective_dt = (std::isfinite(dt) && dt >= 0.0f) ? dt : 0.1f;
    if (timestamp_ns > 0) {
        if (time_initialized_ && timestamp_ns < current_time_ns_) {
            reset();
        }
        current_time_ns_ = timestamp_ns;
        time_initialized_ = true;
    } else {
        if (!time_initialized_) {
            current_time_ns_ = NS_PER_SECOND;
            time_initialized_ = true;
        } else {
            current_time_ns_ += static_cast<std::int64_t>(effective_dt * NS_PER_SECOND);
        }
    }

    const int n_cols = static_cast<int>(detections.size());

    // ------------------------------------------------------
    // Step 1: Kalman predict
    // ------------------------------------------------------
    for (auto& track : tracks_) {
        track.negative_suppressed = false;
        track.observed_this_update = false;

        if (!track.initialized || track.state == TrackState::DEAD) {
            continue;
        }

        if (track.last_is_dead) {
            continue;  // 死亡车辆冻结
        }

        auto box_pred = track.kf_box.predict(dt);

        track.last_box = cv::Rect(
            static_cast<int>(box_pred[0] - box_pred[2] / 2.0f),
            static_cast<int>(box_pred[1] - box_pred[3] / 2.0f),
            static_cast<int>(box_pred[2]),
            static_cast<int>(box_pred[3])
        );

        auto world_pred = track.kf_world.predict(dt);
        track.last_world = cv::Point2f(world_pred[0], world_pred[1]);
    }

    // ------------------------------------------------------
    // Step 2: 构建代价矩阵
    // ------------------------------------------------------
    const int n_rows = static_cast<int>(tracks_.size());
    std::vector<bool> det_matched(n_cols, false);
    // 已接近现有轨迹、但被 Kalman 创新门控拒绝的异常观测，不能在 Step 4
    // 立即创建新轨迹，否则单帧误识别会产生幽灵 track。
    std::vector<bool> det_suppressed(n_cols, false);

    // 负观测的 box/world gate 是“启用项同时满足”。这样死亡点与相邻活车
    // 仅在单一坐标域接近时，不会轻易抑制活车身份。
    const auto is_near_negative = [this](
        const cv::Rect& box,
        const cv::Point2f& world,
        const WorldMeasurement& negative) {
        bool enabled = false;
        bool inside = true;

        if (params_.negative_gate_box > 0.0f) {
            enabled = true;
            inside = inside &&
                box_center_distance(box, negative.box) <= params_.negative_gate_box;
        }
        if (params_.negative_gate_world > 0.0f) {
            enabled = true;
            inside = inside &&
                point_distance(world, negative.world) <= params_.negative_gate_world;
        }
        return enabled && inside;
    };

    // 负观测本身永远不参与关联/建轨；同时抑制其邻域内可能由死亡车辆
    // 短暂误分类产生的正观测，以及已有轨迹的当帧身份输出。
    for (int negative_idx = 0; negative_idx < n_cols; ++negative_idx) {
        const auto& negative = detections[negative_idx];
        if (!negative.is_negative) {
            continue;
        }

        det_suppressed[negative_idx] = true;

        for (int c = 0; c < n_cols; ++c) {
            if (!detections[c].is_negative &&
                is_near_negative(detections[c].box, detections[c].world, negative)) {
                det_suppressed[c] = true;
            }
        }

        for (auto& track : tracks_) {
            if (track.initialized && track.state != TrackState::DEAD &&
                is_near_negative(track.last_box, track.last_world, negative)) {
                track.negative_suppressed = true;
            }
        }
    }

    if (n_rows > 0 && n_cols > 0) {
        radar_core::tracker::HungarianAlgorithm hungarian;

        std::vector<std::vector<float>> cost_matrix(
            n_rows,
            std::vector<float>(n_cols, INF_COST)
        );

        for (int r = 0; r < n_rows; ++r) {
            const auto& track = tracks_[r];

            if (!track.initialized || track.state == TrackState::DEAD) {
                continue;
            }

            for (int c = 0; c < n_cols; ++c) {
                const auto& det = detections[c];

                if (det_suppressed[c]) {
                    continue;
                }

                // 队伍不同，硬过滤
                if (det.team_id != track.team_id) {
                    continue;
                }

                // 1. box gate：只用物理距离做 gate
                float d_box = box_center_distance(track.last_box, det.box);
                if (d_box > params_.max_gate_box) {
                    continue;
                }

                // 2. world gate：如果启用，则也做物理 gate
                float d_world = 0.0f;
                if (params_.max_gate_world > 0.0f) {
                    d_world = point_distance(track.last_world, det.world);
                    if (d_world > params_.max_gate_world) {
                        continue;
                    }
                }

                // 3. Kalman 创新门控：使用当前预测协方差自适应调整 gate。
                // 相比固定距离阈值，预测越不确定时允许的空间范围越大，
                // 稳定跟踪时则会更严格地拒绝离群观测。
                const std::vector<float> box_meas = {
                    det.box.x + det.box.width / 2.0f,
                    det.box.y + det.box.height / 2.0f,
                    static_cast<float>(det.box.width),
                    static_cast<float>(det.box.height)
                };
                const bool box_gate_rejected =
                    params_.kalman_gate_box > 0.0f &&
                    track.kf_box.innovationSquared(box_meas) > params_.kalman_gate_box;
                const bool world_gate_rejected =
                    params_.kalman_gate_world > 0.0f &&
                    track.kf_world.innovationSquared({det.world.x, det.world.y}) >
                        params_.kalman_gate_world;
                if (box_gate_rejected || world_gate_rejected) {
                    continue;
                }

                // 4. 归一化物理代价
                float box_norm = d_box / std::max(params_.max_gate_box, 1.0f);

                float world_norm = 0.0f;
                if (params_.max_gate_world > 0.0f) {
                    world_norm = d_world / std::max(params_.max_gate_world, 1e-3f);
                }

                float cost = params_.w_box * box_norm
                           + params_.w_world * world_norm;

                // 5. class 只做可靠度相关的软惩罚，不参与 gate 或直接拒绝匹配
                cost += calculate_class_mismatch_penalty(track, det);

                cost_matrix[r][c] = cost;
            }
        }

        std::vector<int> assignment;
        hungarian.Solve(cost_matrix, assignment);

        // --------------------------------------------------
        // Step 3: 处理匹配结果
        // --------------------------------------------------
        for (int r = 0; r < n_rows; ++r) {
            auto& track = tracks_[r];

            if (!track.initialized || track.state == TrackState::DEAD) {
                continue;
            }

            int c = -1;
            if (r < static_cast<int>(assignment.size())) {
                c = assignment[r];
            }

            bool matched = (
                c >= 0 &&
                c < n_cols &&
                cost_matrix[r][c] < INF_COST * 0.5f
            );

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
                    auto box_upd = track.kf_box.update(
                        box_meas, params_.kalman_gate_box);

                    track.last_box = cv::Rect(
                        static_cast<int>(box_upd[0] - box_upd[2] / 2.0f),
                        static_cast<int>(box_upd[1] - box_upd[3] / 2.0f),
                        static_cast<int>(box_upd[2]),
                        static_cast<int>(box_upd[3])
                    );

                    auto world_upd = track.kf_world.update(
                        {det.world.x, det.world.y}, params_.kalman_gate_world);
                    track.last_world = cv::Point2f(world_upd[0], world_upd[1]);

                    track.detected_world = det.world;
                }

                if (track.hit_count == 0) {
                    track.last_box = det.box;
                    track.last_world = det.world;
                    track.detected_world = det.world;
                }

                track.bot_id.updateRecent(det.class_id, det.class_conf);
                if (det.class_conf >= params_.min_identity_update_conf) {
                    track.bot_id.update(det.class_id, det.class_conf, det.class_margin);
                }
                update_identity_state(track, det);

                track.hit_count++;
                track.miss_count = 0;
                track.state = TrackState::ACTIVE;
                track.observed_this_update = true;
                track.last_observed_time_ns = current_time_ns_;
                track.lost_duration_s = 0.0f;
                track.dead_since_time_ns = 0;

                track.last_score = det.score;
                track.last_is_dead = det.is_dead;

                det_matched[c] = true;
            } else {
                // 未匹配到观测
                if (track.last_is_dead) {
                    // 死亡车辆冻结，不增加 miss
                } else {
                    track.miss_count++;
                    update_lifecycle(track);
                    track.bot_id.markLost(track.lost_duration_s);
                }
            }
        }
    } else if (n_cols == 0) {
        // --------------------------------------------------
        // 无 detection，所有已初始化且非死亡 track 标记为丢失
        // --------------------------------------------------
        for (auto& track : tracks_) {
            if (!track.initialized || track.last_is_dead) {
                continue;
            }

            track.miss_count++;
            update_lifecycle(track);
            track.bot_id.markLost(track.lost_duration_s);
        }
    }

    // ------------------------------------------------------
    // Step 4: 未匹配 detection 创建新 PhysicalTrack
    // ------------------------------------------------------
    for (int c = 0; c < n_cols; ++c) {
        if (det_matched[c] || det_suppressed[c]) {
            continue;
        }

        if (static_cast<int>(tracks_.size()) >= params_.max_tracks) {
            break;
        }

        const auto& det = detections[c];

        PhysicalTrack track;
        track.track_id = next_track_id_++;
        track.team_id  = det.team_id;

        track.bot_id.configure(params_.botIdentity);

        track.bot_id.updateRecent(det.class_id, det.class_conf);
        if (det.class_conf >= params_.min_identity_update_conf) {
            track.bot_id.update(det.class_id, det.class_conf, det.class_margin);
        }
        update_identity_state(track, det);

        track.initialized = true;
        track.state = TrackState::ACTIVE;

        track.hit_count = 1;
        track.miss_count = 0;
        track.observed_this_update = true;
        track.last_observed_time_ns = current_time_ns_;
        track.lost_duration_s = 0.0f;

        track.last_box = det.box;
        track.last_world = det.world;
        track.detected_world = det.world;

        track.last_score = det.score;
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

    // ------------------------------------------------------
    // Step 5: 清理 DEAD 超时 track
    // ------------------------------------------------------
    tracks_.erase(
        std::remove_if(
            tracks_.begin(),
            tracks_.end(),
            [this](const PhysicalTrack& t) {
                if (t.state != TrackState::DEAD || t.dead_since_time_ns <= 0) {
                    return false;
                }
                const float dead_duration_s = static_cast<float>(
                    std::max<std::int64_t>(0, current_time_ns_ - t.dead_since_time_ns)) * 1e-9f;
                return dead_duration_s > params_.dead_retention_time_s;
            }
        ),
        tracks_.end()
    );

    // ------------------------------------------------------
    // Step 6: 根据 tracks_ 构建稳定 slot 输出
    // ------------------------------------------------------
    build_outputs_with_slot_owner();
}
