#include "tracker.hpp"
#include "hungarian.hpp"
#include <cmath>
#include <algorithm>

// ==========================================
// Tracker 实现 - 固定槽位版
// ==========================================

Tracker::Tracker(const TrackerParams& params) : params_(params) {
    slots_.resize(NUM_SLOTS);

    auto init_slot = [this](int idx, int team, int cls) {
        slots_[idx].slot_idx = idx;
        slots_[idx].team_id  = team;
        slots_[idx].class_id = cls;
        slots_[idx].state    = TrackState::DEAD;
        slots_[idx].initialized = false;
        slots_[idx].hit_count = 0;
        slots_[idx].miss_count = 0;
    };

    // Red
    init_slot(SLOT_RED_R1, robot_id::RED, robot_id::R1);
    init_slot(SLOT_RED_R2, robot_id::RED, robot_id::R2);
    init_slot(SLOT_RED_R3, robot_id::RED, robot_id::R3);
    init_slot(SLOT_RED_R4, robot_id::RED, robot_id::R4);
    init_slot(SLOT_RED_S,  robot_id::RED, robot_id::S);
    // Blue
    init_slot(SLOT_BLUE_R1, robot_id::BLUE, robot_id::R1);
    init_slot(SLOT_BLUE_R2, robot_id::BLUE, robot_id::R2);
    init_slot(SLOT_BLUE_R3, robot_id::BLUE, robot_id::R3);
    init_slot(SLOT_BLUE_R4, robot_id::BLUE, robot_id::R4);
    init_slot(SLOT_BLUE_S,  robot_id::BLUE, robot_id::S);
    // 注：Outpost 不走 Tracker，由 pose_node 直接透传
}

void Tracker::reset() {
    for (auto& slot : slots_) {
        slot.state = TrackState::DEAD;
        slot.initialized = false;
        slot.hit_count = 0;
        slot.miss_count = 0;
        slot.last_box = cv::Rect();
        slot.last_world = cv::Point2f();
        slot.detected_world = cv::Point2f();
    }
}

float Tracker::box_center_distance(const cv::Rect& a, const cv::Rect& b) {
    float cx1 = a.x + a.width  / 2.0f;
    float cy1 = a.y + a.height / 2.0f;
    float cx2 = b.x + b.width  / 2.0f;
    float cy2 = b.y + b.height / 2.0f;
    return std::hypot(cx1 - cx2, cy1 - cy2);
}

Tracker::SlotOutput Tracker::get_slot(int idx) const {
    SlotOutput out;
    if (idx < 0 || idx >= NUM_SLOTS) return out;

    const auto& s = slots_[idx];
    out.slot_idx  = s.slot_idx;
    out.team_id   = s.team_id;
    out.class_id  = s.class_id;
    out.state     = s.state;
    out.smoothed_box   = s.last_box;
    // ACTIVE 状态优先使用原始检测世界坐标，避免卡尔曼融合/外推误差导致路径错乱
    out.smoothed_world = (s.state == TrackState::ACTIVE) ? s.detected_world : s.last_world;
    out.is_dead   = s.last_is_dead;
    out.score     = s.last_score;

    // valid = ACTIVE / PREDICTED 且满足 min_hit（LOST 状态不对外输出）
    out.valid = ((s.state == TrackState::ACTIVE) || (s.state == TrackState::PREDICTED))
                && (s.hit_count >= params_.min_hit);

    // BotIdentity 稳定身份
    auto [stable_cls, stable_conf] = s.bot_id.getStableClass();
    out.stable_class_id  = stable_cls;
    out.stable_class_conf = stable_conf;
    return out;
}

void Tracker::update(const std::vector<WorldMeasurement>& detections) {
    // ---- Step 1: ACTIVE/LOST 槽位先 predict；DEAD / 死亡车辆 保持原位不动 ----
    for (auto& slot : slots_) {
        if (!slot.initialized || slot.state == TrackState::DEAD) continue;
        // 死亡车辆冻结显示：不再 predict，位置保持在最后已知坐标
        if (slot.last_is_dead) continue;

        auto box_pred = slot.kf_box.predict();
        slot.last_box = cv::Rect(
            static_cast<int>(box_pred[0] - box_pred[2] / 2.0f),
            static_cast<int>(box_pred[1] - box_pred[3] / 2.0f),
            static_cast<int>(box_pred[2]),
            static_cast<int>(box_pred[3])
        );
        auto world_pred = slot.kf_world.predict();
        slot.last_world = cv::Point2f(world_pred[0], world_pred[1]);
    }

    // ---- Step 2: 全局匈牙利匹配（槽位 vs detections）----
    int n_rows = NUM_SLOTS;
    int n_cols = static_cast<int>(detections.size());
    std::vector<bool> det_matched(detections.size(), false);

    if (n_cols > 0) {
        radar_core::tracker::HungarianAlgorithm hungarian;
        std::vector<std::vector<float>> cost_matrix(
            n_rows, std::vector<float>(n_cols, 1e6f));

        for (int r = 0; r < n_rows; ++r) {
            const auto& slot = slots_[r];
            for (int c = 0; c < n_cols; ++c) {
                const auto& det = detections[c];
                // 硬过滤：team/class 必须严格匹配槽位（焊死）
                if (det.team_id != slot.team_id || det.class_id != slot.class_id) {
                    cost_matrix[r][c] = 1e6f;
                    continue;
                }
                // 未初始化或 DEAD 槽位：放宽门限，允许复活/首次激活
                float gate = (slot.initialized && slot.state != TrackState::DEAD)
                                 ? params_.max_gate_box : 1e6f;
                float d = box_center_distance(slot.last_box, det.box);
                if (d >= gate) {
                    cost_matrix[r][c] = 1e6f;
                    continue;
                }
                cost_matrix[r][c] = d;
            }
        }

        std::vector<int> assignment;
        hungarian.Solve(cost_matrix, assignment);

        for (int r = 0; r < n_rows; ++r) {
            auto& slot = slots_[r];
            int c = assignment[r];
            // 未初始化或 DEAD 槽位放宽门限，与代价矩阵构建逻辑一致
            float gate_check = (slot.initialized && slot.state != TrackState::DEAD)
                                   ? params_.max_gate_box : 1e6f;
            bool matched = (c >= 0 && c < n_cols && cost_matrix[r][c] < gate_check);

            if (matched) {
                const auto& det = detections[c];

                // ---- Kalman 更新（未初始化则 reset）----
                std::vector<float> box_meas = {
                    det.box.x + det.box.width  / 2.0f,
                    det.box.y + det.box.height / 2.0f,
                    static_cast<float>(det.box.width),
                    static_cast<float>(det.box.height)
                };
                if (!slot.initialized || slot.state == TrackState::DEAD) {
                    // 首次激活或 DEAD 复活：重置 Kalman 和 BotIdentity，同步所有 last_* 变量
                    slot.kf_box.reset(box_meas);
                    slot.kf_world.reset({det.world.x, det.world.y});
                    slot.bot_id.reset();
                    slot.initialized = true;
                    slot.last_box = det.box;
                    slot.last_world = det.world;
                    slot.detected_world = det.world;
                } else {
                    slot.detected_world = det.world;
                    auto box_upd = slot.kf_box.update(box_meas);
                    slot.last_box = cv::Rect(
                        static_cast<int>(box_upd[0] - box_upd[2] / 2.0f),
                        static_cast<int>(box_upd[1] - box_upd[3] / 2.0f),
                        static_cast<int>(box_upd[2]),
                        static_cast<int>(box_upd[3])
                    );
                    auto world_upd = slot.kf_world.update({det.world.x, det.world.y});
                    slot.last_world = cv::Point2f(world_upd[0], world_upd[1]);
                    slot.detected_world = det.world;
                }
                // 如果是首次激活，reset 后也要把 last_box/last_world/detected_world 同步
                if (slot.hit_count == 0) {
                    slot.last_box = det.box;
                    slot.last_world = det.world;
                    slot.detected_world = det.world;
                }

                // ---- BotIdentity 更新 ----
                slot.bot_id.update(det.class_id, det.score);

                // ---- 状态更新 ----
                slot.hit_count++;
                slot.miss_count = 0;
                slot.state = TrackState::ACTIVE;
                slot.last_score = det.score;
                slot.last_is_dead = det.is_dead;
                det_matched[c] = true;
            } else {
                // 未匹配到观测
                if (slot.last_is_dead) {
                    // 死亡车辆冻结：不增加 miss，保持 ACTIVE，持续显示在地图上
                } else {
                    slot.miss_count++;
                    slot.bot_id.markLost();
                    if (slot.miss_count > params_.max_miss) {
                        slot.state = TrackState::DEAD;
                    } else if (slot.miss_count <= params_.max_predict) {
                        slot.state = TrackState::PREDICTED;
                    } else {
                        slot.state = TrackState::LOST;
                    }
                }
            }
        }
    } else {
        // 无 detection，所有已初始化且非死亡槽位标记为丢失
        for (auto& slot : slots_) {
            if (!slot.initialized || slot.last_is_dead) continue;
            slot.miss_count++;
            slot.bot_id.markLost();
            if (slot.miss_count > params_.max_miss) {
                slot.state = TrackState::DEAD;
            } else if (slot.miss_count <= params_.max_predict) {
                slot.state = TrackState::PREDICTED;
            } else {
                slot.state = TrackState::LOST;
            }
        }
    }

    // 未匹配的 detection：在固定槽位架构下，它们只能是对应槽位已满/未激活
    // 由于每个 class 只有一个槽位，未匹配的 detection 直接丢弃，不会新建槽位
    // 这保证了“一个目标一个座位”的硬约束
}
