#include <radar27_tracking/config.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <algorithm>
#include <sstream>
namespace fs=std::filesystem;
TrackingConfig::TrackingConfig(const std::string& configDir) {
 fs::path dir(configDir);
if(fs::exists(dir/"tracker.yaml")) loadTrackerConfig((dir/"tracker.yaml").string());
validateTrackerConfig(tracker);
}
void TrackingConfig::loadTrackerConfig(const std::string& path) {
    YAML::Node cfg = YAML::LoadFile(path);

    // ========== Track 生命周期 ==========
    tracker.maxLostTimeS = cfg["max_lost_time_s"] ? cfg["max_lost_time_s"].as<float>() : 0.30f;
    tracker.maxPredictTimeS = cfg["max_predict_time_s"] ? cfg["max_predict_time_s"].as<float>() : 0.20f;
    tracker.deadRetentionTimeS = cfg["dead_retention_time_s"] ? cfg["dead_retention_time_s"].as<float>() : 0.10f;
    tracker.deadTargetHoldTimeS = cfg["dead_target_hold_time_s"] ? cfg["dead_target_hold_time_s"].as<float>() : 0.10f;
    tracker.minHit = cfg["min_hit"] ? cfg["min_hit"].as<int>() : 2;
    tracker.maxTracks = cfg["max_tracks"] ? cfg["max_tracks"].as<int>() : 20;

    // ========== 物理匹配 gate ==========
    tracker.maxGateBox = cfg["max_gate_box"] ? cfg["max_gate_box"].as<float>() : 300.0f;
    tracker.maxGateWorld = cfg["max_gate_world"] ? cfg["max_gate_world"].as<float>() : 2.5f;
    tracker.kalmanGateBox = cfg["kalman_gate_box"] ? cfg["kalman_gate_box"].as<float>() : 18.467f;
    tracker.kalmanGateWorld = cfg["kalman_gate_world"] ? cfg["kalman_gate_world"].as<float>() : 13.816f;
    tracker.negativeGateBox = cfg["negative_gate_box"] ? cfg["negative_gate_box"].as<float>() : 200.0f;
    tracker.negativeGateWorld = cfg["negative_gate_world"] ? cfg["negative_gate_world"].as<float>() : 1.0f;

    // ========== Hungarian 匹配代价 ==========
    tracker.wBox = cfg["w_box"] ? cfg["w_box"].as<float>() : 1.0f;
    tracker.wWorld = cfg["w_world"] ? cfg["w_world"].as<float>() : 1.0f;
    tracker.classMismatchMinPenalty = cfg["class_mismatch_min_penalty"]
        ? cfg["class_mismatch_min_penalty"].as<float>() : 0.05f;
    tracker.classMismatchPenalty = cfg["class_mismatch_penalty"]
        ? cfg["class_mismatch_penalty"].as<float>() : 0.40f;

    // 两端先限制为非负；若配置顺序颠倒则交换，保留用户给出的完整区间。
    tracker.classMismatchMinPenalty = std::max(0.0f, tracker.classMismatchMinPenalty);
    tracker.classMismatchPenalty = std::max(0.0f, tracker.classMismatchPenalty);
    if (tracker.classMismatchMinPenalty > tracker.classMismatchPenalty) {
        std::swap(tracker.classMismatchMinPenalty, tracker.classMismatchPenalty);
    }

    // ========== 身份更新阈值 ==========
    tracker.minIdentityUpdateConf = cfg["min_identity_update_conf"] ? cfg["min_identity_update_conf"].as<float>() : 0.20f;
    tracker.identityConfirmObservations = cfg["identity_confirm_observations"] ? cfg["identity_confirm_observations"].as<int>() : 3;
    tracker.identitySwitchConfirmObservations = cfg["identity_switch_confirm_observations"] ? cfg["identity_switch_confirm_observations"].as<int>() : 5;
    // 帧数确认的时间门（毫秒）：高 FPS 下保持确认所需物理时间稳定，<=0 关闭
    tracker.identityConfirmMinTimeMs = cfg["identity_confirm_min_time_ms"] ? cfg["identity_confirm_min_time_ms"].as<float>() : 135.0f;
    tracker.identitySwitchConfirmMinTimeMs = cfg["identity_switch_confirm_min_time_ms"] ? cfg["identity_switch_confirm_min_time_ms"].as<float>() : 540.0f;

    // ========== Official slot owner 机制 ==========
    tracker.slotBindMinConf = cfg["slot_bind_min_conf"] ? cfg["slot_bind_min_conf"].as<float>() : 0.40f;
    tracker.slotLeaseTimeS = cfg["slot_lease_time_s"] ? cfg["slot_lease_time_s"].as<float>() : 0.30f;
    tracker.slotMinStability = cfg["slot_min_stability"] ? cfg["slot_min_stability"].as<float>() : 0.70f;
    tracker.slotMaxSwitchRate = cfg["slot_max_switch_rate"] ? cfg["slot_max_switch_rate"].as<float>() : 0.35f;
    tracker.maxSlotJumpDist = cfg["max_slot_jump_dist"] ? cfg["max_slot_jump_dist"].as<float>() : 2.5f;

    // ========== BotIdentity 身份稳定器 ==========
    if (cfg["bot_identity"]) {
        YAML::Node bi = cfg["bot_identity"];
        tracker.botIdentity.maxHistory = bi["max_history"] ? bi["max_history"].as<int>() : 50;
        tracker.botIdentity.purgeAfterLostTimeS = bi["purge_after_lost_time_s"] ? bi["purge_after_lost_time_s"].as<float>() : 1.0f;
        tracker.botIdentity.minHistoryForStable = bi["min_history_for_stable"] ? bi["min_history_for_stable"].as<int>() : 8;
        tracker.botIdentity.decay = bi["decay"] ? bi["decay"].as<float>() : 0.97f;
        tracker.botIdentity.numClasses = bi["num_classes"] ? bi["num_classes"].as<int>() : 8;
    }
}
void TrackingConfig::validateTrackerConfig(const TrackerConfig& cfg) {
    if (cfg.maxPredictTimeS < 0.0f || cfg.maxLostTimeS < 0.0f ||
        cfg.deadRetentionTimeS < 0.0f || cfg.deadTargetHoldTimeS < 0.0f ||
        cfg.slotLeaseTimeS < 0.0f || cfg.botIdentity.purgeAfterLostTimeS < 0.0f) {
        throw std::runtime_error("Tracker 物理时间参数不能为负数");
    }
    if (cfg.maxPredictTimeS > cfg.maxLostTimeS) {
        throw std::runtime_error("max_predict_time_s 不能大于 max_lost_time_s");
    }
    if (cfg.minHit <= 0 || cfg.maxTracks <= 0 ||
        cfg.identityConfirmObservations <= 0 ||
        cfg.identitySwitchConfirmObservations <= 0 ||
        cfg.botIdentity.maxHistory <= 0 ||
        cfg.botIdentity.minHistoryForStable <= 0) {
        throw std::runtime_error("Tracker 观测计数参数必须大于 0");
    }
}