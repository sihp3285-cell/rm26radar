#pragma once
#include <string>
#include <vector>
#include <stdexcept>
#include <opencv2/core.hpp>
#include <radar27_tracking/bot_identity.hpp>
struct TrackerConfig {
    // ========== Track 生命周期 ==========
    float maxLostTimeS = 0.30f;
    float maxPredictTimeS = 0.20f;
    float deadRetentionTimeS = 0.10f;
    float deadTargetHoldTimeS = 0.10f;
    int minHit = 2;
    int maxTracks = 20;

    // ========== 物理匹配 gate ==========
    float maxGateBox = 300.0f;
    float maxGateWorld = 2.5f;
    float kalmanGateBox = 18.467f;
    float kalmanGateWorld = 13.816f;
    float negativeGateBox = 200.0f;
    float negativeGateWorld = 1.0f;

    // ========== Hungarian 匹配代价 ==========
    float wBox = 1.0f;
    float wWorld = 1.0f;
    float classMismatchMinPenalty = 0.05f;
    float classMismatchPenalty = 0.40f;  // 最大类别不一致软惩罚

    // ========== BotIdentity 身份稳定器 ==========
    BotIdentityConfig botIdentity;

    // ========== 身份更新阈值 ==========
    float minIdentityUpdateConf = 0.20f;
    int identityConfirmObservations = 3;
    int identitySwitchConfirmObservations = 5;
    // 帧数确认的时间门（毫秒）：高 FPS 下保持确认所需物理时间稳定，<=0 关闭
    float identityConfirmMinTimeMs = 135.0f;
    float identitySwitchConfirmMinTimeMs = 540.0f;

    // ========== Official slot owner 机制 ==========
    float slotBindMinConf = 0.40f;
    float slotLeaseTimeS = 0.30f;
    float slotMinStability = 0.70f;
    float slotMaxSwitchRate = 0.35f;
    float maxSlotJumpDist = 2.5f;
};
class TrackingConfig {
public:
 explicit TrackingConfig(const std::string& configDir);
TrackerConfig tracker;
private:
void loadTrackerConfig(const std::string& path);
static void validateTrackerConfig(const TrackerConfig& cfg);
};
