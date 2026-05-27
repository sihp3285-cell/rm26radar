#pragma once

#include <deque>
#include <vector>
#include <utility>
#include <cmath>

// ==========================================
// BotIdentityConfig - BotIdentity 可配置参数
// ==========================================
struct BotIdentityConfig {
    int maxHistory = 50;
    int purgeThreshold = 30;
    int minHistoryForStable = 8;
    float decay = 0.97f;
    int numClasses = 9; // CAR=0, ARMOR=1, R1=2, R2=3, R3=4, R4=5, S=6, OUTPOST=7, AIRPLANE=8
};

// ==========================================
// BotIdentity - 单个物理机器人的身份轨迹池
//   跨帧持久化保存最近 N 帧的 class_id / class_conf
//   使用指数加权计算历史类别分布，输出稳定身份
// ==========================================
class BotIdentity {
public:
    struct Observation {
        int class_id;
        float class_conf;
    };

    BotIdentity() = default;
    explicit BotIdentity(const BotIdentityConfig& cfg);

    void configure(const BotIdentityConfig& cfg);

    // 收到新观测时调用
    void update(int class_id, float class_conf);

    // 本帧未匹配到时调用
    void markLost();

    // 强制清空历史
    void reset();

    bool empty() const;
    bool shouldPurge() const;
    int getLostCounter() const { return lost_counter_; }
    size_t getHistorySize() const { return history_.size(); }

    // 返回 {stable_class_id, normalized_confidence}
    std::pair<int, float> getStableClass() const;

private:
    int maxHistory_ = 50;
    int purgeThreshold_ = 30;
    int minHistoryForStable_ = 8;
    float decay_ = 0.97f;
    int numClasses_ = 9;

    std::deque<Observation> history_;
    int lost_counter_ = 0;
};
