#pragma once

#include <deque>
#include <vector>
#include <utility>
#include <cmath>

// ==========================================
// BotIdentity - 单个物理机器人的身份轨迹池
//   跨帧持久化保存最近 50 帧的 class_id / class_conf
//   使用指数加权计算历史类别分布，输出稳定身份
// ==========================================
class BotIdentity {
public:
    static constexpr int MAX_HISTORY = 50;
    static constexpr int PURGE_THRESHOLD = 30;
    static constexpr int MIN_HISTORY_FOR_STABLE = 8;
    static constexpr float DECAY = 0.97f;
    static constexpr int NUM_CLASSES = 9; // CAR=0, ARMOR=1, R1=2, R2=3, R3=4, R4=5, S=6, OUTPOST=7, AIRPLANE=8

    struct Observation {
        int class_id;
        float class_conf;
    };

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
    std::deque<Observation> history_;
    int lost_counter_ = 0;
};
