#pragma once

#include <deque>
#include <vector>
#include <utility>
#include <cmath>

// ==========================================
// BotIdentityConfig - BotIdentity 可配置参数
// ==========================================
struct BotIdentityConfig {
    int maxHistory = 50;//最多保留的有效观测次数
    float purgeAfterLostTimeS = 1.0f;//连续丢失多久之后自动清空
    int minHistoryForStable = 8;//最少积累多少次观测才输出稳定身份
    float decay = 0.97f;//指数衰减因子
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
        float class_margin;
    };

    struct Stats {
        int class_id = -1;//历史加权总分最高的类别
        float confidence = 0.0f;//历史加权总分最高的类别置信度
        float margin = 0.0f;//top1-top2 再按类别加权平均
        float switch_rate = 0.0f;//历史观测中身份切换所占比例
        float stability = 0.0f;//身份稳定性
    };

    BotIdentity() = default;
    explicit BotIdentity(const BotIdentityConfig& cfg);

    void configure(const BotIdentityConfig& cfg);

    // 收到新观测时调用
    void update(int class_id, float class_conf, float class_margin);
    void updateRecent(int class_id, float class_conf);

    // 本次更新未匹配到时调用，参数是从最后真实观测开始累计的秒数。
    void markLost(float lost_duration_s);

    // 强制清空历史
    void reset();

    bool empty() const;
    bool shouldPurge() const;
    float getLostDurationS() const { return lost_duration_s_; }
    size_t getHistorySize() const { return history_.size(); }

    // 返回 {stable_class_id, normalized_confidence}
    std::pair<int, float> getStableClass() const;
    Stats getStats() const;
    float getRecentConfidence(int class_id) const;

private:
    int maxHistory_ = 50;
    float purgeAfterLostTimeS_ = 1.0f;
    int minHistoryForStable_ = 8;
    float decay_ = 0.97f;
    int numClasses_ = 9;

    std::deque<Observation> history_;
    std::deque<std::pair<int, float>> recent_history_;
    float lost_duration_s_ = 0.0f;
};
