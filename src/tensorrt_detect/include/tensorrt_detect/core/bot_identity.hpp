/**
 * @file bot_identity.hpp
 * @brief 将易抖动的逐帧兵种分类累积为轨迹级稳定身份。
 *
 * 每个 PhysicalTrack 独占一个 BotIdentity。它只维护类别证据，不决定像素/世界
 * 关联，也不直接占用 official slot；Tracker 在证据连续确认后提交身份，再由
 * SlotOwner 仲裁唯一槽位归属。
 */
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
        int class_id;       // 本帧分类器 top1 语义类别。
        float class_conf;   // top1 置信度，作为历史投票权重的一部分。
        float class_margin; // top1-top2，表达本帧分类的可分性。
    };

    struct Stats {
        int class_id = -1;//历史加权总分最高的类别
        float confidence = 0.0f;//历史加权总分最高的类别置信度
        float margin = 0.0f;//top1-top2 再按类别加权平均
        float switch_rate = 0.0f;//历史观测中身份切换所占比例
        float stability = 0.0f;//身份稳定性
    };

    /** 构造使用字段默认值的空身份历史。 */
    BotIdentity() = default;
    /** 使用配置初始化空历史；等价于构造后调用 configure()。 */
    explicit BotIdentity(const BotIdentityConfig& cfg);

    /** 应用新配置并清空旧证据，避免不同参数体系下的分数混用。 */
    void configure(const BotIdentityConfig& cfg);

    /** 写入一条通过 Tracker 置信门限的长期类别证据，并把 lost duration 归零。 */
    void update(int class_id, float class_conf, float class_margin);
    /** 写入供槽位接管比较使用的近期类别/置信度，不改变长期统计公式。 */
    void updateRecent(int class_id, float class_conf);

    // 本次更新未匹配到时调用，参数是从最后真实观测开始累计的秒数。
    void markLost(float lost_duration_s);

    /** 强制清空长期/近期证据和丢失计时；PhysicalTrack 重置时调用。 */
    void reset();

    /** 返回长期与近期历史是否都为空。 */
    bool empty() const;
    /** 判断累计丢失时间是否超过配置的身份证据清理阈值。 */
    bool shouldPurge() const;
    /** 返回从最后一次有效身份更新起的累计丢失秒数。 */
    float getLostDurationS() const { return lost_duration_s_; }
    /** 返回长期身份投票窗口当前保存的观测数量。 */
    size_t getHistorySize() const { return history_.size(); }

    /** 返回稳定类别及归一化置信度；证据不足时类别为 -1。 */
    std::pair<int, float> getStableClass() const;
    /** 计算类别、置信度、margin、切换率和 stability 的完整诊断快照。 */
    Stats getStats() const;
    /** 返回近期窗口中指定类别的加权置信度，供同槽候选接管排序。 */
    float getRecentConfidence(int class_id) const;

private:
    int maxHistory_ = 50;
    float purgeAfterLostTimeS_ = 1.0f;
    int minHistoryForStable_ = 8;
    float decay_ = 0.97f;
    int numClasses_ = 9;

    std::deque<Observation> history_; // 经过最低置信度门的长期稳定性证据。
    std::deque<std::pair<int, float>> recent_history_; // 近期接管仲裁用的轻量证据。
    float lost_duration_s_ = 0.0f; // 自 PhysicalTrack 最后真实匹配起的物理秒数。
};
