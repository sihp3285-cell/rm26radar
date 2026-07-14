#pragma once

#include "tracker_types.hpp"
#include "kalman.hpp"
#include "bot_identity.hpp"
#include "robot_id.hpp"

#include <array>
#include <vector>

// BotIdentityConfig 定义在 bot_identity.hpp 中，此处直接复用

// ==========================================
// Tracker - PhysicalTrack 多目标跟踪器
//   内部用动态 PhysicalTrack 按物理连续性跟踪目标
//   BotIdentity 负责轨迹级身份稳定
//   SlotOwner 负责 official slot 的持久归属锁定
//
// 核心目标：
//   1. Tracker 匹配阶段主要相信物理连续性，分类只作为轻量软惩罚
//   2. 只有连续确认后的 committed_class 才能参与 slot 绑定
//   3. official slot 一旦绑定 track_id，owner 短期丢失/遮挡时不被其他 track 抢走
// ==========================================
struct TrackerParams {
    // ========== Track 生命周期 ==========
    int max_miss = 4;                // 连续丢失多少帧后 track 标记为 DEAD
    int max_predict = 2;             // 连续丢失多少帧内保持 PREDICTED，卡尔曼外推仍显示
    int min_hit = 2;                 // 最少命中次数才允许对外输出 / 绑定 slot
    int max_tracks = 20;             // 最大同时跟踪的 PhysicalTrack 数量

    // ========== 物理匹配 gate ==========
    float max_gate_box = 300.0f;     // 像素框中心距离门限

    // 世界坐标 gate；单位取决于 WorldMeasurement::world。
    // 如果 world 是米制坐标，可先尝试 2.0~3.0；如果暂时不用，设为 <= 0。
    float max_gate_world = 2.5f;

    // Kalman 创新 NIS 门控阈值（卡方分布）。box 为 4 维，world 为 2 维；
    // 默认值对应约 99.9% 置信区间，<= 0 可关闭对应门控。
    float kalman_gate_box = 18.467f;
    float kalman_gate_world = 13.816f;

    // 死亡装甲板负观测的邻域。启用的维度必须同时满足，避免抑制相邻活车；
    // 对应值 <= 0 时禁用该维度，两个都禁用则关闭负观测抑制。
    float negative_gate_box = 200.0f;
    float negative_gate_world = 1.0f;

    // ========== Hungarian 匹配代价 ==========
    // 新版建议使用归一化代价：
    // cost = w_box * box_norm + w_world * world_norm + class_penalty
    float w_box = 1.0f;
    float w_world = 1.0f;

    // 类别不一致只增加软惩罚，不参与 gate。惩罚在最小值和最大值之间，
    // 由轨迹身份稳定性、切换率和当前检测分类置信度共同决定。
    float class_mismatch_min_penalty = 0.05f;
    float class_mismatch_penalty = 0.40f;  // 最大类别不一致惩罚

    // ========== BotIdentity 身份稳定器 ==========
    BotIdentityConfig botIdentity;

    // 低于该置信度的分类结果不写入 BotIdentity，避免遮挡/低质量 ROI 污染身份池。
    // 低于该值的 class_conf 不写入身份历史。
    float min_identity_update_conf = 0.20f;

    // 新身份和身份切换都必须连续多帧确认。
    int identity_confirm_frames = 3;
    int identity_switch_confirm_frames = 5;

    // ========== Official slot owner 机制 ==========
    // track 第一次绑定 official slot 时，stable confidence 至少要达到该值。
    float slot_bind_min_conf = 0.40f;

    // owner 不可输出后进入 RESERVED，租约独立于 track 的 miss_count。
    int slot_lease_frames = 8;

    // 不稳定身份禁止参与空槽竞争。
    float slot_min_stability = 0.70f;
    float slot_max_switch_rate = 0.35f;

    // 防止同一个 official slot 在地图上瞬移。
    // 单位同 WorldMeasurement::world；若暂时不用，设为 <= 0。
    float max_slot_jump_dist = 2.5f;
};

class Tracker {
public:
    // 固定槽位索引：Red R1~R4,S | Blue R1~R4,S
    // 注：Outpost 不走 Tracker，由 pose_node 直接透传
    static constexpr int NUM_SLOTS = 10;

    static constexpr int SLOT_RED_R1   = 0;
    static constexpr int SLOT_RED_R2   = 1;
    static constexpr int SLOT_RED_R3   = 2;
    static constexpr int SLOT_RED_R4   = 3;
    static constexpr int SLOT_RED_S    = 4;

    static constexpr int SLOT_BLUE_R1  = 5;
    static constexpr int SLOT_BLUE_R2  = 6;
    static constexpr int SLOT_BLUE_R3  = 7;
    static constexpr int SLOT_BLUE_R4  = 8;
    static constexpr int SLOT_BLUE_S   = 9;

    explicit Tracker(const TrackerParams& params = TrackerParams());

    // 输入新一帧观测，更新所有 track 状态，并在 update 末尾刷新 last_outputs_。
    // dt 使用 ROS 消息时间戳计算的真实帧间隔（秒）；<= 0 时滤波器使用默认值。
    void update(const std::vector<WorldMeasurement>& detections, float dt = -1.0f);

    // 单个槽位的对外输出状态
    struct SlotOutput {
        int slot_idx = 0;

        // official nominal 身份：这个槽本来代表哪个队伍、哪个兵种
        int team_id = robot_id::UNKNOWN;
        int class_id = robot_id::UNKNOWN;

        // 当前 slot 实际由哪个 PhysicalTrack 输出。
        // 这是排查“地图点在两台机器人之间跳变”的关键字段。
        int track_id = -1;

        bool valid = false;          // 当前是否有效：ACTIVE / PREDICTED 且满足 min_hit
        TrackState state = TrackState::DEAD;

        cv::Rect smoothed_box;
        cv::Point2f smoothed_world;  // 建议输出 Kalman 后的 last_world

        bool is_dead = false;
        float score = 0.0f;

        // BotIdentity / SlotOwner 输出给下游看的稳定身份
        int stable_class_id = -1;
        float stable_class_conf = 0.0f;

        // 当前 owner 已连续丢失帧数，调试 slot owner 释放/接管时很有用
        int owner_lost_frames = 0;
    };

    // 获取指定 official slot 的当前状态，兼容旧接口
    SlotOutput get_slot(int idx) const;

    // 批量获取 10 个 official slot 输出。
    // 新版中 get_outputs() 只是返回 update() 缓存好的 last_outputs_，不再每次重新分配槽位。
    std::vector<SlotOutput> get_outputs() const;

    void reset();

private:
    struct PhysicalTrack {
        int track_id = -1;               // 唯一 ID，不能用 vector 下标替代
        int team_id = robot_id::UNKNOWN; // 首次匹配时设定，之后不变

        int hit_count = 0;
        int miss_count = 0;

        TrackState state = TrackState::DEAD;
        bool initialized = false;        // Kalman 是否已初始化
        bool negative_suppressed = false; // 本帧命中死亡负观测，不输出身份

        KalmanFilterBox kf_box;          // 8维 [cx, cy, w, h, vx, vy, vw, vh]
        KalmanFilter2d kf_world;         // 4维 [x, z, vx, vz]

        BotIdentity bot_id;              // 身份轨迹池：跨帧持久化 class 历史
        int committed_class = -1;         // 连续确认后才用于槽位映射
        int pending_class = -1;           // 正在连续确认的候选身份
        int pending_class_frames = 0;

        cv::Rect last_box;
        cv::Point2f last_world;          // Kalman 后的平滑世界坐标
        cv::Point2f detected_world;      // 本帧实际检测到的世界坐标，仅调试/对比用

        float last_score = 0.0f;
        bool last_is_dead = false;
    };

    // official slot 的持久 owner。
    // 只要 owner track 还活着或短期丢失，其他 track 不允许因为误分类抢槽。
    enum class SlotOwnerState {
        FREE,
        RESERVED,
        OWNED
    };

    struct SlotOwner {
        SlotOwnerState state = SlotOwnerState::FREE;
        int track_id = -1;
        int lease_frames = 0;

        cv::Point2f last_world{0.0f, 0.0f};
        bool has_last_world = false;

        float last_conf = 0.0f;
    };

    // 某个 track 试图绑定/接管某个 official slot 时的候选信息
    struct SlotCandidate {
        int track_index = -1;
        int slot_idx = -1;

        int stable_class_id = -1;
        float stable_class_conf = 0.0f;
        float priority = 0.0f;
    };

private:
    TrackerParams params_;

    std::vector<PhysicalTrack> tracks_;
    int next_track_id_ = 0;

    // 10 个 official slot 的持久 owner
    std::array<SlotOwner, NUM_SLOTS> slot_owners_;

    // update() 后缓存好的 10 个 official slot 输出
    std::vector<SlotOutput> last_outputs_;

private:
    static float box_center_distance(const cv::Rect& a, const cv::Rect& b);
    static float point_distance(const cv::Point2f& a, const cv::Point2f& b);

    // committed 身份与检测类别冲突时，根据双方可靠度计算软惩罚；否则返回 0。
    float calculate_class_mismatch_penalty(
        const PhysicalTrack& track,
        const WorldMeasurement& detection
    ) const;

    // 将 (team, committed_class) 映射到 official slot 索引 0-9，无法映射返回 -1
    static int slot_for(int team, int class_id);

    // 初始化 10 个空 official slot 输出
    std::vector<SlotOutput> make_empty_outputs() const;

    PhysicalTrack* find_track_by_id(int track_id);
    const PhysicalTrack* find_track_by_id(int track_id) const;

    bool is_output_state(TrackState state) const;
    void reserve_owner(SlotOwner& owner);
    void update_identity_state(PhysicalTrack& track, const WorldMeasurement& detection);

    void fill_slot_output(
        SlotOutput& out,
        int slot_idx,
        const PhysicalTrack& track,
        int stable_class_id,
        float stable_class_conf,
        int owner_lost_frames
    ) const;

    // 在 update() 最后调用：根据 tracks_、BotIdentity 和 slot_owners_ 生成稳定 official outputs
    void build_outputs_with_slot_owner();

    // 旧版保留：如果你还没有切换到 SlotOwner 版 tracker.cpp，可以临时兼容。
    // SlotOwner 版不再依赖这个函数。
    static void arbitrate_outputs(std::vector<SlotOutput>& outputs);
};
