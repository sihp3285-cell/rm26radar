/**
 * @file prior_types.hpp
 * @brief Position Prior 从离线分布到在线 Gate 的内部数据契约。
 *
 * 本文件不依赖 ROS。Point2d 在该包中可能表示 world/field/canonical，具体坐标系由
 * 字段名或函数签名限定；裸 Point2d 本身不携带 frame 信息，调用时必须保持语义。
 */
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace position_prior {

struct Point2d {
    double x = 0.0;
    double y = 0.0;
};

struct Rect2d {
    double min_x = 0.0;
    double min_y = 0.0;
    double max_x = 0.0;
    double max_y = 0.0;

    /** 对闭区间边界做包含判断，用于 blocked region 安全门。 */
    bool contains(const Point2d& point) const {
        return point.x >= min_x && point.x <= max_x &&
               point.y >= min_y && point.y <= max_y;
    }
};

enum class FallbackLevel : std::uint8_t {
    LOCAL_ZONE = 0,
    GLOBAL_ROLE = 1,
};

/** 模型 query 后、在线 Gate 前的候选概率质量。 */
struct PriorCandidate {
    int grid_index = -1;
    Point2d canonical;
    double probability = 0.0; // 离线/盲区/锚点先验质量，尚未乘在线运动权重。
    bool from_blind_zone = false;
    bool stay_anchor = false;
};

/**
 * PositionPriorModel::query 的分布结果。query_top_k 在这里已经生效，但尚未做
 * 可达性、距离、运动 Gate，也尚未受 output_top_k 截断。
 */
struct PriorDistribution {
    bool valid = false;
    std::string error;
    std::string role;
    std::string context;
    int horizon_seconds = 0;
    int zone_index = -1;
    FallbackLevel fallback_level = FallbackLevel::GLOBAL_ROLE;
    std::uint32_t sample_count = 0;
    double local_weight = 0.0; // local zone 与 global role 分布混合时的局部权重。
    double stay_probability = 0.0; // 离线统计中仍停留在当前区域的质量。
    double retained_probability_mass = 0.0; // 稀疏导出候选保留的原分布质量。
    double normalized_entropy = 1.0; // 候选不确定性：越接近 1 越分散。
    std::vector<PriorCandidate> candidates;
};

/** 单候选经过 NavGrid/距离/运动在线约束后的诊断结果。 */
struct GatedCandidate {
    PriorCandidate prior;
    double fused_probability = 0.0; // 在线权重融合并归一化后的候选间相对质量。
    bool reachable = false;
    bool blocked = false;
    double distance_from_last_m = 0.0;
    double straight_distance_from_last_m = 0.0;
};

/**
 * PriorGate 的最终输出。confidence 是整次预测可信度；它与某个候选的
 * fused_probability 含义不同。predicted_canonical 还可能是候选加权均值后 snap
 * 的点，并不保证等于 candidates[0]。
 */
struct GateResult {
    bool valid = false;
    std::string rejection_reason;
    Point2d predicted_canonical;
    Point2d motion_prediction_canonical;
    double confidence = 0.0;
    double reachable_probability_mass = 0.0; // Gate 前质量中物理可达的比例。
    double normalized_entropy = 1.0;
    bool motion_gated = false;
    bool mesh_used = false;
    bool blind_zone_biased = false;
    double blind_zone_probability_mass = 0.0;
    double stay_anchor_probability_mass = 0.0;
    std::vector<GatedCandidate> candidates;
};

}  // namespace position_prior
