/**
 * @file position_prior_model.hpp
 * @brief 加载并查询离线条件位置先验模型。
 *
 * 模型以 role/context/当前位置 zone/预测 horizon 索引稀疏候选。节点启动时一次加载
 * 并可校验 SHA-256，实时 query 只读内存；本类不知道 Tracker、NavGrid 或 ROS。
 */
#pragma once

#include "position_prior/prior_types.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace position_prior {

/** 离线 probability 的只读查询层，由 PositionPriorNode 创建并贯穿节点生命周期。 */
class PositionPriorModel {
public:
    /** 加载离线 YAML 模型并可选校验 SHA-256；成功后查询数据保持只读。 */
    void load(const std::string& model_path, const std::string& expected_sha256 = "");

    /** 返回模型是否已完整通过版本、地图和结构校验。 */
    bool loaded() const { return loaded_; }
    /** 返回已加载模型文件路径。 */
    const std::string& model_path() const { return model_path_; }
    /** 返回加载时计算的模型文件小写 SHA-256。 */
    const std::string& model_sha256() const { return model_sha256_; }
    /** 返回模型 schema 版本。 */
    int model_version() const { return model_version_; }
    /** 返回模型声明的场地地图 ID。 */
    const std::string& map_id() const { return map_id_; }
    /** 返回模型 canonical 场地长度，单位米。 */
    double field_length() const { return field_length_; }
    /** 返回模型 canonical 场地宽度，单位米。 */
    double field_width() const { return field_width_; }
    /** 返回模型支持的有序预测 horizon 秒数列表。 */
    const std::vector<int>& horizons_seconds() const { return horizons_seconds_; }

    /** 判断模型是否包含指定兵种及其 context 分布。 */
    bool supports_role(const std::string& role) const;
    /** 将合法 canonical 位置映射到离散 zone index；无效或未加载返回 -1。 */
    int zone_index(const Point2d& canonical) const;
    /** 选择与真实丢失时长最接近的离线 horizon；无可用 horizon 返回 -1。 */
    int nearest_horizon(double lost_duration_s) const;

    /**
     * 先选 local zone 分布，必要时回退 global role，再按 top_k 截取模型候选。
     * 这里的 top_k 即 query_top_k，发生在 BlindZone/PriorGate 之前。
     */
    PriorDistribution query(
        const std::string& role,
        const Point2d& canonical_current,
        int horizon_seconds,
        const std::string& context = "all_phase",
        std::size_t top_k = 0) const;

    /** 流式读取文件并返回十六进制 SHA-256；文件或 OpenSSL 失败时抛异常。 */
    static std::string sha256_file(const std::string& path);

public:
    // 暴露仅用于轻量解析辅助；实时调用方使用 query()。
    struct DistributionData {
        std::uint32_t samples = 0;
        double local_weight = 0.0; // 离线导出的 local/global 混合权重。
        double stay_probability = 0.0; // 用于在线注入最后可靠位置锚点。
        double retained_probability_mass = 0.0;
        std::vector<PriorCandidate> candidates;
    };

    struct ContextData {
        DistributionData global;
        std::unordered_map<int, std::unordered_map<int, DistributionData>> zones;
    };

    struct RoleData {
        std::unordered_map<std::string, ContextData> contexts;
    };

private:
    /** 检查 canonical 点有限且位于模型几何边界内。 */
    bool valid_canonical(const Point2d& point) const;
    /** 以候选概率计算 [0,1] 归一化熵；空/零质量分布返回最大不确定性。 */
    static double normalized_entropy(const std::vector<PriorCandidate>& candidates);

    bool loaded_ = false;
    std::string model_path_;
    std::string model_sha256_;
    int model_version_ = 0;
    std::string map_id_;
    double field_length_ = 0.0;
    double field_width_ = 0.0;
    double zone_size_x_ = 0.0;
    double zone_size_y_ = 0.0;
    int zone_count_x_ = 0;
    int zone_count_y_ = 0;
    double grid_size_x_ = 0.0;
    double grid_size_y_ = 0.0;
    int grid_count_x_ = 0;
    int grid_count_y_ = 0;
    std::vector<int> horizons_seconds_;
    std::string default_context_ = "all_phase";
    std::unordered_map<std::string, RoleData> roles_;
};

}  // namespace position_prior
