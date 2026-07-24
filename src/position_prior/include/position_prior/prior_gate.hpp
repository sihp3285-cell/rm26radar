#pragma once

#include "position_prior/blind_zone_prior.hpp"
#include "position_prior/navigation_mesh.hpp"
#include "position_prior/prior_types.hpp"

#include <unordered_map>
#include <vector>

namespace position_prior {

struct PriorGateConfig {
    double field_length = 28.0;
    double field_width = 15.0;
    double max_speed_mps = 5.0;
    double reachability_margin_m = 0.5;
    // 相对最后可靠观测点的绝对距离上限。<= 0 表示只使用速度可达约束。
    double max_guess_distance_m = 6.0;
    // 阈值内的近距离高斯偏好尺度。<= 0 表示不施加软距离偏好。
    double distance_preference_sigma_m = 3.0;
    // 观测点和模型候选点吸附到兵种可通行网格的最大距离。
    double mesh_snap_distance_m = 0.6;
    // 融合均值可能落到障碍物内，最终猜点会在该半径内吸附到同一连通分量。
    double mesh_prediction_snap_distance_m = 1.0;
    // 使用离线模型 stay_probability 在最后观测点注入锚点的缩放系数。
    double stay_anchor_probability_scale = 0.8;
    // 进入相机遮挡区时锚点的最低概率质量。高可信速度仍可通过运动权重
    // 把主猜点推向沿路候选，低速/速度噪声则优先保持最后可靠位置。
    double blind_zone_minimum_stay_anchor_mass = 0.70;
    // 最后一帧速度在遮挡后按该时间常数衰减；仅影响运动方向似然，不影响
    // 使用真实 lost_duration_s 计算的物理可达距离。
    double motion_velocity_decay_time_s = 3.0;
    double minimum_confidence = 0.05;
    std::size_t output_top_k = 5;
    std::unordered_map<int, double> motion_gate_mps{{2, 0.8}, {5, 0.4}, {10, 0.4}};
    std::unordered_map<int, double> motion_sigma_m{{2, 0.5}, {5, 3.0}, {10, 4.0}};
    std::vector<Rect2d> blocked_regions_canonical;
};

class PriorGate {
public:
    explicit PriorGate(PriorGateConfig config = {});

    const PriorGateConfig& config() const { return config_; }
    void set_navigation_mesh(const NavigationMesh* navigation_mesh) {
        navigation_mesh_ = navigation_mesh;
    }
    void set_blind_zone_prior(const BlindZonePrior* blind_zone_prior) {
        blind_zone_prior_ = blind_zone_prior;
    }

    GateResult apply(
        const PriorDistribution& distribution,
        const Point2d& last_canonical,
        const Point2d& canonical_velocity,
        double lost_duration_s,
        double last_tracking_confidence,
        const std::string& role = "",
        const NavigationRouteMap* prepared_routes = nullptr,
        double velocity_reliability = 1.0) const;

private:
    bool in_field(const Point2d& point) const;
    bool blocked(const Point2d& point) const;

    PriorGateConfig config_;
    const NavigationMesh* navigation_mesh_ = nullptr;
    const BlindZonePrior* blind_zone_prior_ = nullptr;
};

}  // namespace position_prior
