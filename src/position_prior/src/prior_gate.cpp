/**
 * @file prior_gate.cpp
 * @brief 候选可达性、距离偏好、速度似然、置信度与主猜点选择实现。
 *
 * path distance 优先取预计算 Dijkstra route map，失败/未启用时按代码规则回退；
 * fused_probability 是候选间归一化质量，confidence 则聚合跟踪可信度、离线样本、
 * entropy 与 reachable mass。二者用途不同，不能互相替代。
 */
#include "position_prior/prior_gate.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace position_prior {

namespace {

/** 计算候选先验质量的归一化熵；空或零质量候选按最大不确定性处理。 */
double normalized_entropy(const std::vector<PriorCandidate>& candidates) {
    if (candidates.size() <= 1) {
        return 0.0;
    }
    double total = 0.0;
    for (const auto& candidate : candidates) {
        total += std::max(0.0, candidate.probability);
    }
    if (total <= std::numeric_limits<double>::epsilon()) {
        return 1.0;
    }
    double entropy = 0.0;
    for (const auto& candidate : candidates) {
        const double probability =
            std::max(0.0, candidate.probability) / total;
        if (probability > 0.0) {
            entropy -= probability * std::log(probability);
        }
    }
    return std::clamp(
        entropy / std::log(static_cast<double>(candidates.size())),
        0.0, 1.0);
}

}  // namespace

PriorGate::PriorGate(PriorGateConfig config) : config_(std::move(config)) {}

bool PriorGate::in_field(const Point2d& point) const {
    return std::isfinite(point.x) && std::isfinite(point.y) &&
           point.x >= 0.0 && point.x <= config_.field_length &&
           point.y >= 0.0 && point.y <= config_.field_width;
}

bool PriorGate::blocked(const Point2d& point) const {
    return std::any_of(
        config_.blocked_regions_canonical.begin(),
        config_.blocked_regions_canonical.end(),
        [&point](const Rect2d& region) { return region.contains(point); });
}

GateResult PriorGate::apply(
    const PriorDistribution& distribution,
    const Point2d& last_canonical,
    const Point2d& canonical_velocity,
    double lost_duration_s,
    double last_tracking_confidence,
    const std::string& role,
    const NavigationRouteMap* prepared_routes,
    double velocity_reliability) const {
    GateResult result;
    if (!distribution.valid || distribution.candidates.empty()) {
        result.rejection_reason = "no_distribution";
        return result;
    }
    if (!in_field(last_canonical)) {
        result.rejection_reason = "last_position_out_of_field";
        return result;
    }

    // 1. 建立本次失联的运动参考。速度先按 Kalman covariance 可靠度缩放，再限制
    // 到车辆最大速度，并按遮挡时长指数衰减；它只提供方向偏好，不替代可达上限。
    const int horizon = distribution.horizon_seconds;
    const double reliable_velocity_weight =
        std::clamp(velocity_reliability, 0.0, 1.0);
    Point2d capped_velocity{
        canonical_velocity.x * reliable_velocity_weight,
        canonical_velocity.y * reliable_velocity_weight};
    const double raw_speed =
        std::hypot(capped_velocity.x, capped_velocity.y);
    if (raw_speed > config_.max_speed_mps && raw_speed > 0.0) {
        const double scale = config_.max_speed_mps / raw_speed;
        capped_velocity.x *= scale;
        capped_velocity.y *= scale;
    }
    const double elapsed_s = std::max(0.0, lost_duration_s);
    const double velocity_decay_time_s =
        std::max(1e-3, config_.motion_velocity_decay_time_s);
    const double velocity_decay =
        std::exp(-elapsed_s / velocity_decay_time_s);
    const double motion_reliability =
        reliable_velocity_weight * velocity_decay;
    // 对指数衰减速度积分，避免把最后一帧速度噪声按几十秒匀速外推到边界。
    const double effective_motion_time_s = velocity_decay_time_s *
        (1.0 - velocity_decay);
    result.motion_prediction_canonical = Point2d{
        last_canonical.x + capped_velocity.x * effective_motion_time_s,
        last_canonical.y + capped_velocity.y * effective_motion_time_s};
    result.motion_prediction_canonical.x = std::clamp(
        result.motion_prediction_canonical.x, 0.0, config_.field_length);
    result.motion_prediction_canonical.y = std::clamp(
        result.motion_prediction_canonical.y, 0.0, config_.field_width);

    const auto threshold_it = config_.motion_gate_mps.find(horizon);
    const double motion_threshold = threshold_it == config_.motion_gate_mps.end()
        ? 0.4 : threshold_it->second;
    const auto sigma_it = config_.motion_sigma_m.find(horizon);
    const double sigma = std::max(1e-3,
        sigma_it == config_.motion_sigma_m.end() ? 3.0 : sigma_it->second);
    result.motion_gated = raw_speed >= motion_threshold;

    // 2. 硬可达半径取“速度×真实失联时间+余量”和全局猜点上限的较小值。
    // 这与后面的高斯 distance_weight 不同：硬门外候选绝不会因概率高而复活。
    const double physical_maximum_distance =
        config_.max_speed_mps * elapsed_s + config_.reachability_margin_m;
    const double configured_maximum_distance = config_.max_guess_distance_m > 0.0
        ? config_.max_guess_distance_m
        : std::numeric_limits<double>::infinity();
    // 同时满足“按时间和速度可达”与“不可离最后观测点过远”。较小者生效。
    const double maximum_distance = std::min(
        physical_maximum_distance, configured_maximum_distance);
    const double distance_sigma = config_.distance_preference_sigma_m;

    // 3. 若该 role 有 NavGrid，使用 mesh 路径距离；否则才使用二维直线距离。
    // prepared_routes 来自 TargetCache，可让同一锚点的多帧预测复用 Dijkstra。
    NavigationRouteMap local_routes;
    const NavigationRouteMap* routes = nullptr;
    const bool mesh_requested =
        navigation_mesh_ && navigation_mesh_->enabled() &&
        navigation_mesh_->loaded() &&
        navigation_mesh_->supports_role(role);
    if (mesh_requested) {
        if (prepared_routes && prepared_routes->role == role) {
            routes = prepared_routes;
        } else {
            local_routes = navigation_mesh_->route_map(
                role, last_canonical, config_.mesh_snap_distance_m);
            routes = &local_routes;
        }
        if (!routes->valid) {
            result.rejection_reason = routes->reason;
            return result;
        }
        result.mesh_used = true;
    }

    // 4. 在不修改原始 model query 结果的副本上应用盲区偏置和驻留锚点。
    // 盲区先验与 navgrid 核心逻辑解耦：mesh_used=false（Dijkstra 关闭）时
    // 传入无效 route map，apply() 内部退回欧氏距离过滤，盲区先验仍生效。
    PriorDistribution effective_distribution = distribution;
    if (blind_zone_prior_ && blind_zone_prior_->loaded() && navigation_mesh_) {
        NavigationRouteMap empty_routes;  // valid=false → 欧氏回退
        const NavigationRouteMap& bias_routes =
            (result.mesh_used && routes) ? *routes : empty_routes;
        const auto bias = blind_zone_prior_->apply(
            role, last_canonical, result.motion_prediction_canonical,
            maximum_distance, *navigation_mesh_, bias_routes,
            effective_distribution);
        result.blind_zone_biased = bias.applied;
        result.blind_zone_probability_mass = bias.injected_probability_mass;
    }

    // 离线模型中的 stay_probability 表示车辆在当前区域保持不动的频率。
    // 旧逻辑虽已加载该字段，却没有参与门控，导致相机遮挡时远处热区压过
    // 最后可靠位置。显式注入驻留锚点后，速度似然仍可把结果沿 mesh 推进。
    double stay_anchor_mass = std::clamp(
        effective_distribution.stay_probability *
            config_.stay_anchor_probability_scale,
        0.0, 0.95);
    if (result.blind_zone_biased) {
        stay_anchor_mass = std::max(
            stay_anchor_mass,
            std::clamp(
                config_.blind_zone_minimum_stay_anchor_mass, 0.0, 0.95));
    }
    if (stay_anchor_mass > 0.0) {
        double existing_total = 0.0;
        for (const auto& candidate : effective_distribution.candidates) {
            existing_total += std::max(0.0, candidate.probability);
        }
        const double existing_scale = existing_total > 0.0
            ? (1.0 - stay_anchor_mass) / existing_total : 0.0;
        for (auto& candidate : effective_distribution.candidates) {
            candidate.probability =
                std::max(0.0, candidate.probability) * existing_scale;
        }

        const Point2d anchor = result.mesh_used
            ? routes->start_snap.canonical : last_canonical;
        const double merge_radius = result.mesh_used
            ? navigation_mesh_->resolution() * 0.5 : 1e-6;
        auto duplicate = std::find_if(
            effective_distribution.candidates.begin(),
            effective_distribution.candidates.end(),
            [&](const PriorCandidate& candidate) {
                return std::hypot(
                    candidate.canonical.x - anchor.x,
                    candidate.canonical.y - anchor.y) <= merge_radius;
            });
        if (duplicate != effective_distribution.candidates.end()) {
            duplicate->probability += stay_anchor_mass;
            duplicate->stay_anchor = true;
            duplicate->from_blind_zone =
                duplicate->from_blind_zone || result.blind_zone_biased;
        } else {
            PriorCandidate candidate;
            candidate.grid_index = result.mesh_used
                ? 2000000 + routes->start_cell : 2000000;
            candidate.canonical = anchor;
            candidate.probability = stay_anchor_mass;
            candidate.from_blind_zone = result.blind_zone_biased;
            candidate.stay_anchor = true;
            effective_distribution.candidates.push_back(candidate);
        }
        result.stay_anchor_probability_mass = stay_anchor_mass;
        effective_distribution.normalized_entropy =
            normalized_entropy(effective_distribution.candidates);
        effective_distribution.retained_probability_mass = 1.0;
    }
    result.normalized_entropy = effective_distribution.normalized_entropy;
    result.candidates.reserve(effective_distribution.candidates.size());
    double raw_fused_total = 0.0;

    // 5. 对每个候选计算 straight/mesh 两种距离、硬门状态与在线融合质量：
    // fused_raw = prior_probability * distance_weight * motion_weight。
    for (const auto& candidate : effective_distribution.candidates) {
        GatedCandidate gated;
        gated.prior = candidate;
        gated.straight_distance_from_last_m = std::hypot(
            candidate.canonical.x - last_canonical.x,
            candidate.canonical.y - last_canonical.y);
        gated.distance_from_last_m = gated.straight_distance_from_last_m;
        bool mesh_reachable = true;
        if (result.mesh_used) {
            gated.distance_from_last_m = navigation_mesh_->path_distance(
                *routes, candidate.canonical, config_.mesh_snap_distance_m);
            mesh_reachable = std::isfinite(gated.distance_from_last_m);
        }
        gated.blocked = blocked(candidate.canonical);
        gated.reachable = candidate.probability > 0.0 &&
            in_field(candidate.canonical) && !gated.blocked &&
            mesh_reachable &&
            gated.distance_from_last_m <= maximum_distance;
        if (gated.reachable) {
            result.reachable_probability_mass += candidate.probability;
            // 在硬阈值内继续保留连续偏好，避免高概率远点压过同样合理的近点。
            // 该权重只改变候选间相对排序，不会让阈值外候选重新变为可达。
            const double distance_weight = distance_sigma > 0.0
                ? std::exp(-gated.distance_from_last_m * gated.distance_from_last_m /
                    (2.0 * distance_sigma * distance_sigma))
                : 1.0;
            double motion_weight = 1.0;
            if (result.motion_gated) {
                const double dx = candidate.canonical.x - result.motion_prediction_canonical.x;
                const double dy = candidate.canonical.y - result.motion_prediction_canonical.y;
                motion_weight = std::exp(
                    -motion_reliability * (dx * dx + dy * dy) /
                    (2.0 * sigma * sigma));
            }
            gated.fused_probability =
                candidate.probability * motion_weight * distance_weight;
            raw_fused_total += gated.fused_probability;
        }
        result.candidates.push_back(gated);
    }

    if (raw_fused_total <= std::numeric_limits<double>::epsilon()) {
        result.rejection_reason = "no_reachable_candidate";
        return result;
    }
    // 归一化只发生在可达候选的 fused raw quality 之间；不可达候选保持 0，仍可
    // 作为诊断项输出。它不会改变 reachable_probability_mass 的先验质量含义。
    for (auto& candidate : result.candidates) {
        candidate.fused_probability /= raw_fused_total;
    }

    // 6. 选择主猜点。它不是无条件 candidates Top-1：
    // 盲区生效 -> 最佳可达盲区格；否则有运动 Gate -> 按 fused probability 加权；
    // 否则 -> 保持最后可靠位置。之后统一做 NavGrid snap。
    if (result.blind_zone_biased) {
        // 盲区多边形可能环绕高台。对候选坐标求均值有机会重新落到高台内，
        // 因此主猜点直接采用融合概率最高的可达盲区格，保持在配置的高度层。
        const auto best_blind = std::max_element(
            result.candidates.begin(), result.candidates.end(),
            [](const GatedCandidate& lhs, const GatedCandidate& rhs) {
                const double lhs_probability =
                    lhs.reachable && lhs.prior.from_blind_zone
                        ? lhs.fused_probability : -1.0;
                const double rhs_probability =
                    rhs.reachable && rhs.prior.from_blind_zone
                        ? rhs.fused_probability : -1.0;
                return lhs_probability < rhs_probability;
            });
        if (best_blind == result.candidates.end() ||
            !best_blind->reachable ||
            !best_blind->prior.from_blind_zone) {
            result.rejection_reason = "no_reachable_blind_zone_candidate";
            return result;
        }
        result.predicted_canonical = best_blind->prior.canonical;
    } else if (result.motion_gated) {
        for (const auto& candidate : result.candidates) {
            result.predicted_canonical.x +=
                candidate.fused_probability * candidate.prior.canonical.x;
            result.predicted_canonical.y +=
                candidate.fused_probability * candidate.prior.canonical.y;
        }
    } else {
        // 离线评测中的 motion gate：低速目标保持最后可靠位置。
        result.predicted_canonical = last_canonical;
    }

    if (result.mesh_used) {
        const auto predicted_snap = navigation_mesh_->snap_to_walkable(
            role, result.predicted_canonical,
            config_.mesh_prediction_snap_distance_m, routes->component_id);
        if (predicted_snap.valid) {
            result.predicted_canonical = predicted_snap.canonical;
        } else {
            // 融合均值无法安全落到 mesh 时，退回概率最高的可达候选。
            const auto best = std::max_element(
                result.candidates.begin(), result.candidates.end(),
                [](const GatedCandidate& lhs, const GatedCandidate& rhs) {
                    return lhs.fused_probability < rhs.fused_probability;
                });
            if (best == result.candidates.end() || !best->reachable) {
                result.rejection_reason = "prediction_not_walkable";
                return result;
            }
            NavigationSnap best_snap;
            navigation_mesh_->path_distance(
                *routes, best->prior.canonical,
                config_.mesh_snap_distance_m, &best_snap);
            result.predicted_canonical = best_snap.valid
                ? best_snap.canonical : best->prior.canonical;
        }
    }

    if (config_.max_guess_distance_m > 0.0) {
        const double prediction_distance = result.mesh_used
            ? navigation_mesh_->path_distance(
                *routes, result.predicted_canonical,
                config_.mesh_prediction_snap_distance_m)
            : std::hypot(
                result.predicted_canonical.x - last_canonical.x,
                result.predicted_canonical.y - last_canonical.y);
        if (!std::isfinite(prediction_distance) ||
            prediction_distance > config_.max_guess_distance_m + 1e-9) {
            result.rejection_reason = "prediction_beyond_distance_threshold";
            return result;
        }
    }

    // 7. output_top_k 在所有在线融合与排序完成之后截取，和 model query 阶段的
    // query_top_k 是两个不同层级的容量限制。
    std::sort(result.candidates.begin(), result.candidates.end(),
        [](const GatedCandidate& lhs, const GatedCandidate& rhs) {
            if (lhs.fused_probability != rhs.fused_probability) {
                return lhs.fused_probability > rhs.fused_probability;
            }
            return lhs.prior.grid_index < rhs.prior.grid_index;
        });
    if (config_.output_top_k > 0 && result.candidates.size() > config_.output_top_k) {
        result.candidates.resize(config_.output_top_k);
    }

    // 8. 计算整次 prior confidence。它衡量“本次猜点是否值得展示”，而不是某个
    // 坐标的 posterior probability；因此使用 tracking/sample/entropy/reachable
    // 四类可靠度乘积，而不是直接取 candidates[0].fused_probability。
    const double sample_reliability =
        effective_distribution.fallback_level == FallbackLevel::LOCAL_ZONE
            ? std::clamp(effective_distribution.local_weight, 0.0, 1.0)
            : 0.45;
    const double entropy_reliability =
        0.25 + 0.75 * (1.0 -
            std::clamp(effective_distribution.normalized_entropy, 0.0, 1.0));
    // 丢失时长只用于选择模型 horizon 和运动可达性，不再让置信度随时间
    // 线性归零。否则即使缓存保留，长时间丢失后也会被低置信度门控停猜。
    result.confidence =
        std::clamp(last_tracking_confidence, 0.0, 1.0) *
        sample_reliability * entropy_reliability *
        std::clamp(result.reachable_probability_mass, 0.0, 1.0);

    if (result.confidence < config_.minimum_confidence) {
        result.rejection_reason = "confidence_below_threshold";
        return result;
    }
    result.valid = true;
    return result;
}

}  // namespace position_prior
