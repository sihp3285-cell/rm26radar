/**
 * @file blind_zone_prior.hpp
 * @brief 在最后可靠观测靠近已知遮挡区时，为模型分布注入盲区与驻留候选。
 *
 * home/gully 对所有兵种生效，engineer 区只对工程生效。候选来自同一兵种 NavGrid
 * 的可通行 cell；本模块只重分配先验 probability，不负责最终运动可达 Gate。
 */
#pragma once

#include "position_prior/navigation_mesh.hpp"
#include "position_prior/prior_types.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace position_prior {

struct BlindZoneBiasConfig {
    double trigger_distance_m = 1.2;
    double maximum_probability_mass = 0.65;
    std::size_t candidates_per_zone = 4;
    double minimum_candidate_separation_m = 0.6;
};

struct BlindZoneBiasResult {
    bool applied = false;
    std::size_t active_zone_count = 0;
    std::size_t injected_candidate_count = 0;
    double injected_probability_mass = 0.0;
};

class BlindZonePrior {
public:
    /** 保存并规整盲区偏置参数；尚不加载任何区域文件。 */
    explicit BlindZonePrior(BlindZoneBiasConfig config = {});

    /** 依次加载公共区域与工程专用区域；任一解析失败都阻止节点带半配置运行。 */
    void load(
        const std::vector<std::string>& common_zone_paths,
        const std::string& engineer_zone_path);

    /** 返回区域文件是否已完成一次成功加载。 */
    bool loaded() const { return loaded_; }
    /** 返回展开镜像规则后保存在内存中的区域总数。 */
    std::size_t zone_count() const { return zones_.size(); }
    /** 返回当前只读偏置配置引用。 */
    const BlindZoneBiasConfig& config() const { return config_; }

    /**
     * 设置视角翻转。flip_team=false（敌方为红）时盲区直接按红方 canonical
     * 多边形比较；flip_team=true（敌方为蓝）时蓝方目标先经中心对称进入
     * canonical，物理盲区也随相机换到对面半场，因此 polygon 需按 x=14 轴
     * 左右翻转 (28-x, y) 后才与 canonical 位置匹配。
     */
    void set_flipped_view(bool flipped) { flipped_view_ = flipped; }
    /** 返回当前是否处于敌方为蓝的翻转视角。 */
    bool flipped_view() const { return flipped_view_; }

    /** 对靠近活动盲区的分布原地注入候选/驻留质量，并返回注入诊断统计。 */
    BlindZoneBiasResult apply(
        const std::string& role,
        const Point2d& last_canonical,
        const Point2d& motion_prediction_canonical,
        double maximum_path_distance_m,
        const NavigationMesh& navigation_mesh,
        const NavigationRouteMap& routes,
        PriorDistribution& distribution) const;

private:
    struct Zone {
        std::string name;
        bool engineer_only = false;
        bool mirror_centrally = false;
        bool prefer_lowest_elevation = false;
        double maximum_height_above_lowest_m = 0.0;
        bool same_elevation_as_last = false;
        double maximum_height_delta_from_last_m = 0.0;
        std::vector<Point2d> polygon;
        Point2d centroid;
    };

    /** 解析单个区域 YAML，并按文件/调用参数标记工程专用属性和镜像副本。 */
    void load_file(const std::string& path, bool engineer_only);
    /** 返回当前视角下生效的多边形（flip 视角按 x=14 轴左右翻转）。 */
    std::vector<Point2d> effective_polygon(const Zone& zone) const;
    /** 用奇偶射线法判断 canonical 点是否位于多边形内部。 */
    static bool point_in_polygon(
        const Point2d& point,
        const std::vector<Point2d>& polygon);
    /** 返回点到多边形边界的最小欧氏距离；内部点距离为零。 */
    static double distance_to_polygon(
        const Point2d& point,
        const std::vector<Point2d>& polygon);
    /** 根据候选 probability 计算 [0,1] 归一化熵，用于偏置前后诊断。 */
    static double normalized_entropy(
        const std::vector<PriorCandidate>& candidates);

    BlindZoneBiasConfig config_;
    bool loaded_ = false;
    bool flipped_view_ = false;
    std::vector<Zone> zones_;
};

}  // namespace position_prior
