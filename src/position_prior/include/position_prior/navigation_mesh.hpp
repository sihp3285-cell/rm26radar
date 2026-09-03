/**
 * @file navigation_mesh.hpp
 * @brief 在线加载离线生成的 NavGrid，并提供吸附与网格最短路查询。
 *
 * 运行时不解析 PLY。JSON/YAML 网格通常以 0.2 m cell 保存 height_mm，以及每个兵种
 * profile 的 walkable、component_id、maximum_step_height。route_map() 从最后可靠
 * 点运行一次 Dijkstra，随后多个候选复用距离表，避免逐候选重复寻路。
 */
#pragma once

#include "position_prior/prior_types.hpp"

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

namespace position_prior {

/** 任意 canonical 点吸附到可通行 cell 后的结果。 */
struct NavigationSnap {
    bool valid = false;
    int cell_index = -1;
    int component_id = -1;
    Point2d canonical;
    double snap_distance_m = 0.0;
};

/** 一个起点/兵种对应的 Dijkstra 全场距离表；distances_m 以 cell index 索引。 */
struct NavigationRouteMap {
    bool valid = false;
    std::string reason;
    std::string role;
    int start_cell = -1;
    int component_id = -1;
    NavigationSnap start_snap;
    std::vector<double> distances_m;
};

/** PositionPriorNode 启动时加载、PriorGate 查询的只读在线 NavGrid。 */
class NavigationMesh {
public:
    /** 解析离线 NavGrid 元数据、高度和各兵种 profile；校验失败抛异常。 */
    void load(const std::string& path);

    /** 返回 NavGrid 是否已成功完整加载。 */
    bool loaded() const { return loaded_; }
    /** 返回当前 NavGrid 来源路径，便于日志和模型一致性诊断。 */
    const std::string& path() const { return path_; }
    /** 返回网格覆盖的场地长度，单位米。 */
    double field_length() const { return field_length_; }
    /** 返回网格覆盖的场地宽度，单位米。 */
    double field_width() const { return field_width_; }
    /** 返回单个方格边长，单位米。 */
    double resolution() const { return resolution_; }
    /** 返回网格行数。 */
    int rows() const { return rows_; }
    /** 返回网格列数。 */
    int columns() const { return columns_; }

    /** 判断 NavGrid 是否包含指定兵种的 walkable/component profile。 */
    bool supports_role(const std::string& role) const;
    /** 在限定半径和可选连通分量内，将 canonical 点吸附到最近可通行 cell。 */
    NavigationSnap snap_to_walkable(
        const std::string& role,
        const Point2d& canonical,
        double maximum_snap_distance_m,
        int required_component_id = -1) const;
    /**
     * 将起点 snap 到可行网格并以 8-neighbor 运行 Dijkstra。实现会检查高度差，
     * 对角移动还要求相邻正交边可走，避免从障碍角点“穿角”。
     */
    NavigationRouteMap route_map(
        const std::string& role,
        const Point2d& canonical_start,
        double maximum_snap_distance_m) const;
    /** 复用已计算 route map 查询到目标的路径距离；不可达返回 infinity。 */
    double path_distance(
        const NavigationRouteMap& routes,
        const Point2d& canonical_goal,
        double maximum_snap_distance_m,
        NavigationSnap* goal_snap = nullptr) const;

    /** 将扁平 cell index 转成 canonical 米制格心；索引无效返回非有限哨兵点。 */
    Point2d cell_center(int cell_index) const;
    /** 返回 cell 的离线高度（米）；索引无效返回非有限值。 */
    double cell_height_m(int cell_index) const;
    /** 枚举多边形内、满足兵种与可选连通分量约束的可通行 cell。 */
    std::vector<int> walkable_cells_in_polygon(
        const std::string& role,
        const std::vector<Point2d>& polygon,
        int required_component_id = -1) const;

private:
    struct Profile {
        double maximum_step_height_m = 0.0; // 该兵种单边允许的最大高度变化。
        std::vector<std::uint8_t> walkable; // 每格是否可供该兵种通行。
        std::vector<int> component_id; // 离线连通分量；不同分量无需尝试寻路。
    };

    /** 判断二维行列是否落在当前 NavGrid 范围。 */
    bool valid_cell(int row, int column) const;
    /** 检查相邻边两端可通行、高差允许，且对角移动不会穿越障碍角。 */
    bool edge_is_walkable(
        const Profile& profile,
        int row,
        int column,
        int next_row,
        int next_column) const;
    /** 用奇偶射线法判断 canonical 格心是否位于区域多边形。 */
    static bool point_in_polygon(
        const Point2d& point,
        const std::vector<Point2d>& polygon);

    bool loaded_ = false;
    std::string path_;
    double field_length_ = 0.0;
    double field_width_ = 0.0;
    double resolution_ = 0.0;
    int rows_ = 0;
    int columns_ = 0;
    std::vector<int> height_mm_;
    std::unordered_map<std::string, Profile> profiles_;
};

}  // namespace position_prior
