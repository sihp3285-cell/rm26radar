#pragma once
#include <radar27_interfaces/msg/map_tactics.hpp>
#include <radar27_interfaces/msg/world_target.hpp>
#include <rm_field/robot_id.hpp>
#include <string>
#include <utility>
#include <vector>

// Snapshot analysis; no UI/prior dependency. Geometry is loaded once.
class MapAnalyzer {
public:
    explicit MapAnalyzer(int own_team = robot_id::BLUE) : own_team_(own_team) {}
    void loadRegions(const std::string& directory, double margin_m = 0.75,
                     double max_prediction_age_s = 1.0);
    // 重叠归属规则，格式 "优先区域>被覆盖区域"，按区域文件基名匹配（含对称实例）。
    // 目标落在两者重叠处时只计入优先区域。需在 loadRegions() 之前调用以便校验。
    void setOverlapPriority(const std::vector<std::string>& rules);
    void setFieldXFlip(bool toward_blue) { toward_blue_ = toward_blue; }
    void setTeamByFlip(bool red) { own_team_ = red ? robot_id::RED : robot_id::BLUE; }
    radar27_interfaces::msg::MapTactics evaluate(
        const std::vector<radar27_interfaces::msg::WorldTarget>& targets) const;
private:
    int own_team_;
    bool toward_blue_ = true;
    double margin_m_ = 0.75;
    double max_prediction_age_s_ = 1.0;
    std::vector<radar27_interfaces::msg::RegionOccupancy> regions_;
    // 与 regions_ 一一对应的区域文件基名，用于把优先级规则套到对称/多段实例上。
    std::vector<std::string> region_base_;
    std::vector<std::pair<std::string, std::string>> overlap_priority_;
};
