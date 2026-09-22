#include <radar27_visualization/config.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <algorithm>
#include <sstream>
namespace fs=std::filesystem;
DisplayConfig::DisplayConfig(const std::string& configDir) {
 fs::path dir(configDir);
loadMapConfig((dir/"map.yaml").string());
 auto n=YAML::LoadFile((dir/"map.yaml").string());
 class_names=n["class_names"].as<std::vector<std::string>>();
 map.outpostMapPointsRed=n["outpost_mappoints_red"].as<std::vector<int>>();
 map.outpostMapPointsBlue=n["outpost_mappoints_blue"].as<std::vector<int>>();
 if (!map.mapPath.empty() && fs::path(map.mapPath).is_relative()) map.mapPath=(dir/map.mapPath).lexically_normal().string();
validateMapConfig(map);
}
void DisplayConfig::loadMapConfig(const std::string& path) {
    YAML::Node cfg = YAML::LoadFile(path);

    map.mapPath         = cfg["mapPath"].as<std::string>();
    map.race_size       = cfg["race_size"].as<std::vector<float>>();
    map.map_size        = cfg["map_size"].as<std::vector<int>>();
    map.isFlip          = cfg["isflip"].as<bool>();
}
void DisplayConfig::validateMapConfig(const MapConfig& cfg) {
    if (cfg.mapPath.empty()) {
        throw std::runtime_error("mapPath 不能为空");
    }
    if (cfg.race_size.size() != 2) {
        throw std::runtime_error("race_size 必须包含 2 个元素 [length, width]");
    }
    if (cfg.map_size.size() != 2) {
        throw std::runtime_error("map_size 必须包含 2 个元素 [width, height]");
    }
    if (cfg.race_size[0] <= 0 || cfg.race_size[1] <= 0) {
        throw std::runtime_error("race_size 的元素必须大于 0");
    }
    if (cfg.map_size[0] <= 0 || cfg.map_size[1] <= 0) {
        throw std::runtime_error("map_size 的元素必须大于 0");
    }
}