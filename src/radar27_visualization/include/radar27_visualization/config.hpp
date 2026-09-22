#pragma once
#include <string>
#include <vector>
#include <stdexcept>
#include <opencv2/core.hpp>

struct MapConfig {
    std::string mapPath;
    std::vector<float> race_size;  // [length, width] 场地物理尺寸，单位：米
    std::vector<int> map_size;     // [width, height] 地图像素尺寸
    bool isFlip = false;

    std::vector<int> outpostMapPointsRed;   // [x, y] 红方前哨站在地图上的像素坐标
    std::vector<int> outpostMapPointsBlue;  // [x, y] 蓝方前哨站在地图上的像素坐标

    /** 根据当前显示阵营选择红/蓝方前哨站像素坐标；返回对配置成员的只读引用。 */
    const std::vector<int>& getOutpostMapPoints(bool flipTeam) const {
        return flipTeam ? outpostMapPointsBlue : outpostMapPointsRed;
    }
};
class DisplayConfig {
public:
 explicit DisplayConfig(const std::string& configDir);
MapConfig map; std::vector<std::string> class_names;
private:
void loadMapConfig(const std::string& path);
static void validateMapConfig(const MapConfig& cfg);
};
