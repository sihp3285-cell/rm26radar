#include "ConfigManager.hpp"

#include "ConfigManager.hpp"

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <algorithm>
#include <sstream>

namespace fs = std::filesystem;

namespace {

// 兼容 classNames:
// 1) sequence: ["car", "armor", ...]
// 2) map: {0: "car", 1: "armor", ...}
std::vector<std::string> parseClassNamesNode(const YAML::Node& node) {
    std::vector<std::string> result;
    if (!node) return result;

    if (node.IsSequence()) {
        result = node.as<std::vector<std::string>>();
        return result;
    }

    if (node.IsMap()) {
        std::vector<std::pair<int, std::string>> kvs;
        for (auto it = node.begin(); it != node.end(); ++it) {
            int idx = it->first.as<int>();
            std::string name = it->second.as<std::string>();
            kvs.emplace_back(idx, name);
        }

        std::sort(kvs.begin(), kvs.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });

        result.resize(kvs.empty() ? 0 : (kvs.back().first + 1));
        for (const auto& [idx, name] : kvs) {
            if (idx >= 0 && idx < static_cast<int>(result.size())) {
                result[idx] = name;
            }
        }
        return result;
    }

    throw std::runtime_error("classNames 格式错误，必须是 sequence 或 map");
}

} // namespace

Config::Config(const std::string& configDir) {
    fs::path dir(configDir);

    // 兼容两种传法：
    // 1) 传目录: /path/to/configs
    // 2) 传旧的单文件路径: /path/to/detectConfig.yaml
    if (dir.has_extension()) {
        dir = dir.parent_path();
    }

    const std::string modelYaml   = (dir / "model.yaml").string();
    const std::string cameraYaml  = (dir / "camera.yaml").string();
    const std::string mapYaml     = (dir / "map.yaml").string();
    const std::string trackerYaml = (dir / "tracker.yaml").string();
    const std::string runtimeYaml = (dir / "runtime.yaml").string();

    loadModelConfig(modelYaml);
    loadCameraConfig(cameraYaml);
    loadMapConfig(mapYaml);
    loadTrackerConfig(trackerYaml);
    loadRuntimeConfig(runtimeYaml);

    validateModelConfig(model);
    validateCameraConfig(camera);
    validateMapConfig(map);
    validateTrackerConfig(tracker);
}

Config::Config(const std::string& modelYaml,
               const std::string& cameraYaml,
               const std::string& mapYaml,
               const std::string& trackerYaml,
               const std::string& runtimeYaml) {
    loadModelConfig(modelYaml);
    loadCameraConfig(cameraYaml);
    loadMapConfig(mapYaml);
    loadTrackerConfig(trackerYaml);
    loadRuntimeConfig(runtimeYaml);

    validateModelConfig(model);
    validateCameraConfig(camera);
    validateMapConfig(map);
    validateTrackerConfig(tracker);
}

void Config::loadModelConfig(const std::string& path) {
    YAML::Node cfg = YAML::LoadFile(path);

    model.modelPath       = cfg["modelPath"].as<std::string>();
    model.imgSize1        = cfg["imgSize1"].as<int>();
    model.iouThreshold1   = cfg["iouThreshold1"].as<float>();
    model.scoreThreshold1 = cfg["scoreThreshold1"].as<float>();
    model.isNMS1          = cfg["isNMS1"].as<bool>();

    model.armorModelPath       = cfg["armorModelPath"].as<std::string>();
    model.imgSize2             = cfg["imgSize2"].as<int>();
    model.iouThreshold2        = cfg["iouThreshold2"].as<float>();
    model.scoreThreshold2      = cfg["scoreThreshold2"].as<float>();
    model.isNMS2               = cfg["isNMS2"].as<bool>();

    model.classifyModelPath       = cfg["classifyModelPath"].as<std::string>();
    model.imgSize3                = cfg["imgSize3"].as<int>();
    model.iouThreshold3           = cfg["iouThreshold3"].as<float>();
    model.scoreThreshold3         = cfg["scoreThreshold3"].as<float>();
    model.isNMS3                  = cfg["isNMS3"].as<bool>();

    model.minRoiSize  = cfg["minRoiSize"] ? cfg["minRoiSize"].as<int>() : 0;
    model.padRatio    = cfg["padRatio"] ? cfg["padRatio"].as<float>() : 0.0f;
    model.classIdxBase = cfg["classIdxBase"] ? cfg["classIdxBase"].as<int>() : 0;

    model.classNames = parseClassNamesNode(cfg["classNames"]);
}

void Config::loadCameraConfig(const std::string& path) {
    YAML::Node cfg = YAML::LoadFile(path);

    camera.cameraMatrix    = parseMat3x3(cfg["cameraMatrix"].as<std::vector<double>>());
    camera.distCoeffs      = parseRowMat(cfg["distCoeffs"].as<std::vector<double>>());
    camera.requirePointsNum = cfg["requirePointsNum"].as<int>();
    camera.worldPoints     = parsePoint3fList(cfg["worldPoints"].as<std::vector<std::vector<float>>>());
}

void Config::loadMapConfig(const std::string& path) {
    YAML::Node cfg = YAML::LoadFile(path);

    map.mapPath         = cfg["mapPath"].as<std::string>();
    map.mapPoints       = parsePoint2fList(cfg["MapPoints"].as<std::vector<std::vector<float>>>());
    map.world_points_2d = parsePoint2fList(cfg["world_points_2d"].as<std::vector<std::vector<float>>>());
    map.isFlip          = cfg["isflip"].as<bool>();
}

void Config::loadTrackerConfig(const std::string& path) {
    YAML::Node cfg = YAML::LoadFile(path);

    tracker.maxMissCount = cfg["maxMissCount"].as<int>();
    tracker.maxHistory   = cfg["maxhistory"].as<int>();
    tracker.distThreshold = cfg["distheshold"].as<float>();
}

void Config::loadRuntimeConfig(const std::string& path) {
    YAML::Node cfg = YAML::LoadFile(path);

    runtime.showFlag = cfg["show_flag"].as<bool>();
}

cv::Mat Config::parseMat3x3(const std::vector<double>& data) {
    if (data.size() != 9) {
        throw std::runtime_error("cameraMatrix 长度必须为 9");
    }

    cv::Mat mat(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i) {
        mat.at<double>(i / 3, i % 3) = data[i];
    }
    return mat;
}

cv::Mat Config::parseRowMat(const std::vector<double>& data) {
    if (data.empty()) {
        throw std::runtime_error("distCoeffs 不能为空");
    }

    cv::Mat mat(1, static_cast<int>(data.size()), CV_64F);
    for (int i = 0; i < static_cast<int>(data.size()); ++i) {
        mat.at<double>(0, i) = data[i];
    }
    return mat;
}

std::vector<cv::Point3f> Config::parsePoint3fList(const std::vector<std::vector<float>>& data) {
    std::vector<cv::Point3f> pts;
    pts.reserve(data.size());

    for (const auto& p : data) {
        if (p.size() != 3) {
            throw std::runtime_error("Point3f 列表中的每个点都必须有 3 个元素");
        }
        pts.emplace_back(p[0], p[1], p[2]);
    }
    return pts;
}

std::vector<cv::Point2f> Config::parsePoint2fList(const std::vector<std::vector<float>>& data) {
    std::vector<cv::Point2f> pts;
    pts.reserve(data.size());

    for (const auto& p : data) {
        if (p.size() != 2) {
            throw std::runtime_error("Point2f 列表中的每个点都必须有 2 个元素");
        }
        pts.emplace_back(p[0], p[1]);
    }
    return pts;
}

void Config::validateModelConfig(const ModelConfig& cfg) {
    if (cfg.modelPath.empty()) {
        throw std::runtime_error("detect modelPath 不能为空");
    }
    if (cfg.armorModelPath.empty()) {
        throw std::runtime_error("armorModelPath 不能为空");
    }
    if (cfg.classifyModelPath.empty()) {
        throw std::runtime_error("classifyModelPath 不能为空");
    }
    if (cfg.classNames.empty()) {
        throw std::runtime_error("classNames 不能为空");
    }
}

void Config::validateCameraConfig(const CameraConfig& cfg) {
    if (cfg.cameraMatrix.empty()) {
        throw std::runtime_error("cameraMatrix 不能为空");
    }
    if (cfg.distCoeffs.empty()) {
        throw std::runtime_error("distCoeffs 不能为空");
    }
    if (cfg.worldPoints.empty()) {
        throw std::runtime_error("worldPoints 不能为空");
    }
    if (cfg.requirePointsNum != static_cast<int>(cfg.worldPoints.size())) {
        throw std::runtime_error("requirePointsNum 与 worldPoints 数量不一致");
    }
}

void Config::validateMapConfig(const MapConfig& cfg) {
    if (cfg.mapPath.empty()) {
        throw std::runtime_error("mapPath 不能为空");
    }
    if (cfg.mapPoints.empty()) {
        throw std::runtime_error("MapPoints 不能为空");
    }
    if (cfg.world_points_2d.empty()) {
        throw std::runtime_error("world_points_2d 不能为空");
    }
    if (cfg.mapPoints.size() != cfg.world_points_2d.size()) {
        throw std::runtime_error("MapPoints 与 world_points_2d 数量不一致");
    }
}

void Config::validateTrackerConfig(const TrackerConfig& cfg) {
    if (cfg.maxMissCount < 0) {
        throw std::runtime_error("maxMissCount 不能小于 0");
    }
    if (cfg.maxHistory <= 0) {
        throw std::runtime_error("maxHistory 必须大于 0");
    }
    if (cfg.distThreshold <= 0.0f) {
        throw std::runtime_error("distThreshold 必须大于 0");
    }
}