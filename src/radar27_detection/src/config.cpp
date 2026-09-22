#include <radar27_detection/config.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <algorithm>
#include <sstream>
namespace fs=std::filesystem;
namespace {
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
}
DetectionConfig::DetectionConfig(const std::string& configDir, const std::string& roiPath) {
 fs::path dir(configDir);
loadModelConfig((dir/"model.yaml").string());
 const auto roi=roiPath.empty()?dir/"outpost_roi.yaml":fs::path(roiPath);
 if (fs::exists(roi)) {
  auto n=YAML::LoadFile(roi.string());
  if(n["outpost_enabled"]) model.outpostEnabled=n["outpost_enabled"].as<bool>();
  if(n["outpost_roi"]) model.outpostRoi=n["outpost_roi"].as<std::vector<int>>();
  if(n["outpost_score_threshold"]) model.outpostScoreThreshold=n["outpost_score_threshold"].as<float>();
  if(n["outpost_miss_timeout_s"]) model.outpostMissTimeoutS=n["outpost_miss_timeout_s"].as<float>();
 }
 if (!model.modelPath.empty() && fs::path(model.modelPath).is_relative()) model.modelPath=(dir/model.modelPath).lexically_normal().string();
if (!model.armorModelPath.empty() && fs::path(model.armorModelPath).is_relative()) model.armorModelPath=(dir/model.armorModelPath).lexically_normal().string();
if (!model.classifyModelPath.empty() && fs::path(model.classifyModelPath).is_relative()) model.classifyModelPath=(dir/model.classifyModelPath).lexically_normal().string();
if (!model.airplaneModelPath.empty() && fs::path(model.airplaneModelPath).is_relative()) model.airplaneModelPath=(dir/model.airplaneModelPath).lexically_normal().string();
validateModelConfig(model);

}
void DetectionConfig::loadModelConfig(const std::string& path) {
    YAML::Node cfg = YAML::LoadFile(path);

    model.modelPath       = cfg["modelPath"].as<std::string>();
    model.imgSize1        = cfg["imgSize1"].as<int>();
    model.iouThreshold1   = cfg["iouThreshold1"].as<float>();
    model.scoreThreshold1 = cfg["scoreThreshold1"].as<float>();
    model.isNMS1          = cfg["isNMS1"].as<bool>();
    model.modelType1      = cfg["modelType1"].as<std::string>();

    model.armorModelPath       = cfg["armorModelPath"].as<std::string>();
    model.imgSize2             = cfg["imgSize2"].as<int>();
    model.iouThreshold2        = cfg["iouThreshold2"].as<float>();
    model.scoreThreshold2      = cfg["scoreThreshold2"].as<float>();
    model.isNMS2               = cfg["isNMS2"].as<bool>();
    model.modelType2      = cfg["modelType2"].as<std::string>();

    model.classifyModelPath       = cfg["classifyModelPath"].as<std::string>();
    model.imgSize3                = cfg["imgSize3"].as<int>();
    model.iouThreshold3           = cfg["iouThreshold3"].as<float>();
    model.scoreThreshold3         = cfg["scoreThreshold3"].as<float>();
    model.isNMS3                  = cfg["isNMS3"].as<bool>();
    model.modelType3      = cfg["modelType3"].as<std::string>();

    model.airplaneModelPath = cfg["airplaneModelPath"] ? cfg["airplaneModelPath"].as<std::string>() : "";
    model.imgSize4          = cfg["imgSize4"] ? cfg["imgSize4"].as<int>() : 0;
    model.iouThreshold4     = cfg["iouThreshold4"] ? cfg["iouThreshold4"].as<float>() : 0.0f;
    model.scoreThreshold4   = cfg["scoreThreshold4"] ? cfg["scoreThreshold4"].as<float>() : 0.0f;
    model.isNMS4            = cfg["isNMS4"] ? cfg["isNMS4"].as<bool>() : false;
    model.modelType4        = cfg["modelType4"] ? cfg["modelType4"].as<std::string>() : "";
    model.airplaneIntervalMs = cfg["airplane_interval_ms"] ? cfg["airplane_interval_ms"].as<int>() : 33;

    model.minRoiSize  = cfg["minRoiSize"] ? cfg["minRoiSize"].as<int>() : 0;
    model.padRatio    = cfg["padRatio"] ? cfg["padRatio"].as<float>() : 0.0f;
    model.classIdxBase = cfg["classIdxBase"] ? cfg["classIdxBase"].as<int>() : 0;
    model.multiCarRecognition = cfg["multi_car_recognition"]
                                    ? cfg["multi_car_recognition"].as<bool>() : true;
    model.armorCanvasPadding = cfg["armor_canvas_padding"]
                                  ? std::max(0, cfg["armor_canvas_padding"].as<int>()) : 2;
    model.maxArmorRois = cfg["max_armor_rois"]
                            ? std::max(1, cfg["max_armor_rois"].as<int>()) : 4;

    model.classNames = parseClassNamesNode(cfg["classNames"]);
}
void DetectionConfig::validateModelConfig(const ModelConfig& cfg) {
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
    if (cfg.outpostMissTimeoutS < 0.0f) {
        throw std::runtime_error("outpost_miss_timeout_s 不能为负数");
    }
}