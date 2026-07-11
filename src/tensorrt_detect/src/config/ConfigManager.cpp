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
    const std::string calibYaml   = (dir / "calib_result.yaml").string();

    loadModelConfig(modelYaml);
    loadCameraConfig(cameraYaml);
    loadMapConfig(mapYaml);
    loadRuntimeConfig(runtimeYaml);
    if (fs::exists(trackerYaml)) {
        try {
            loadTrackerConfig(trackerYaml);
        } catch (const std::exception& e) {
            // tracker.yaml 可选，加载失败不阻断
        }
    }
    if (fs::exists(calibYaml)) {
        try {
            loadCalibConfig(calibYaml);
        } catch (const std::exception& e) {
            // calib_result.yaml 可选，加载失败不阻断
        }
    }

    // outpost_roi.yaml 优先覆盖 model.yaml 中的 outpost 配置
    const std::string outpostYaml = (dir / "outpost_roi.yaml").string();
    if (fs::exists(outpostYaml)) {
        try {
            YAML::Node cfg = YAML::LoadFile(outpostYaml);
            if (cfg["outpost_enabled"]) {
                model.outpostEnabled = cfg["outpost_enabled"].as<bool>();
            }
            if (cfg["outpost_roi"]) {
                model.outpostRoi = cfg["outpost_roi"].as<std::vector<int>>();
            }
            if (cfg["outpost_score_threshold"]) {
                model.outpostScoreThreshold = cfg["outpost_score_threshold"].as<float>();
            }
            if (cfg["outpost_miss_threshold"]) {
                model.outpostMissThreshold = cfg["outpost_miss_threshold"].as<int>();
            }
            if (cfg["outpost_mappoints_red"]) {
                map.outpostMapPointsRed = cfg["outpost_mappoints_red"].as<std::vector<int>>();
            }
            if (cfg["outpost_mappoints_blue"]) {
                map.outpostMapPointsBlue = cfg["outpost_mappoints_blue"].as<std::vector<int>>();
            }
        } catch (const std::exception& e) {
            // outpost_roi.yaml 可选，加载失败不阻断
        }
    }

    validateModelConfig(model);
    validateCameraConfig(camera);
    validateMapConfig(map);
}

Config::Config(const std::string& modelYaml,
               const std::string& cameraYaml,
               const std::string& mapYaml,
               const std::string& runtimeYaml) {
    loadModelConfig(modelYaml);
    loadCameraConfig(cameraYaml);
    loadMapConfig(mapYaml);
    loadRuntimeConfig(runtimeYaml);

    validateModelConfig(model);
    validateCameraConfig(camera);
    validateMapConfig(map);
}

void Config::loadModelConfig(const std::string& path) {
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

    model.outpostEnabled = cfg["outpost_enabled"] ? cfg["outpost_enabled"].as<bool>() : false;
    if (cfg["outpost_roi"]) {
        model.outpostRoi = cfg["outpost_roi"].as<std::vector<int>>();
    }
    model.outpostScoreThreshold = cfg["outpost_score_threshold"]
                                      ? cfg["outpost_score_threshold"].as<float>()
                                      : 0.0f;
    model.outpostMissThreshold = cfg["outpost_miss_threshold"]
                                     ? cfg["outpost_miss_threshold"].as<int>()
                                     : 20;
}

void Config::loadCameraConfig(const std::string& path) {
    YAML::Node cfg = YAML::LoadFile(path);

    camera.cameraMatrix    = parseMat3x3(cfg["cameraMatrix"].as<std::vector<double>>());
    camera.distCoeffs      = parseRowMat(cfg["distCoeffs"].as<std::vector<double>>());
    camera.requirePointsNum = cfg["requirePointsNum"].as<int>();
    camera.worldPoints     = parsePoint3fList(cfg["worldPoints"].as<std::vector<std::vector<float>>>());
    camera.meshPath        = cfg["meshPath"] ? cfg["meshPath"].as<std::string>() : "";
}

void Config::loadMapConfig(const std::string& path) {
    YAML::Node cfg = YAML::LoadFile(path);

    map.mapPath         = cfg["mapPath"].as<std::string>();
    map.race_size       = cfg["race_size"].as<std::vector<float>>();
    map.map_size        = cfg["map_size"].as<std::vector<int>>();
    map.isFlip          = cfg["isflip"].as<bool>();
}

void Config::loadTrackerConfig(const std::string& path) {
    YAML::Node cfg = YAML::LoadFile(path);

    // ========== Track 生命周期 ==========
    tracker.maxMiss = cfg["max_miss"] ? cfg["max_miss"].as<int>() : 4;
    tracker.maxPredict = cfg["max_predict"] ? cfg["max_predict"].as<int>() : 2;
    tracker.minHit = cfg["min_hit"] ? cfg["min_hit"].as<int>() : 2;
    tracker.maxTracks = cfg["max_tracks"] ? cfg["max_tracks"].as<int>() : 20;

    // ========== 物理匹配 gate ==========
    tracker.maxGateBox = cfg["max_gate_box"] ? cfg["max_gate_box"].as<float>() : 300.0f;
    tracker.maxGateWorld = cfg["max_gate_world"] ? cfg["max_gate_world"].as<float>() : 2.5f;

    // ========== Hungarian 匹配代价 ==========
    tracker.wBox = cfg["w_box"] ? cfg["w_box"].as<float>() : 1.0f;
    tracker.wWorld = cfg["w_world"] ? cfg["w_world"].as<float>() : 1.0f;
    tracker.classMismatchPenalty = cfg["class_mismatch_penalty"] ? cfg["class_mismatch_penalty"].as<float>() : 0.25f;

    // ========== 身份更新阈值 ==========
    tracker.minIdentityUpdateConf = cfg["min_identity_update_conf"] ? cfg["min_identity_update_conf"].as<float>() : 0.20f;

    // ========== Official slot owner 机制 ==========
    tracker.slotBindMinConf = cfg["slot_bind_min_conf"] ? cfg["slot_bind_min_conf"].as<float>() : 0.40f;
    tracker.slotTakeoverMiss = cfg["slot_takeover_miss"] ? cfg["slot_takeover_miss"].as<int>() : 6;
    tracker.slotReleaseMiss = cfg["slot_release_miss"] ? cfg["slot_release_miss"].as<int>() : 8;
    tracker.maxSlotJumpDist = cfg["max_slot_jump_dist"] ? cfg["max_slot_jump_dist"].as<float>() : 2.5f;

    // ========== BotIdentity 身份稳定器 ==========
    if (cfg["bot_identity"]) {
        YAML::Node bi = cfg["bot_identity"];
        tracker.botIdentity.maxHistory = bi["max_history"] ? bi["max_history"].as<int>() : 50;
        tracker.botIdentity.purgeThreshold = bi["purge_threshold"] ? bi["purge_threshold"].as<int>() : 30;
        tracker.botIdentity.minHistoryForStable = bi["min_history_for_stable"] ? bi["min_history_for_stable"].as<int>() : 8;
        tracker.botIdentity.decay = bi["decay"] ? bi["decay"].as<float>() : 0.97f;
        tracker.botIdentity.numClasses = bi["num_classes"] ? bi["num_classes"].as<int>() : 9;
    }
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

void Config::loadCalibConfig(const std::string& path) {
    YAML::Node cfg = YAML::LoadFile(path);
    calib.imagePoints = parsePoint2fList(cfg["image_points"].as<std::vector<std::vector<float>>>());
    std::vector<double> r_data = cfg["r"].as<std::vector<double>>();
    if (r_data.size() != 9) {
        throw std::runtime_error("calib_result.yaml 中 r 必须包含 9 个元素");
    }
    calib.R = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i) {
        calib.R.at<double>(i / 3, i % 3) = r_data[i];
    }
    std::vector<double> t_data = cfg["t"].as<std::vector<double>>();
    if (t_data.size() != 3) {
        throw std::runtime_error("calib_result.yaml 中 t 必须包含 3 个元素");
    }
    calib.T = cv::Mat(3, 1, CV_64F);
    for (int i = 0; i < 3; ++i) {
        calib.T.at<double>(i, 0) = t_data[i];
    }
    calib.valid = true;
}
