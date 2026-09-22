#include <radar27_localization/config.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <algorithm>
#include <sstream>
namespace fs=std::filesystem;
LocalizationConfig::LocalizationConfig(const std::string& configDir) {
 fs::path dir(configDir);
loadCameraConfig((dir/"camera.yaml").string());

if (!camera.meshPath.empty() && fs::path(camera.meshPath).is_relative()) camera.meshPath=(dir/camera.meshPath).lexically_normal().string();
validateCameraConfig(camera);
}
void LocalizationConfig::loadCameraConfig(const std::string& path) {
    YAML::Node cfg = YAML::LoadFile(path);

    camera.cameraMatrix    = parseMat3x3(cfg["cameraMatrix"].as<std::vector<double>>());
    camera.distCoeffs      = parseRowMat(cfg["distCoeffs"].as<std::vector<double>>());
    camera.requirePointsNum = cfg["requirePointsNum"].as<int>();
    camera.worldPoints     = parsePoint3fList(cfg["worldPoints"].as<std::vector<std::vector<float>>>());
    camera.meshPath        = cfg["meshPath"] ? cfg["meshPath"].as<std::string>() : "";
}
void LocalizationConfig::loadCalibConfig(const std::string& path) {
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
cv::Mat LocalizationConfig::parseMat3x3(const std::vector<double>& data) {
    if (data.size() != 9) {
        throw std::runtime_error("cameraMatrix 长度必须为 9");
    }

    cv::Mat mat(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i) {
        mat.at<double>(i / 3, i % 3) = data[i];
    }
    return mat;
}
cv::Mat LocalizationConfig::parseRowMat(const std::vector<double>& data) {
    if (data.empty()) {
        throw std::runtime_error("distCoeffs 不能为空");
    }

    cv::Mat mat(1, static_cast<int>(data.size()), CV_64F);
    for (int i = 0; i < static_cast<int>(data.size()); ++i) {
        mat.at<double>(0, i) = data[i];
    }
    return mat;
}
std::vector<cv::Point3f> LocalizationConfig::parsePoint3fList(const std::vector<std::vector<float>>& data) {
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
std::vector<cv::Point2f> LocalizationConfig::parsePoint2fList(const std::vector<std::vector<float>>& data) {
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
void LocalizationConfig::validateCameraConfig(const CameraConfig& cfg) {
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