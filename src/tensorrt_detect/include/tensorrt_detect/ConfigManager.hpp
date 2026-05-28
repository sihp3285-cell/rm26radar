#pragma once

#include <string>
#include <vector>
#include <stdexcept>
#include <opencv2/opencv.hpp>
#include "core/bot_identity.hpp"

struct InferModelConfig {
    std::string modelPath;
    int imgSize = 0;
    float iouThreshold = 0.0f;
    float scoreThreshold = 0.0f;
    bool isNMS = false;
};

struct PipelineConfig {
    int minRoiSize = 0;
    float padRatio = 0.0f;
    int classIdxBase = 0;
    std::vector<std::string> classNames;
};

struct ModelConfig {
    std::string modelPath;
    int imgSize1 = 0;
    float iouThreshold1 = 0.0f;
    float scoreThreshold1 = 0.0f;
    bool isNMS1 = false;
    std::string modelType1 = "";
    

    std::string armorModelPath;
    int imgSize2 = 0;
    float iouThreshold2 = 0.0f;
    float scoreThreshold2 = 0.0f;
    bool isNMS2 = false;
    std::string modelType2 = "";

    std::string classifyModelPath;
    int imgSize3 = 0;
    float iouThreshold3 = 0.0f;
    float scoreThreshold3 = 0.0f;
    bool isNMS3 = false;
    std::string modelType3 = "";

    std::string airplaneModelPath;
    int imgSize4 = 0;
    float iouThreshold4 = 0.0f;
    float scoreThreshold4 = 0.0f;
    bool isNMS4 = false;
    std::string modelType4 = "";
    int airplaneIntervalMs = 33;

    int minRoiSize = 0;
    float padRatio = 0.0f;
    int classIdxBase = 0;

    std::vector<std::string> classNames;

    bool outpostEnabled = false;
    std::vector<int> outpostRoi;           // [x, y, width, height]
    float outpostScoreThreshold = 0.0f;
    int outpostMissThreshold = 20;         // 前哨站未检测判定死亡阈值（帧数）
};

struct CameraConfig {
    cv::Mat cameraMatrix;                  // 3x3, CV_64F
    cv::Mat distCoeffs;                    // 1xN, CV_64F
    int requirePointsNum = 0;
    std::vector<cv::Point3f> worldPoints;  // PnP 3D 点
    std::string meshPath;
};

struct MapConfig {
    std::string mapPath;
    std::vector<float> race_size;  // [length, width] 场地物理尺寸，单位：米
    std::vector<int> map_size;     // [width, height] 地图像素尺寸
    bool isFlip = false;

    std::vector<int> outpostMapPointsRed;   // [x, y] 红方前哨站在地图上的像素坐标
    std::vector<int> outpostMapPointsBlue;  // [x, y] 蓝方前哨站在地图上的像素坐标

    // 根据阵营获取对应的前哨站地图像素坐标
    const std::vector<int>& getOutpostMapPoints(bool flipTeam) const {
        return flipTeam ? outpostMapPointsBlue : outpostMapPointsRed;
    }
};

struct TrackerConfig {
    int maxMiss = 4;
    int maxPredict = 2;
    int minHit = 2;
    float maxGateBox = 200.0f;
    float classMismatchPenalty = 300.0f;
    int maxTracks = 20;
    BotIdentityConfig botIdentity;
};

struct RuntimeConfig {
    bool showFlag = true;
};

struct CalibConfig {
    std::vector<cv::Point2f> imagePoints;
    cv::Mat R;       // 3x3 CV_64F
    cv::Mat T;       // 3x1 CV_64F
    bool valid = false;
};

class Config {
public:
    // 传配置目录，例如: "/home/delphine/rm/tensorrt10_detect/configs"
    explicit Config(const std::string& configDir);

    // 或者直接传五个/六个文件路径
    Config(const std::string& modelYaml,
           const std::string& cameraYaml,
           const std::string& mapYaml,
           const std::string& runtimeYaml);

    ModelConfig model;
    CameraConfig camera;
    MapConfig map;
    TrackerConfig tracker;
    RuntimeConfig runtime;
    CalibConfig calib;

private:
    void loadModelConfig(const std::string& path);
    void loadCameraConfig(const std::string& path);
    void loadMapConfig(const std::string& path);
    void loadTrackerConfig(const std::string& path);
    void loadRuntimeConfig(const std::string& path);
    void loadCalibConfig(const std::string& path);

    static cv::Mat parseMat3x3(const std::vector<double>& data);
    static cv::Mat parseRowMat(const std::vector<double>& data);
    static std::vector<cv::Point3f> parsePoint3fList(const std::vector<std::vector<float>>& data);
    static std::vector<cv::Point2f> parsePoint2fList(const std::vector<std::vector<float>>& data);

    static void validateModelConfig(const ModelConfig& cfg);
    static void validateCameraConfig(const CameraConfig& cfg);
    static void validateMapConfig(const MapConfig& cfg);

};