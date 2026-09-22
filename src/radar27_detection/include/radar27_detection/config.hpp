#pragma once
#include <string>
#include <vector>
#include <stdexcept>
#include <opencv2/core.hpp>

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

    bool multiCarRecognition = true;
    int armorCanvasPadding = 2;
    int maxArmorRois = 4;

    std::vector<std::string> classNames;

    bool outpostEnabled = false;
    std::vector<int> outpostRoi;           // [x, y, width, height]
    float outpostScoreThreshold = 0.0f;
    float outpostMissTimeoutS = 1.0f;      // 前哨站连续未检测判定死亡时间（秒）
};
class DetectionConfig {
public:
 explicit DetectionConfig(const std::string& configDir, const std::string& roiPath = "");
ModelConfig model;
private:
void loadModelConfig(const std::string& path);
static void validateModelConfig(const ModelConfig& cfg);
};
