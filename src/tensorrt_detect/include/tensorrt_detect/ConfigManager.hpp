/**
 * @file ConfigManager.hpp
 * @brief 主感知包 YAML 配置的强类型表示、加载与启动期校验入口。
 *
 * Detect/Pose/Map 节点各自创建 Config，从 config_dir 读取 model、camera、map、
 * tracker、runtime 与 calibration。它不是动态参数服务器；除专用 reload service
 * 外，磁盘变化不会自动传播到已经构造的对象。
 */
#pragma once

#include <string>
#include <vector>
#include <stdexcept>
#include <opencv2/opencv.hpp>
#include "core/bot_identity.hpp"

/** 一个 TensorRT engine 及其输入尺寸/阈值；路径指向离线构建的 .engine。 */
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

/** DetectPipeline 的模型组合、类别表与特殊目标开关。 */
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

/** pixel->world 所需相机内参、畸变、场地 mesh 与已保存外参。 */
struct CameraConfig {
    cv::Mat cameraMatrix;                  // 3x3, CV_64F
    cv::Mat distCoeffs;                    // 1xN, CV_64F
    int requirePointsNum = 0;
    std::vector<cv::Point3f> worldPoints;  // PnP 3D 点
    std::string meshPath;
};

/** 场地尺寸、底图像素尺寸、方向和固定设施绘制坐标。 */
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

/** YAML 对应的 Tracker 参数；PoseNode 会显式复制为 TrackerParams。 */
struct TrackerConfig {
    // ========== Track 生命周期 ==========
    float maxLostTimeS = 0.30f;
    float maxPredictTimeS = 0.20f;
    float deadRetentionTimeS = 0.10f;
    float deadTargetHoldTimeS = 0.10f;
    int minHit = 2;
    int maxTracks = 20;

    // ========== 物理匹配 gate ==========
    float maxGateBox = 300.0f;
    float maxGateWorld = 2.5f;
    float kalmanGateBox = 18.467f;
    float kalmanGateWorld = 13.816f;
    float negativeGateBox = 200.0f;
    float negativeGateWorld = 1.0f;

    // ========== Hungarian 匹配代价 ==========
    float wBox = 1.0f;
    float wWorld = 1.0f;
    float classMismatchMinPenalty = 0.05f;
    float classMismatchPenalty = 0.40f;  // 最大类别不一致软惩罚

    // ========== BotIdentity 身份稳定器 ==========
    BotIdentityConfig botIdentity;

    // ========== 身份更新阈值 ==========
    float minIdentityUpdateConf = 0.20f;
    int identityConfirmObservations = 3;
    int identitySwitchConfirmObservations = 5;
    // 帧数确认的时间门（毫秒）：高 FPS 下保持确认所需物理时间稳定，<=0 关闭
    float identityConfirmMinTimeMs = 135.0f;
    float identitySwitchConfirmMinTimeMs = 540.0f;

    // ========== Official slot owner 机制 ==========
    float slotBindMinConf = 0.40f;
    float slotLeaseTimeS = 0.30f;
    float slotMinStability = 0.70f;
    float slotMaxSwitchRate = 0.35f;
    float maxSlotJumpDist = 2.5f;
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

/**
 * 一次性加载的配置聚合对象。构造失败以异常阻止节点带着半有效参数运行；公开成员
 * 供实时路径只读访问，DetectNode reload ROI 是当前少数显式修改点。
 */
class Config {
public:
    /** 从约定文件名组成的配置目录加载全部配置；任一必需项非法都会抛异常。 */
    explicit Config(const std::string& configDir);

    /** 从显式文件路径加载核心配置；主要供旧调用点或独立工具使用。 */
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
    /** 解析检测模型、阈值、类别表和特殊目标开关，并立即执行模型配置校验。 */
    void loadModelConfig(const std::string& path);
    /** 解析相机内参、畸变、PnP 世界点和场地 mesh 路径。 */
    void loadCameraConfig(const std::string& path);
    /** 解析场地物理尺寸、底图尺寸、方向和固定设施像素坐标。 */
    void loadMapConfig(const std::string& path);
    /** 解析关联、Kalman、身份稳定和槽位租约参数。 */
    void loadTrackerConfig(const std::string& path);
    /** 解析只影响运行展示行为的轻量开关。 */
    void loadRuntimeConfig(const std::string& path);
    /** 读取已保存外参；文件缺失或内容不完整时将 calib.valid 保持为 false。 */
    void loadCalibConfig(const std::string& path);

    /** 将九个行主序数值转换成独立持有数据的 3x3 CV_64F 矩阵。 */
    static cv::Mat parseMat3x3(const std::vector<double>& data);
    /** 将一维数值数组转换成 1xN CV_64F 行矩阵，主要用于畸变系数。 */
    static cv::Mat parseRowMat(const std::vector<double>& data);
    /** 将 YAML 二维数组转换成 PnP 使用的三维点列表，并检查每项维数。 */
    static std::vector<cv::Point3f> parsePoint3fList(const std::vector<std::vector<float>>& data);
    /** 将 YAML 二维数组转换成像素点列表，并检查每项维数。 */
    static std::vector<cv::Point2f> parsePoint2fList(const std::vector<std::vector<float>>& data);

    /** 对模型路径、输入尺寸、阈值和类别表做启动期 fail-fast 校验。 */
    static void validateModelConfig(const ModelConfig& cfg);
    /** 检查内参、畸变、标定点和 mesh 配置是否满足投影前置条件。 */
    static void validateCameraConfig(const CameraConfig& cfg);
    /** 检查场地/底图尺寸及比例配置，防止运行期除零或越界。 */
    static void validateMapConfig(const MapConfig& cfg);
    /** 检查 Tracker 的时间、门限、权重和身份参数是否处于有效范围。 */
    static void validateTrackerConfig(const TrackerConfig& cfg);

};