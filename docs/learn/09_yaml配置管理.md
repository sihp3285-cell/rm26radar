# 09 YAML 配置管理

> 本项目使用 YAML 作为统一的配置格式，涵盖模型路径、相机参数、地图设置、跟踪器参数等。`yaml-cpp` 库负责解析，ConfigManager 封装了类型安全的读取接口和参数验证逻辑。本章讲解 yaml-cpp 的使用和本项目的配置系统设计。

---

## 9.1 yaml-cpp 基础

### 9.1.1 读取标量

```cpp
#include <yaml-cpp/yaml.h>

YAML::Node config = YAML::LoadFile("config.yaml");

std::string modelPath = config["modelPath"].as<std::string>();
int imgSize = config["imgSize"].as<int>();
float threshold = config["scoreThreshold"].as<float>();
bool enabled = config["outpost_enabled"].as<bool>();
```

### 9.1.2 读取数组

```cpp
// 一维数组
std::vector<int> roi = config["outpost_roi"].as<std::vector<int>>();
// roi = [1807, 1451, 173, 65]

// 二维数组
auto worldPoints = config["worldPoints"].as<std::vector<std::vector<float>>>();
// worldPoints = [[-4.379, 0.953, 5.710], [-3.857, 1.748, -2.907], ...]
```

### 9.1.3 安全读取（带默认值）

```cpp
// 如果键不存在，使用默认值
cfg_->model.outpostEnabled = node["outpost_enabled"]
    ? node["outpost_enabled"].as<bool>()
    : false;
```

---

## 9.2 本项目的配置结构

### 9.2.1 配置文件清单

```
configs/
├── model.yaml          # 模型路径、输入尺寸、阈值、类别名
├── camera.yaml         # 相机内参、畸变系数、3D 标定点、mesh 路径
├── map.yaml            # 地图图片路径、场地尺寸、像素尺寸
├── runtime.yaml        # 运行时标志（如是否显示调试窗口）
├── calib_result.yaml   # 标定结果（R 矩阵、T 向量、图像点）
└── outpost_roi.yaml    # 前哨站 ROI 区域
```

### 9.2.2 model.yaml 详解

```yaml
modelPath: "/home/delphine/rm/tensorrt10_detect/models/robot_only.engine"
imgSize1: 1280
iouThreshold1: 0.3
scoreThreshold1: 0.3
isNMS1: false
modelType1: "DETECT"

# 装甲板检测模型
armorModelPath: "/home/delphine/rm/tensorrt10_detect/models/newarmor.engine"
imgSize2: 192
# ...

# 分类模型
classifyModelPath: "/home/delphine/rm/tensorrt10_detect/models/classify_hku.engine"
imgSize3: 64
modelType3: "CLASSIFY"

# 无人机检测模型
airplaneModelPath: "/home/delphine/rm/tensorrt10_detect/models/airplane640.engine"
imgSize4: 640
airplane_interval_ms: 200

# 前哨站 ROI
outpost_enabled: true
outpost_roi: [1807, 1451, 173, 65]
outpost_score_threshold: 0.4
outpost_miss_threshold: 20

# 类别映射
classNames:
  0: "car"
  1: "armor"
  2: "1"
  3: "2"
  4: "3"
  5: "4"
  6: "s"
  7: "outpost"
  8: "airplane"

minRoiSize: 30
padRatio: 0.25
classIdxBase: 2
```

### 9.2.3 camera.yaml 详解

```yaml
cameraMatrix: [5033.780199, 0.000000, 2829.234535,
               0.000000, 5036.139955, 1929.489557,
               0.000000, 0.000000, 1.000000]

distCoeffs: [-0.061883, 0.104794, 0.000434, -0.000036, 0.000000]

requirePointsNum: 6
worldPoints:
  - [-4.379, 0.953, 5.710] 
  - [-3.857, 1.748, -2.907] 
  - [-3.55, 0.735, -10.334] 
  - [-0.344, 2.598, 0.35] 
  - [2.299, 0.273, -10.975] 
  - [4.21, 0.403, 3.531] 

meshPath: "/home/delphine/rm/tensorrt10_detect/configs/RMUC2025_National.PLY"
```

### 9.2.4 map.yaml 详解

```yaml
mapPath: "/home/delphine/rm/tensorrt10_detect/configs/map2.png"
race_size: [28, 15]      # 真实场地尺寸（米）：长 Z，宽 X
map_size: [722, 388]     # 地图图片像素分辨率
isflip: true             # 是否翻转坐标系
```

---

## 9.3 ConfigManager 设计

### 9.3.1 配置结构体定义

```cpp
// ConfigManager.hpp

struct ModelConfig {
    std::string modelPath;
    int imgSize1 = 0;
    float scoreThreshold1 = 0.0f;
    bool isNMS1 = false;
    std::string modelType1 = "";
    
    std::string armorModelPath;
    // ... 共 4 组模型配置
    
    int minRoiSize = 0;
    float padRatio = 0.0f;
    std::vector<std::string> classNames;
    
    bool outpostEnabled = false;
    std::vector<int> outpostRoi;
};

struct CameraConfig {
    cv::Mat cameraMatrix;        // 3x3 CV_64F
    cv::Mat distCoeffs;          // 1xN CV_64F
    int requirePointsNum = 0;
    std::vector<cv::Point3f> worldPoints;
    std::string meshPath;
};

struct MapConfig {
    std::string mapPath;
    std::vector<float> race_size;
    std::vector<int> map_size;
    bool isFlip = false;
};

struct TrackerConfig {
    int maxMiss = 4;
    int maxPredict = 2;
    int minHit = 2;
    float maxGateBox = 200.0f;
};
```

### 9.3.2 Config 类接口

```cpp
class Config {
public:
    // 从配置目录加载全部配置
    explicit Config(const std::string& configDir);
    
    // 或直接传文件路径
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
    // ...
    
    static cv::Mat parseMat3x3(const std::vector<double>& data);
    static std::vector<cv::Point3f> parsePoint3fList(
        const std::vector<std::vector<float>>& data);
    
    static void validateModelConfig(const ModelConfig& cfg);
    static void validateCameraConfig(const CameraConfig& cfg);
};
```

### 9.3.3 特殊类型解析

```cpp
cv::Mat Config::parseMat3x3(const std::vector<double>& data) {
    cv::Mat mat(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i) {
        mat.at<double>(i / 3, i % 3) = data[i];
    }
    return mat;
}

std::vector<cv::Point3f> Config::parsePoint3fList(
    const std::vector<std::vector<float>>& data) 
{
    std::vector<cv::Point3f> result;
    for (const auto& p : data) {
        if (p.size() != 3) throw std::runtime_error("Point3f must have 3 elements");
        result.emplace_back(p[0], p[1], p[2]);
    }
    return result;
}
```

### 9.3.4 参数验证

```cpp
void Config::validateModelConfig(const ModelConfig& cfg) {
    if (cfg.modelPath.empty()) {
        throw std::invalid_argument("modelPath cannot be empty");
    }
    if (cfg.imgSize1 <= 0) {
        throw std::invalid_argument("imgSize1 must be positive");
    }
    // ...
}
```

---

## 9.4 动态配置重载

### 9.4.1 detect_node 的 ROI 重载

```cpp
void reloadROI(const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
               std_srvs::srv::Trigger::Response::SharedPtr response)
{
    try {
        std::string outpost_yaml = (dir / "outpost_roi.yaml").string();
        YAML::Node cfg = YAML::LoadFile(outpost_yaml);
        
        cfg_->model.outpostEnabled = cfg["outpost_enabled"]
            ? cfg["outpost_enabled"].as<bool>() : false;
        if (cfg["outpost_roi"]) {
            cfg_->model.outpostRoi = cfg["outpost_roi"].as<std::vector<int>>();
        }
        cfg_->model.outpostScoreThreshold = cfg["outpost_score_threshold"]
            ? cfg["outpost_score_threshold"].as<float>() : 0.0f;
        
        response->success = true;
        response->message = "outpost ROI 配置已重载";
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("重载失败: ") + e.what();
    }
}
```

### 9.4.2 pose_node 的标定重载

```cpp
void reloadCalibration(...) {
    YAML::Node node = YAML::LoadFile(calibPath);
    std::vector<double> r_data = node["r"].as<std::vector<double>>();
    std::vector<double> t_data = node["t"].as<std::vector<double>>();
    
    cv::Mat R(3, 3, CV_64F);
    cv::Mat T(3, 1, CV_64F);
    for (int i = 0; i < 9; ++i) R.at<double>(i / 3, i % 3) = r_data[i];
    for (int i = 0; i < 3; ++i) T.at<double>(i, 0) = t_data[i];
    
    pose_solver_->setExtrinsic(R, T);
    is_calibrated_ = true;
}
```

---

## 9.5 YAML 自定义类型转换

本项目为 `cv::Point2f` 注册了 YAML 自定义转换：

```cpp
namespace YAML {
    template<>
    struct convert<cv::Point2f> {
        static Node encode(const cv::Point2f& rhs) {
            Node node;
            node.push_back(rhs.x);
            node.push_back(rhs.y);
            return node;
        }
        
        static bool decode(const Node& node, cv::Point2f& rhs) {
            if (!node.IsSequence() || node.size() != 2) return false;
            rhs.x = node[0].as<float>();
            rhs.y = node[1].as<float>();
            return true;
        }
    };
}
```

---

## 9.6 本章小结

| 技术点 | 应用 | 关键 API |
|:---|:---|:---|
| yaml-cpp | 配置文件解析 | `YAML::LoadFile()`, `node.as<T>()` |
| 结构体封装 | 类型安全的配置访问 | `ModelConfig`, `CameraConfig`... |
| 特殊类型解析 | cv::Mat, cv::Point3f | 静态工厂方法 |
| 参数验证 | 启动时检查配置合法性 | `validateModelConfig()` |
| 动态重载 | 运行时不重启更新配置 | Service + `YAML::LoadFile()` |
| 自定义转换 | cv::Point2f 序列化 | `YAML::convert<T>` 特化 |
