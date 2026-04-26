# `ConfigManager.hpp` + `ConfigManager.cpp` 逐行讲解

这两份文件是项目的**配置管理中心**，负责从 YAML 文件读取模型参数、相机参数、地图参数等，并转换成 C++ 结构体供其他模块使用。

---

# 第一部分：`ConfigManager.hpp`

## 一、引入的头文件

```cpp
#pragma once

#include <string>
#include <vector>
#include <stdexcept>
#include <opencv2/opencv.hpp>
```

---

### `<stdexcept>`

标准异常基类。`ConfigManager.cpp` 中的验证函数会抛出 `std::runtime_error`。

---

## 二、配置结构体

### `InferModelConfig`

```cpp
struct InferModelConfig {
    std::string modelPath;
    int imgSize = 0;
    float iouThreshold = 0.0f;
    float scoreThreshold = 0.0f;
    bool isNMS = false;
};
```

单个推理模型的配置。但注意：`ModelConfig` 才是实际使用的结构体，`InferModelConfig` 似乎是一个早期设计或备用结构。

---

### `PipelineConfig`

```cpp
struct PipelineConfig {
    int minRoiSize = 0;
    float padRatio = 0.0f;
    int classIdxBase = 0;
    std::vector<std::string> classNames;
};
```

流水线配置。同样，`ModelConfig` 已经包含了这些字段。

---

### `ModelConfig`

```cpp
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

    int minRoiSize = 0;
    float padRatio = 0.0f;
    int classIdxBase = 0;

    std::vector<std::string> classNames;
};
```

---

#### 三个模型的配置

| 后缀 | 模型 | 用途 |
|------|------|------|
| `1` | `detectModel_` | 目标检测 |
| `2` | `armorDetector_` | 装甲板检测 |
| `3` | `classifyModel_` | 分类 |

每个模型都有：路径、输入尺寸、IoU 阈值、置信度阈值、NMS 开关、模型类型字符串。

---

#### 公共配置

* `minRoiSize`：ROI 最小尺寸，小于此值的 ROI 被丢弃
* `padRatio`：填充比例（当前未使用）
* `classIdxBase`：类别索引偏移基值（当前未使用）
* `classNames`：类别名称表，用于可视化

---

### `CameraConfig`

```cpp
struct CameraConfig {
    cv::Mat cameraMatrix;                  // 3x3, CV_64F
    cv::Mat distCoeffs;                    // 1xN, CV_64F
    int requirePointsNum = 0;
    std::vector<cv::Point3f> worldPoints;  // PnP 3D 点
    std::string meshPath;
};
```

---

#### `cameraMatrix`

3×3 内参矩阵 `K`，`CV_64F`（double 类型）。

---

#### `distCoeffs`

畸变系数向量，1×N，`CV_64F`。

---

#### `requirePointsNum`

PnP 标定需要的点数。通常等于 `worldPoints.size()`。

---

#### `worldPoints`

用于 PnP 的 3D 世界坐标点。例如场地标定板的 4 个角点。

---

#### `meshPath`

场地 3D Mesh 文件路径。供 `Raycaster` 加载。

---

### `MapConfig`

```cpp
struct MapConfig {
    std::string mapPath;
    std::vector<float> race_size;  // [length, width] 场地物理尺寸，单位：米
    std::vector<int> map_size;     // [width, height] 地图像素尺寸
    bool isFlip = false;
};
```

小地图配置。

---

### `RuntimeConfig`

```cpp
struct RuntimeConfig {
    bool showFlag = true;
};
```

运行时配置。目前只有一个 `showFlag`，控制是否显示可视化窗口。

---

### `CalibConfig`

```cpp
struct CalibConfig {
    std::vector<cv::Point2f> imagePoints;
    cv::Mat R;       // 3x3 CV_64F
    cv::Mat T;       // 3x1 CV_64F
    bool valid = false;
};
```

标定结果配置。

---

#### `valid`

关键字段。`true` 表示标定结果有效，可以被 `pose_node` 直接使用；`false` 表示无效，需要 fallback 到文件加载。

---

## 三、`Config` 类

```cpp
class Config {
public:
    explicit Config(const std::string& configDir);
    Config(const std::string& modelYaml,
           const std::string& cameraYaml,
           const std::string& mapYaml,
           const std::string& runtimeYaml);

    ModelConfig model;
    CameraConfig camera;
    MapConfig map;
    RuntimeConfig runtime;
    CalibConfig calib;
```

---

### 两个构造函数

1. **目录版本**：传入配置目录，自动拼接五个 YAML 文件路径
2. **文件版本**：直接传入四个 YAML 文件路径

---

### `explicit`

```cpp
explicit Config(const std::string& configDir);
```

`explicit` 防止隐式类型转换。例如：

```cpp
Config cfg = "/path/to/configs";  // 如果不用 explicit，这行能编译（不好）
```

用 `explicit` 后，必须显式构造：

```cpp
Config cfg("/path/to/configs");  // 正确
```

---

### 公共数据成员

```cpp
    ModelConfig model;
    CameraConfig camera;
    MapConfig map;
    RuntimeConfig runtime;
    CalibConfig calib;
```

Config 类本质上是一个**数据聚合体（POD-like class）**，没有复杂逻辑，只是把配置数据组织在一起。

这种设计的好处：

* 访问方便：`cfg.model.modelPath`、`cfg.camera.cameraMatrix`
* 一目了然：所有配置字段都有明确分类

---

### 私有方法

```cpp
private:
    void loadModelConfig(const std::string& path);
    void loadCameraConfig(const std::string& path);
    void loadMapConfig(const std::string& path);
    void loadRuntimeConfig(const std::string& path);
    void loadCalibConfig(const std::string& path);

    static cv::Mat parseMat3x3(const std::vector<double>& data);
    static cv::Mat parseRowMat(const std::vector<double>& data);
    static std::vector<cv::Point3f> parsePoint3fList(const std::vector<std::vector<float>>& data);
    static std::vector<cv::Point2f> parsePoint2fList(const std::vector<std::vector<float>>& data);

    static void validateModelConfig(const ModelConfig& cfg);
    static void validateCameraConfig(const CameraConfig& cfg);
    static void validateMapConfig(const MapConfig& cfg);
```

---

#### 加载方法

每个 `loadXxxConfig` 负责读取一个 YAML 文件，解析后填充对应的结构体。

---

#### 解析方法

`static` 工具函数，把 YAML 中的原始数据（如 `std::vector<double>`）转换成 OpenCV 的 `cv::Mat` 或 `std::vector<cv::PointXf>`。

用 `static` 是因为它们不依赖 `Config` 实例的状态，只是纯数据转换函数。

---

#### 验证方法

`static` 工具函数，检查配置是否合法。不合法时抛出 `std::runtime_error`。

---

# 第二部分：`ConfigManager.cpp`

## 一、匿名命名空间

```cpp
namespace {
    std::vector<std::string> parseClassNamesNode(const YAML::Node& node) { ... }
}
```

---

### 匿名命名空间的作用

```cpp
namespace { ... }
```

匿名命名空间中的符号具有**内部链接性**，只在当前 `.cpp` 文件中可见。这相当于 C 语言中的 `static` 全局函数，用于封装文件私有的辅助函数。

---

### `parseClassNamesNode`

```cpp
std::vector<std::string> parseClassNamesNode(const YAML::Node& node) {
    std::vector<std::string> result;
    if (!node) return result;
```

如果 YAML 节点为空，返回空向量。

---

#### 兼容 Sequence 格式

```cpp
    if (node.IsSequence()) {
        result = node.as<std::vector<std::string>>();
        return result;
    }
```

YAML 格式一：

```yaml
classNames:
  - car
  - armor
  - R1
  - R2
```

直接转成 `std::vector<std::string>`。

---

#### 兼容 Map 格式

```cpp
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
```

YAML 格式二：

```yaml
classNames:
  0: car
  1: armor
  2: R1
  3: R2
```

处理步骤：

1. 遍历 map，收集 `(idx, name)` 对
2. 按 `idx` 排序
3. 根据最大索引分配数组空间
4. 按索引填入名称

---

#### 格式错误

```cpp
    throw std::runtime_error("classNames 格式错误，必须是 sequence 或 map");
```

如果既不是 sequence 也不是 map，抛异常。

---

## 二、目录版本构造函数

```cpp
Config::Config(const std::string& configDir) {
    fs::path dir(configDir);

    if (dir.has_extension()) {
        dir = dir.parent_path();
    }
```

---

### 兼容两种传法

1. 传目录：`/path/to/configs`
2. 传文件：`/path/to/configs/model.yaml` → 自动取父目录

`has_extension()` 判断路径是否有扩展名（如 `.yaml`），有就认为是文件路径。

---

### 自动拼接文件路径

```cpp
    const std::string modelYaml   = (dir / "model.yaml").string();
    const std::string cameraYaml  = (dir / "camera.yaml").string();
    const std::string mapYaml     = (dir / "map.yaml").string();
    const std::string trackerYaml = (dir / "tracker.yaml").string();
    const std::string runtimeYaml = (dir / "runtime.yaml").string();
    const std::string calibYaml   = (dir / "calib_result.yaml").string();
```

使用 C++17 `std::filesystem::path` 的 `/` 运算符拼接路径，跨平台兼容（Windows 用 `\`，Linux 用 `/`）。

---

### 加载配置

```cpp
    loadModelConfig(modelYaml);
    loadCameraConfig(cameraYaml);
    loadMapConfig(mapYaml);
    loadRuntimeConfig(runtimeYaml);
```

前四个配置文件是**必需的**。

---

### 可选加载标定结果

```cpp
    if (fs::exists(calibYaml)) {
        try {
            loadCalibConfig(calibYaml);
        } catch (const std::exception& e) {
            // calib_result.yaml 可选，加载失败不阻断
        }
    }
```

`calib_result.yaml` 是**可选的**。如果存在就加载，加载失败也不抛异常（catch 住），因为标定结果可以在运行时重新生成。

---

### 验证

```cpp
    validateModelConfig(model);
    validateCameraConfig(camera);
    validateMapConfig(map);
}
```

最后验证三类配置的合法性。如果有必填字段为空，抛异常终止程序。

---

## 三、文件版本构造函数

```cpp
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
```

和目录版本类似，只是直接传入文件路径，不做自动拼接。

---

## 四、`loadModelConfig`

```cpp
void Config::loadModelConfig(const std::string& path) {
    YAML::Node cfg = YAML::LoadFile(path);

    model.modelPath       = cfg["modelPath"].as<std::string>();
    model.imgSize1        = cfg["imgSize1"].as<int>();
    model.iouThreshold1   = cfg["iouThreshold1"].as<float>();
    ...
```

用 `yaml-cpp` 库读取 YAML 文件，按字段名提取值并转换类型。

---

### 可选字段处理

```cpp
    model.minRoiSize  = cfg["minRoiSize"] ? cfg["minRoiSize"].as<int>() : 0;
    model.padRatio    = cfg["padRatio"] ? cfg["padRatio"].as<float>() : 0.0f;
    model.classIdxBase = cfg["classIdxBase"] ? cfg["classIdxBase"].as<int>() : 0;
```

`cfg["key"]` 如果节点存在且非空，条件为 `true`；否则为 `false`。

这是 `yaml-cpp` 提供的**存在性检查**，用于给可选字段设置默认值。

---

### classNames 解析

```cpp
    model.classNames = parseClassNamesNode(cfg["classNames"]);
```

调用前面定义的兼容解析函数。

---

## 五、`loadCameraConfig`

```cpp
void Config::loadCameraConfig(const std::string& path) {
    YAML::Node cfg = YAML::LoadFile(path);

    camera.cameraMatrix    = parseMat3x3(cfg["cameraMatrix"].as<std::vector<double>>());
    camera.distCoeffs      = parseRowMat(cfg["distCoeffs"].as<std::vector<double>>());
    camera.requirePointsNum = cfg["requirePointsNum"].as<int>();
    camera.worldPoints     = parsePoint3fList(cfg["worldPoints"].as<std::vector<std::vector<float>>>());
    camera.meshPath        = cfg["meshPath"] ? cfg["meshPath"].as<std::string>() : "";
}
```

---

### `parseMat3x3`

```cpp
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
```

YAML 中的 `cameraMatrix` 是一个包含 9 个 double 的数组（行优先），这里把它转成 3×3 的 `cv::Mat`。

`i / 3` 得到行号，`i % 3` 得到列号。

---

### `parseRowMat`

```cpp
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
```

畸变系数是 1×N 的行向量，长度不固定（4、5、8、12、14 都可能）。

---

### `parsePoint3fList`

```cpp
std::vector<cv::Point3f> Config::parsePoint3fList(
    const std::vector<std::vector<float>>& data) {
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
```

YAML 格式示例：

```yaml
worldPoints:
  - [0, 0, 0]
  - [1, 0, 0]
  - [1, 1, 0]
  - [0, 1, 0]
```

`reserve` 预先分配内存，避免多次扩容。

---

## 六、验证函数

```cpp
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
```

检查模型的必填字段。如果某个路径为空，说明 YAML 里漏配置了，直接抛异常。

---

```cpp
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
```

检查相机配置的合法性。特别检查 `requirePointsNum` 和 `worldPoints.size()` 是否一致，防止标定时点数不匹配。

---

```cpp
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
```

检查地图配置的合法性。确保尺寸数组长度为 2 且元素为正数。

---

## 七、`loadCalibConfig`

```cpp
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
```

读取 `calib_result.yaml`，解析标定点、旋转矩阵 `R` 和平移向量 `T`。

最后设置 `calib.valid = true`，表示标定结果可用。

---

# 三、从 ConfigManager 层学到的设计要点

## 1. 配置即代码

把所有配置字段显式定义成 C++ 结构体，编译期就能发现类型错误，比运行时读 JSON/YAML 更安全。

## 2. 严格验证

```cpp
validateModelConfig(model);
validateCameraConfig(camera);
validateMapConfig(map);
```

在构造时就验证，有问题立即崩溃，避免带着错误配置运行到一半才发现。

## 3. 兼容多种 YAML 格式

`parseClassNamesNode` 同时支持 sequence 和 map 两种 YAML 写法，提高配置灵活性。

## 4. 可选配置的优雅处理

```cpp
cfg["meshPath"] ? cfg["meshPath"].as<std::string>() : "";
```

存在就读，不存在就给默认值，不抛异常。

## 5. 标定结果隔离

`calib_result.yaml` 单独管理，和基础配置分离。可以独立更新、独立加载失败。
