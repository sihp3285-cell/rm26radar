# `standalone_main.cpp` 逐行讲解

这份文件是项目的**独立运行入口**（非 ROS2 模式）。它把视频读取、检测、位姿解算、小地图绘制、UI 显示全部串在一个进程里，是一个完整的 monolithic 应用程序。

通过阅读这份文件，可以理解整个算法流水线的端到端流程，也能看出为什么后来要拆成 ROS2 节点。

---

## 一、YAML 特化模板

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
            if (!node.IsSequence() || node.size() != 2) {
                return false;
            }
            rhs.x = node[0].as<float>();
            rhs.y = node[1].as<float>();
            return true;
        }
    };
}
```

---

### 为什么需要特化？

`yaml-cpp` 库默认不知道 `cv::Point2f` 怎么序列化/反序列化。通过特化 `YAML::convert<T>`，告诉 `yaml-cpp`：

* `encode`：`cv::Point2f` → YAML sequence `[x, y]`
* `decode`：YAML sequence `[x, y]` → `cv::Point2f`

这是 C++ **模板特化**的经典应用，扩展第三方库以支持自定义类型。

---

## 二、主函数开始

```cpp
int main(int argc, char const *argv[])
{
    Config cfg("/home/delphine/rm/tensorrt10_detect/configs");
    cv::VideoCapture cap("/home/delphine/rm/car_project/test/005.mp4");
```

---

### 硬编码路径

Standalone 模式使用硬编码路径，这是它和 ROS2 模式的最大区别。ROS2 模式下路径通过参数系统配置，standalone 模式下直接写死在代码里。

---

### 打开视频

```cpp
    if (!cap.isOpened()) {
        std::cerr << "错误：无法打开视频文件！" << std::endl;
        return -1;
    }
```

检查视频是否成功打开，失败直接退出。

---

## 三、初始化 Pipeline 和 PoseSolver

```cpp
    DetectPipeline pipeline(cfg);
    PoseSolver poseSolver(cfg.camera.cameraMatrix, cfg.camera.distCoeffs);
```

---

### `DetectPipeline`

用配置初始化三阶段检测流水线。构造函数内部会加载三个 TensorRT 模型，这是整个程序**最耗时的初始化操作**。

---

### `PoseSolver`

用相机内参和畸变系数初始化位姿解算器。此时外参（`R` 和 `T`）还没有求解。

---

## 四、加载 3D Mesh

```cpp
    std::cout << "配置的 mesh 路径: " << cfg.camera.meshPath << std::endl;
    if (!poseSolver.getRaycaster().loadingMesh(cfg.camera.meshPath)) {
        std::cerr << "警告：无法加载 3D 网格文件，将使用平面地面回退方案" << std::endl;
    }
```

尝试加载场地 3D Mesh。如果失败，打印警告但**继续运行**，因为 `Raycaster` 有平地 fallback。

---

## 五、标定流程

```cpp
    cv::Mat calibrateFrame;
    {
        int num = cfg.camera.requirePointsNum;
        cv::namedWindow("Video Preview", cv::WINDOW_NORMAL);
        cv::resizeWindow("Video Preview", 1280, 720);
```

---

### 创建预览窗口

显示视频预览，提示用户按 `S` 键截取当前帧进行标定。

---

### 标定循环

```cpp
        bool calibrationDone = false;

        while (true) {
            cv::Mat tempFrame;
            if (!cap.read(tempFrame)) {
                cap.set(cv::CAP_PROP_POS_FRAMES, 0);
                continue;
            }
            cv::imshow("Video Preview", tempFrame);
```

循环读取视频帧并显示。如果视频读完，跳回开头继续播放。

---

### 按键处理

```cpp
            int key = cv::waitKey(30);
            if (key == 's' || key == 'S') {
                calibrateFrame = tempFrame.clone();
                std::cout << "已截取当前帧！请在弹出窗口中依次点击 "
                          << num << " 个标定点。" << std::endl;
                cv::destroyWindow("Video Preview");
                break;
            }
            if (key == 'q' || key == 27) {
                return -1;
            }
```

* `s`/`S`：截取当前帧，开始标定
* `q`/ESC：退出程序

---

### 加载已有标定结果

```cpp
            if (key == ' ' && !calibrationDone) {
                std::cout << "正在读取calib_result.yaml文件..." << std::endl;
```

按**空格键**可以直接加载已有的 `calib_result.yaml`，跳过重新标定。

---

#### 读取并解析 calib_result.yaml

```cpp
                try {
                    std::filesystem::path configDir =
                        std::filesystem::path("/home/delphine/rm/tensorrt10_detect/configs");
                    std::string calibPath = (configDir / "calib_result.yaml").string();
                    YAML::Node node = YAML::LoadFile(calibPath);
```

---

#### 验证数据格式

```cpp
                    if (node["r"].IsSequence() && node["t"].IsSequence()) {
                        std::vector<double> r_data = node["r"].as<std::vector<double>>();
                        std::vector<double> t_data = node["t"].as<std::vector<double>>();

                        if (r_data.size() == 9 && t_data.size() == 3) {
```

检查 `r` 和 `t` 是否存在且为序列，然后检查元素个数。

---

#### 构造 R 和 T 矩阵

```cpp
                            cv::Mat R(3, 3, CV_64F);
                            cv::Mat T(3, 1, CV_64F);
                            for (int i = 0; i < 9; ++i) {
                                R.at<double>(i / 3, i % 3) = r_data[i];
                            }
                            for (int i = 0; i < 3; ++i) {
                                T.at<double>(i, 0) = t_data[i];
                            }
                            poseSolver.setExtrinsic(R, T);
```

和 `ConfigManager.cpp` 中 `loadCalibConfig` 的解析逻辑完全一致。

---

#### 标定完成

```cpp
                            std::cout << "成功从calib_result.yaml加载R和T矩阵！" << std::endl;
                            calibrationDone = true;
                            cv::destroyWindow("Video Preview");
                            break;
```

设置标志位，关闭预览窗口，跳出标定循环。

---

### 交互式标定

```cpp
        if (!calibrationDone) {
            MouseBack mouseBack("Calibrate 1", num);
            std::vector<cv::Point2f> imagePoints = mouseBack.getPoints(calibrateFrame);
```

如果没有加载已有标定结果，进入交互式标定：用 `MouseBack` 在截取的帧上点击 `num` 个点。

---

### PnP 解算

```cpp
            if(imagePoints.size() == num) {
                poseSolver.calibrate(cfg.camera.worldPoints, imagePoints);
                std::cout << "相机标定（PnP）成功！" << std::endl;
```

把 3D 世界点（来自 `camera.yaml`）和 2D 图像点（用户点击）传入 `PoseSolver::calibrate()`，求解外参。

---

### 保存标定结果

```cpp
                cv::Mat R, T;
                poseSolver.getExtrinsic(R, T);
                std::filesystem::path configDir =
                    std::filesystem::path("/home/delphine/rm/tensorrt10_detect/configs");
                std::string calibPath = (configDir / "calib_result.yaml").string();
```

获取求解出的 `R` 和 `T`，准备保存。

---

### YAML 序列化

```cpp
                YAML::Emitter out;
                out << YAML::BeginMap;
                out << YAML::Key << "image_points" << YAML::Value << YAML::BeginSeq;
                for (const auto& pt : imagePoints) {
                    out << YAML::Flow << YAML::BeginSeq << pt.x << pt.y << YAML::EndSeq;
                }
                out << YAML::EndSeq;
                out << YAML::Key << "r" << YAML::Value << YAML::Flow << YAML::BeginSeq;
                for (int i = 0; i < 9; ++i) {
                    out << R.at<double>(i);
                }
                out << YAML::EndSeq;
                out << YAML::Key << "t" << YAML::Value << YAML::Flow << YAML::BeginSeq;
                for (int i = 0; i < 3; ++i) {
                    out << T.at<double>(i);
                }
                out << YAML::EndSeq;
                out << YAML::EndMap;
```

使用 `yaml-cpp` 的 `YAML::Emitter` 手动构造 YAML 文档：

* `BeginMap` / `EndMap`：开始/结束 map
* `Key` / `Value`：键值对
* `BeginSeq` / `EndSeq`：开始/结束序列
* `YAML::Flow`：紧凑格式（单行）

输出格式：

```yaml
image_points: [[x1, y1], [x2, y2], ...]
r: [r1, r2, ..., r9]
t: [t1, t2, t3]
```

---

### 写入文件

```cpp
                std::ofstream fout(calibPath);
                if (fout.is_open()) {
                    fout << out.c_str();
                    fout.close();
                    std::cout << "标定结果已保存到: " << calibPath << std::endl;
                }
```

---

### 标定失败处理

```cpp
            } else {
                std::cout << "标定点数不足，程序退出。" << std::endl;
                return -1;
            }
        }
```

如果用户点击的点数不足，无法做 PnP，退出程序。

---

### 重置视频位置

```cpp
        cap.set(cv::CAP_PROP_POS_FRAMES, 0);
    }
```

标定完成后，把视频指针重置到第 0 帧，准备开始正式处理。

---

## 六、初始化 RadarMap

```cpp
    RadarMap radarMap(cfg.map.mapPath, cfg.map.isFlip);
    radarMap.calibrate2(cfg.map.race_size[0], cfg.map.race_size[1],
                        cfg.map.map_size[0], cfg.map.map_size[1]);
```

加载地图底图，标定世界坐标到地图像素的转换参数。

---

## 七、初始化 UI

```cpp
    UI ui("Video & Radar");
    bool isPaused = false;
    using Clock = std::chrono::steady_clock;

    auto last_time = Clock::now();
    double fps = 0.0;
```

创建 UI 窗口，初始化暂停状态和 FPS 计算变量。

---

## 八、主循环

```cpp
    while (true)
    {
        cv::Mat frame;
        if(!cap.read(frame)) break;
```

读取一帧视频。如果读取失败（视频结束），退出循环。

注意：这里不像 `video_node` 那样循环播放，而是播完就停。

---

### 运行检测流水线

```cpp
        std::vector<Result> allresults = pipeline.process(frame);
```

调用三阶段检测流水线，获取所有检测结果（车辆 + 装甲板）。

---

### 解算世界坐标

```cpp
        std::vector<Mappoint> mappoints;

        for(const auto& result : allresults)
        {
            if (result.idx == 0 ) continue;

            cv::Point2f wp = poseSolver.middletoworld(result.car_box);
            cv::Point2f mp = radarMap.worldtomap(wp);
            mappoints.push_back({mp, "", result.idx, result.armorColor, result.isDead});
        }
```

---

#### 过滤 CAR

`result.idx == 0` 对应 `robot_id::CAR`，跳过。只处理有明确身份的机器人和哨兵。

---

#### 像素 → 世界 → 地图

```text
result.car_box（像素）
    ↓ poseSolver.middletoworld()
wp（世界坐标，米）
    ↓ radarMap.worldtomap()
mp（地图像素坐标）
```

两层坐标转换，最终得到小地图上的绘制位置。

---

#### 填充 Mappoint

```cpp
mappoints.push_back({mp, "", result.idx, result.armorColor});
```

`label` 传空字符串，`drawMap` 内部会根据 `isDead` 和 `classIdx` 以及 `classNames` 自动生成。

---

### 绘制检测结果

```cpp
        drawDetect(frame, allresults, cfg.model.classNames);
        cv::Mat radarImg = radarMap.drawMap(mappoints, cfg.model.classNames);
```

在原始图像上画检测框，在地图底图上画机器人位置。

---

### 计算 FPS

```cpp
        auto now = Clock::now();
        double dt = std::chrono::duration<double>(now - last_time).count();
        last_time = now;

        double instant_fps = 1.0 / std::max(dt, 1e-6);
        fps = 0.9 * fps + 0.1 * instant_fps;
```

和 `detect_node` 中完全相同的指数平滑 FPS 计算。

---

### 绘制 FPS

```cpp
        cv::putText(frame,
                cv::format("FPS: %.1f", fps),
                cv::Point(20,300),
                cv::FONT_HERSHEY_SIMPLEX,
                5.0,
                cv::Scalar(0, 255, 0),
                2);
```

注意这里的字体大小是 `5.0`，比 ROS2 节点中的 `1.0` 大很多。因为 standalone 模式的显示窗口可能更大，或者这是早期调试用的夸张尺寸。

---

### UI 更新

```cpp
        int key = ui.update(frame, radarImg, isPaused);
```

把检测图像和小地图传给 `UI`，显示拼接后的窗口，并返回用户按键。

---

### 按键处理

```cpp
        if (key == 'q' || key == 27)  break;
        if (key == ' ')  isPaused = !isPaused;
```

* `q`/ESC：退出
* `空格`：暂停/播放切换

---

## 九、清理

```cpp
    cap.release();
    cv::destroyAllWindows();
    return 0;
}
```

释放视频资源，销毁所有 OpenCV 窗口，正常退出。

---

# 十、从 standalone_main 学到的设计要点

## 1. Monolithic vs Distributed

Standalone 模式把所有功能串在一个 `main` 里：

```text
视频 → 检测 → 解算 → 地图 → 显示
```

优点是简单直观，缺点是耦合严重、难以替换模块。

ROS2 模式把同样的流程拆成 5 个节点，通过话题通信，实现了真正的模块化。

## 2. 标定的两种入口

```text
S 键 → 交互式标定 → 保存到 calib_result.yaml
空格键 → 直接加载 calib_result.yaml
```

既支持重新标定，又支持快速复用已有结果。

## 3. 坐标转换链

```text
car_box（像素）→ middletoworld() → world（米）→ worldtomap() → map（像素）
```

Standalone 模式在一处代码里连续调用两层转换，ROS2 模式下这两层分别属于 `pose_node` 和 `map_node`。

## 4. 深拷贝保护

```cpp
calibrateFrame = tempFrame.clone();
```

截取标定帧时用 `clone()`，确保保存的是当前帧的独立副本，不受后续视频读取影响。
