# 相机节点系统 (Camera Node System)

## 1. 系统架构

本项目支持 **海康机器人 (Hikvision)** 和 **大恒图像 (Daheng)** 两种工业相机，通过 `rb26SDK` 抽象层统一接口，向上层 `CameraNode` 暴露与品牌无关的操作。

```
┌──────────────────────────────────────────────────────────┐
│  CameraNode (ROS2 Component / Standalone Executable)      │
│                                                           │
│  ┌──────────────────────────────────────────────────┐    │
│  │  状态机: sdk_initialized_ → camera_opened_        │    │
│  │  线程:   capture_thread_ + record_thread_         │    │
│  │  输出:   /image_raw (sensor_msgs/Image, bgr8)     │    │
│  └──────────────┬───────────────────────────────────┘    │
│                 │  grabFrame()                            │
│  ┌──────────────▼───────────────────────────────────┐    │
│  │  CamreaExmple<T>  (模板包装类)                     │    │
│  │  ├── CamreaExmple<HikCamera>                      │    │
│  │  └── CamreaExmple<DahengCamera>                   │    │
│  └──────────────┬───────────────────────────────────┘    │
│                 │  CameraInit() / getFrame() / ~dtor()    │
│  ┌──────────────▼───────────────────────────────────┐    │
│  │  rb26SDK (静态库, 条件编译)                        │    │
│  │  ├── hik.cpp   ← libMvCameraControl.so            │    │
│  │  └── daheng.cpp ← libgxiapi.so + libdximageproc   │    │
│  └──────────────────────────────────────────────────┘    │
└──────────────────────────────────────────────────────────┘
```

**文件结构**：

| 文件 | 职责 |
|---|---|
| [camera_node.cpp](../src/tensorrt_detect/src/nodes/camera_node.cpp) | ROS2 节点：参数管理、状态机、生命周期、多线程 |
| [rb26SDK/CMakeLists.txt](../rb26SDK/CMakeLists.txt) | 条件编译：检测 SDK 库、导出 PUBLIC 宏定义 |
| [rb26SDK/include/CamreaExmple.hpp](../rb26SDK/include/CamreaExmple.hpp) | 模板包装类，隐藏品牌差异 |
| [rb26SDK/include/hik/hik.hpp](../rb26SDK/include/hik/hik.hpp) | HikCamera 类定义 + 内联方法 |
| [rb26SDK/src/hik/hik.cpp](../rb26SDK/src/hik/hik.cpp) | HikCamera 实现：设备枚举、初始化、取流、停止 |
| [rb26SDK/include/daheng/daheng.hpp](../rb26SDK/include/daheng/daheng.hpp) | DahengCamera 类定义 + 内联方法 |
| [rb26SDK/src/daheng/daheng.cpp](../rb26SDK/src/daheng/daheng.cpp) | DahengCamera 实现：设备打开、参数配置、取流、图像处理 |
| [rb26SDK/include/sdk.h](../rb26SDK/include/sdk.h) | 基类 Camera：公共属性（分辨率、帧率、品牌枚举） |

---

## 2. 编译时条件构建

### 2.1 SDK 检测逻辑

[rb26SDK/CMakeLists.txt](../rb26SDK/CMakeLists.txt) 在 CMake 配置阶段自动检测可用的相机 SDK：

```cmake
# Hikvision MVS SDK — 搜索 /opt/MVS/lib/64
find_library(MVCAMERA_LIB MvCameraControl HINTS ${MVS_LIB_DIR})
if(MVCAMERA_LIB)
    set(HAS_HIK ON)
    target_compile_definitions(rb26SDK PUBLIC RB26SDK_HAS_HIK=1)
endif()

# Daheng Galaxy SDK — 搜索系统库路径
find_library(GXIAPI_LIB gxiapi)
if(GXIAPI_LIB)
    set(HAS_DAHENG ON)
    target_compile_definitions(rb26SDK PUBLIC RB26SDK_HAS_DAHENG=1)
endif()
```

**条件编译效果**：

- 源文件 `hik.cpp` / `daheng.cpp` 仅在对应 SDK 存在时才编译进 `librb26SDK.a`
- 宏 `RB26SDK_HAS_HIK` / `RB26SDK_HAS_DAHENG` 通过 PUBLIC 属性传递给消费者（CameraNode）
- 头文件中的类定义、`CamreaExmple` 模板的 `static_assert`、CameraNode 中的品牌分支均由预处理器守卫

### 2.2 构建选项

```bash
# 默认: BUILD_CAMERA=ON，编译相机节点
colcon build

# 完全跳过相机节点编译（无相机 SDK 时使用）
colcon build --cmake-args -DBUILD_CAMERA=OFF
```

### 2.3 异常处理

如果两个 SDK 都未找到，CMake 配置阶段直接报错退出：

```
FATAL_ERROR: rb26SDK: 没有可用的相机 SDK，至少需要 Hikvision 或 Daheng 之一
```

---

## 3. SDK 抽象层设计

### 3.1 类继承结构

```
sdk::Camera (抽象基类)
  ├── sensorWidth, sensorHeight  — 相机分辨率
  ├── fps                        — 帧率计时器
  ├── cap_init                   — 设备初始化成功标志
  ├── cap_sn                     — 相机序列号
  └── camera_breand              — 品牌枚举 {NullClass, Daheng, Hik}
        │
        ├── sdk::HikCamera : public Camera
        │     ├── CameraSDKInit()  static — MV_CC_Initialize()
        │     ├── CameraInit()          — 枚举设备 → 打开 → 配置 → 开采
        │     ├── getFrame()            — MV_CC_GetImageBuffer → Bayer→RGB
        │     ├── capture_start()       — 设置曝光/增益/gamma → MV_CC_StartGrabbing
        │     ├── capture_stop()        — StopGrabbing → CloseDevice → DestroyHandle
        │     └── ~HikCamera()          — 按 cap_init 分级的析构
        │
        └── sdk::DahengCamera : public Camera
              ├── CameraSDKInit()  static — GXInitLib()
              ├── CameraInit()          — 打开设备 → 配置 → 开采
              ├── getFrame()            — GXGetImage → Bayer处理 → RGB
              ├── ProcessData()         — DxImageMirror / DxRaw16toRaw8 / DxRaw8toRGB24
              └── ~DahengCamera()       — 按 cap_init 分级的析构
```

### 3.2 模板包装类 CamreaExmple`<T>`

```cpp
// CamreaExmple.hpp
template<class CamreaType>
class CameraExmple : public CamreaType {
public:
    // 编译期类型约束 — 仅允许已启用的后端
    static_assert(
#if defined(RB26SDK_HAS_HIK) && defined(RB26SDK_HAS_DAHENG)
        std::is_same<CamreaType, HikCamera>::value ||
        std::is_same<CamreaType, DahengCamera>::value
#elif defined(RB26SDK_HAS_HIK)
        std::is_same<CamreaType, HikCamera>::value
#elif defined(RB26SDK_HAS_DAHENG)
        std::is_same<CamreaType, DahengCamera>::value
#endif
        , "Template parameter CamreaType must be a supported camera type"
    );

    cv::Mat getFrame(bool flip = false, bool mirror = false) {
        frame = CamreaType::getFrame(flip, mirror);
        return frame;
    }

    void putFps() { /* 从基类 fps 读取帧率 */ }
    void putResolution() { /* 打印 sensorWidth × sensorHeight */ }
};
```

**设计意图**：`CameraExmple<T>` 不是简单的 thin wrapper — 它通过继承将 `HikCamera`/`DahengCamera` 的接口统一为 `getFrame()`，同时保留了基类 `Camera` 的属性访问（`sensorWidth`、`fps` 等）。上层 CameraNode 只需持有 `unique_ptr<CameraExmple<T>>` 即可操作任意品牌的相机。

### 3.3 HikCamera 实现详解

**设备枚举与初始化**（[hik.cpp](../rb26SDK/src/hik/hik.cpp)）：

```cpp
bool HikCamera::CameraInit(char *sn, bool autoWhiteBalance,
                           int expoosureTime, double gainFactor,
                           double dGammaParam) {
    // 1. 枚举 USB 设备
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);

    // 2. 按序列号匹配目标设备
    bool exist = ChoiceCamrea(device_list.pDeviceInfo,
                              (unsigned char*)sn, cameraIndex);

    // 3. 创建句柄 → 打开设备
    MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[cameraIndex]);
    nRet = MV_CC_OpenDevice(camera_handle_);

    // 4. 配置采集参数 + 开始取流
    capture_start(expoosureTime, gainFactor, dGammaParam);
    //   内部调用: SetEnumValue(BalanceWhiteAuto/ExposureAuto/GainAuto/...)
    //            SetFloatValue(Gamma/ExposureTime/Gain)
    //            MV_CC_StartGrabbing(camera_handle_)

    // 5. 读取实际分辨率
    MV_CC_GetIntValueEx(camera_handle_, "WidthMax", &width_max);
    MV_CC_GetIntValueEx(camera_handle_, "Height", &height_max);
    this->sensorWidth  = width_max.nCurValue;
    this->sensorHeight = height_max.nCurValue;

    cap_init = true;  // 标志设备已成功打开
    return true;
}
```

**取帧**（[hik.cpp](../rb26SDK/src/hik/hik.cpp)）：

```cpp
cv::Mat HikCamera::getFrame(bool flip, bool mirror) {
    // 1. 从驱动获取图像缓冲 (100ms 超时)
    nRet = MV_CC_GetImageBuffer(camera_handle_, &raw, nMsec);

    // 2. Bayer → RGB 转换 (基于哈希表分发 4 种 Bayer 模式)
    const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
        {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
        {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
        {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
        {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}
    };
    cv::cvtColor(raw_img, img, type_map.at(raw.stFrameInfo.enPixelType));

    // 3. 可选翻转/镜像
    if (flip)   cv::flip(img, img, 0);
    if (mirror) cv::flip(img, img, 1);

    // 4. 释放图像缓冲
    MV_CC_FreeImageBuffer(camera_handle_, &raw);

    update_timer();  // 更新帧率计时器
    return img;
}
```

### 3.4 DahengCamera 实现详解

**与 HikCamera 的关键差异**：

| 方面 | Hikvision MVS | Daheng Galaxy |
|---|---|---|
| SDK 初始化 | `MV_CC_Initialize()` | `GXInitLib()` |
| 设备打开 | `MV_CC_OpenDevice(handle)` | `GXOpenDevice(openParam, &hDevice)` — 支持 SN/IP/MAC/Index 多种模式 |
| 取流 | `MV_CC_GetImageBuffer` + 手动 Bayer→RGB | `GXGetImage` + DxImageProc 系列函数处理 |
| 图像处理 | OpenCV `cvtColor` | DxImageMirror / DxRaw16toRaw8 / DxRaw8toRGB24 |
| Gamma | `MV_CC_SetFloatValue("Gamma", ...)` | `DxGetGammatLut` 计算 Gamma 查找表 |
| 开采/停采 | `MV_CC_StartGrabbing` / `MV_CC_StopGrabbing` | `GX_COMMAND_ACQUISITION_START` / `GX_COMMAND_ACQUISITION_STOP` |
| buffer 管理 | SDK 内部管理，`FreeImageBuffer` 归还 | 用户自行 malloc/free (`pRaw8Buffer`, `pRGBframeData`, `pGammaLut`) |

**ProcessData 图像处理管线**（[daheng.cpp](../rb26SDK/src/daheng/daheng.cpp)）：

```cpp
void DahengCamera::ProcessData(void *pImageBuf, void *pImageRaw8Buf,
                               void *pImageRGBBuf, int nImageWidth,
                               int nImageHeight, int nPixelFormat,
                               int nPixelColorFilter,
                               bool flip, bool mirror) {
    switch (nPixelFormat) {
    case GX_PIXEL_FORMAT_BAYER_GR8:  // 8-bit Bayer
        if (mirror) {
            DxImageMirror(pImageBuf, pMirrorBuffer, ...);     // 水平镜像
            DxRaw8toRGB24(pMirrorBuffer, pImageRGBBuf, ...);  // Bayer→RGB
        } else {
            DxRaw8toRGB24(pImageBuf, pImageRGBBuf, ...);
        }
        break;

    case GX_PIXEL_FORMAT_BAYER_GR12: // 12-bit Bayer
        if (mirror) {
            DxImageMirror(pImageBuf, pMirrorBuffer, ...);
            DxRaw16toRaw8(pMirrorBuffer, pImageRaw8Buf, ...); // 16→8 bit
            DxRaw8toRGB24(pImageRaw8Buf, pImageRGBBuf, ...);
        } else {
            DxRaw16toRaw8(pImageBuf, pImageRaw8Buf, ...);
            DxRaw8toRGB24(pImageRaw8Buf, pImageRGBBuf, ...);
        }
        break;
    // ... GR10, BG8, MONO12, MONO8 等格式
    }
}
```

---

## 4. CameraNode 状态机

### 4.1 状态标志

```cpp
// camera_node.cpp — 成员变量
bool camera_opened_   = false;  // CameraInit() 成功，设备已打开并取流
bool sdk_initialized_ = false;  // CameraSDKInit() 已调用成功
```

### 4.2 状态转移图

```
构造函数开始
    │
    ▼
[参数声明 & 读取]
    │
    ├── camera_brand 未知 ──→ RCLCPP_FATAL → throw → 进程退出
    │
    ▼
[make_unique<CameraExmple<T>>()]  ← 创建 SDK 对象（轻量，仅成员初始化）
    │
    ▼
[CameraSDKInit()]                  ← MV_CC_Initialize() / GXInitLib()
    │
    ├── 失败 ──→ sdk_initialized_=false → 跳过 CameraInit
    │                                       → init_ok=false → throw
    │
    ├── 成功 ──→ sdk_initialized_=true
    │               │
    │               ▼
    │           [CameraInit()]     ← 枚举设备 → 打开 → 配置 → 开采
    │               │
    │               ├── 失败 ──→ camera_opened_=false → init_ok=false → throw
    │               │              (SDK 析构函数检查 cap_init=false，仅 Finalize/CloseLib)
    │               │
    │               └── 成功 ──→ camera_opened_=true, cap_init=true
    │                               │
    │                               ▼
    │                           [读取分辨率]
    │                               │
    │                               ▼
    │                           [可选: 初始化内录]
    │                               │
    │                               ▼
    │                           [创建 Publisher + 启动 capture_thread_]
    │                               │
    │                               ▼
    │                           [captureLoop 运行中]
    │                               │
    └───────────────────────────────┘
                                    │
                            ~CameraNode() 析构
                                    │
                                    ▼
                           1. is_running_ = false
                           2. join record_thread_
                           3. join capture_thread_
                           4. hik_.reset() / daheng_.reset()
                              └── SDK 析构函数检查 cap_init
                                   ├── true:  StopGrabbing → CloseDevice → DestroyHandle → Finalize
                                   └── false: 跳过设备级清理，仅 Finalize/CloseLib
```

### 4.3 错误处理策略

| 错误场景 | 行为 | 进程结果 |
|---|---|---|
| 未知 camera_brand | `RCLCPP_FATAL` → `throw std::runtime_error` | 进程退出 (main catch → shutdown) |
| CameraSDKInit 失败 | `sdk_initialized_=false` → 跳过 CameraInit → `throw` | 进程退出 |
| CameraInit 失败 | `camera_opened_=false` → `throw` → SDK 析构安全 (cap_init=false) | 进程退出 |
| 相机中途断开 | `getFrame()` 返回空 Mat → `captureLoop` 跳过 → sleep 2ms 重试 | 进程继续运行 |
| 正常关闭 (Ctrl+C) | 析构函数按状态分级释放 | 优雅退出 |

**为什么 fatal 后要 throw 而不是 return**：

`RCLCPP_FATAL` 仅记录日志，不终止进程。构造函数中 `return` 会导致一个半初始化状态的 Node 继续存活在 executor 中，既不工作也不释放。`throw std::runtime_error` 确保进程立即退出，且已构造的成员变量（`hik_`/`daheng_` unique_ptr）自动析构，触发 SDK 的安全清理路径。

---

## 5. 多线程模型

```
┌─────────────────────────────────────────────┐
│              CameraNode 线程架构              │
│                                              │
│  主线程 (ROS2 spin)                          │
│    └── 构造函数（同步初始化）                  │
│    └── 析构函数（同步清理）                    │
│                                              │
│  capture_thread_                             │
│    └── captureLoop()                         │
│         while (rclcpp::ok() && is_running_)  │
│           frame = grabFrame()                │
│           if enable_recording_:              │
│              push to record_queue_           │
│              notify record_thread_           │
│           publish(frame)                     │
│                                              │
│  record_thread_ (可选，enable_recording=true) │
│    └── recordLoop()                          │
│         while (is_running_ || queue not empty)│
│           wait on queue_cv_                  │
│           pop frame → writer_.write()        │
└─────────────────────────────────────────────┘
```

**同步机制**：

| 对象 | 保护方式 |
|---|---|
| `record_queue_` | `std::mutex queue_mutex_` + `std::condition_variable queue_cv_` |
| `is_running_` | `std::atomic<bool>` — capture/record 线程读取，主线程写入 |
| `writer_` | 仅 record_thread_ 访问，无需锁 |
| `hik_` / `daheng_` | 仅 capture_thread_ 调用 `getFrame()`，构造/析构在主线程 |

**线程安全保证**：
- 采集线程和录制线程通过生产者-消费者队列解耦，队列有界（`MAX_QUEUE_SIZE=60`，约 2 秒缓冲 @30fps）
- 录制线程使用 `condition_variable::wait`，不忙等
- 关闭顺序：先设置 `is_running_=false` → 通知录制线程 → join 录制线程 → join 采集线程

---

## 6. 内录系统

### 6.1 启用条件

```yaml
# ros2_params.yaml
camera_node:
  ros__parameters:
    enable_recording: true
    record_path: "/home/delphine/rm/recording/"
```

### 6.2 编码管道

```cpp
// 首选 GStreamer 硬件编码管道
std::string pipeline =
    "appsrc ! videoconvert ! video/x-raw, format=I420 ! "
    "x264enc bitrate=15000 speed-preset=ultrafast tune=zerolatency ! "
    "h264parse ! mp4mux ! filesink location=" + full_path;

writer_.open(pipeline, cv::CAP_GSTREAMER, 0, 30.0, cv::Size(w, h));

// 降级方案: 原生 OpenCV mp4v 编码
if (!writer_.isOpened()) {
    writer_.open(full_path,
        cv::VideoWriter::fourcc('m','p','4','v'), 30.0,
        cv::Size(w, h));
}
```

### 6.3 文件命名

输出文件名格式：`cam_YYYYMMDD_HHMMSS.mp4`，基于构造时的系统时间生成。路径由 `record_path` 参数指定，自动补全末尾 `/`。

---

## 7. 配置参数

### 7.1 ROS2 Parameters

```yaml
# ros2_params.yaml — camera_node 段
camera_node:
  ros__parameters:
    # ── 相机标识 ──
    camera_brand: "hik"           # "hik" 或 "daheng" (大小写不敏感)
    camera_sn: "DA7831910"        # 相机序列号，用于 USB 总线上唯一识别

    # ── 图像质量 ──
    auto_white_balance: true      # 自动白平衡 (Hik: 连续模式; Daheng: 连续/关闭)
    exposure_time: 10000          # 曝光时间 (μs)
    gain: 0.7                     # 增益 (0.0~1.0, 最大增益的分数)
    gamma: 0.3                    # Gamma 值

    # ── 输出配置 ──
    topic_name: "/image_raw"      # 发布话题名
    frame_id: "camera_frame"      # ROS frame_id

    # ── 内录 ──
    enable_recording: false       # 是否启用本地视频录制
    record_path: "/home/delphine/rm/recording/"
```

### 7.2 相机内参配置

```yaml
# configs/camera.yaml
cameraMatrix:
  - [fx,  0, cx]
  - [ 0, fy, cy]
  - [ 0,  0,  1]
distCoeffs: [k1, k2, p1, p2, k3]
```

此文件由 PoseNode 使用（PnP 解算 + 射线检测），CameraNode 本身不读取。但相机接入后必须重新标定以获取正确的内参。

---

## 8. 话题与服务

### 8.1 发布话题

| 话题 | 类型 | QoS | 说明 |
|---|---|---|---|
| `/image_raw` | `sensor_msgs/Image` | reliable, queue=1 | bgr8 编码的原始图像帧，帧率由相机硬件决定 |

### 8.2 发布者创建时机

```cpp
// camera_node.cpp — 构造函数末尾
// Publisher 在相机成功初始化后创建，但即使相机未打开也会创建
// (camera_opened_=false 时仅创建 Publisher，不启动采集线程)
pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic, rclcpp::QoS(1));

if (camera_opened_) {
    is_running_ = true;
    capture_thread_ = std::thread(&CameraNode::captureLoop, this);
}
```

**设计理由**：即使相机初始化失败（进程即将退出），Publisher 也已创建，下游节点不会因 topic 不存在而超时等待。实际采集仅在 `camera_opened_=true` 时启动。

---

## 9. 部署模式

### 9.1 Composable Node（推荐，零拷贝）

```python
# detect_pipeline.launch.py 中使用
ComposableNode(
    package='tensorrt_detect',
    plugin='tensorrt_detect::CameraNode',
    name='camera_node',
    parameters=[param_file],
    extra_arguments=[{'use_intra_process_comms': True}],
)
```

CameraNode 作为 component 加载到 `component_container` 中，与 DetectNode、PoseNode、MapNode 共享进程空间。`/image_raw` 通过 intra-process 零拷贝传递给 DetectNode，避免 ~6MB 图像的序列化开销。

### 9.2 Standalone Executable（独立进程，调试用）

```bash
ros2 run tensorrt_detect camera_node --ros-args --params-file ros2_params.yaml
```

独立模式用于调试相机参数（曝光、增益、gamma），不依赖其他节点。

---

## 10. 快速启动检查清单

### 10.1 环境准备

```bash
# 1. 确认相机 SDK 已安装
ls /opt/MVS/lib/64/libMvCameraControl.so        # Hikvision
# 或
ldconfig -p | grep gxiapi                         # Daheng

# 2. 确认环境变量
export LD_LIBRARY_PATH=/opt/MVS/lib/64:$LD_LIBRARY_PATH

# 3. 确认相机已连接 (USB3)
lsusb | grep -i "hik\|daheng\|MV"
```

### 10.2 编译验证

```bash
cd ~/rm/tensorrt10_detect
colcon build 2>&1 | grep "rb26SDK: Found"
# 期望输出:
#   rb26SDK: Found MvCameraControl → 启用 Hikvision 相机
#   rb26SDK: gxiapi 未找到，跳过 Daheng 支持（仅 Hikvision 可用）
```

### 10.3 配置相机参数

1. 获取相机序列号（查看相机机身标签或 SDK 自带的设备枚举工具）
2. 编辑 [ros2_params.yaml](../src/tensorrt_detect/config/ros2_params.yaml)，修改 `camera_sn`
3. 按场景调整 `exposure_time` / `gain` / `gamma`（室外强光 → 低曝光 + 低增益；室内弱光 → 高曝光 + 高增益）

### 10.4 启动

```bash
source install/setup.bash
ros2 launch tensorrt_detect detect_pipeline.launch.py mode:=camera
```

### 10.5 标定

相机首次接入后必须进行 PnP 标定（通过 Qt GUI 的"标定"按钮或 calibrate_node），生成 `calib_result.yaml`，否则 PoseNode 无法解算世界坐标。

---

## 11. 故障排查

| 症状 | 可能原因 | 排查方法 |
|---|---|---|
| 编译报 `gxiapi not found` | Daheng SDK 未安装或不在搜索路径 | `find / -name "libgxiapi*"` 确认；设置 `CMAKE_LIBRARY_PATH` |
| 启动报 `未知 camera_brand` | `camera_brand` 参数拼写错误 | 仅支持 `hik`/`Hik` 或 `daheng`/`Daheng` |
| 启动报 `相机初始化失败` | 序列号错误或相机未连接 | `lsusb` 确认设备；用 SDK 工具枚举设备获取正确 SN |
| 启动后无图像 | 曝光/增益参数不匹配场景 | 逐步增大 `exposure_time` 或 `gain` |
| `camera_opened_=0 sdk_initialized_=0` | SDK 库加载失败 | `ldd` 检查 `libMvCameraControl.so` 依赖；确认 `LD_LIBRARY_PATH` |
| `camera_opened_=0 sdk_initialized_=1` | SDK init 成功但设备打开失败 | 检查 USB 连接、设备权限 (`sudo chmod 777 /dev/bus/usb/...`) |
| 帧率过低 | USB 带宽不足或曝光时间过长 | 减少 `exposure_time`；使用 USB3 端口；检查 `lsusb -t` 带宽分配 |
