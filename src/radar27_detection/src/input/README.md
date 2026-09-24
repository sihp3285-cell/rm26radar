# 内部图像输入

`VideoSource` 和 `CameraSource` 实现同一个 `FrameSource` 接口，由 `detect_node`
直接持有（进程内的采集线程），不再经过 `/image_raw` 的 ROS Image 订阅。

## CameraSource

构建 `radar27_detection` 时传入 `-DBUILD_CAMERA=ON`，自动检测已安装的 MVS、
Galaxy SDK。默认 OFF，视频输入无需安装相机 SDK。非标准位置可以显式指定
`HIK_INCLUDE_DIR`、`HIK_LIBRARY`、`DAHENG_INCLUDE_DIR`、`DAHENG_LIBRARY`。
指定 ON 但没有任何完整 SDK 时，CMake 报错；只有海康支持时选择大恒也会明确报错。

海康和大恒两条路的实现方式不同：

- **海康复用 `rb26SDK` 的 `sdk::HikCamera`**（`rb26SDK/src/hik/hik.cpp`，
  原样使用，不在本包内改动 SDK）。选机、曝光/增益/Gamma/白平衡、取流、颜色
  转换、翻转镜像都在那一层；`CameraSource` 只做适配：帧号、单调时钟、
  `Timeout`/`Error` 分类、翻转和内存所有权。
- **大恒仍直连 Galaxy SDK**（本文件内的 `GX_*` 调用）。

```cpp
radar27_detection::input::CameraSourceConfig config;
config.brand = "hik";
config.serial_number = "实际序列号";
config.exposure_time_us = 6000;
radar27_detection::input::CameraSource source(config);
source.open();
auto result = source.read();
// 根据 result.status 处理 Ok / Timeout / Error。
source.close();
```

### 海康路径的已知限制（来自 vendor 实现，未在 SDK 内修改）

- 取图超时写死 100 ms，`camera.timeout_ms` 对海康不生效；`getFrame()` 失败只返回
  空 Mat，拿不到 SDK 错误码。适配层因此按“连续失败时长”区分：1 s 内算可重试的
  `Timeout`，超过就升级为粘性 `Error`，避免相机掉线时无限重试。
- 彩色相机默认输出 Bayer，先转换再释放缓存，路径安全。若把相机设成
  `BGR8_Packed`/`RGB8_Packed`，vendor 实现会**先 `MV_CC_FreeImageBuffer` 再返回
  缓存视图**：适配层会复制成自有内存（不会把悬空指针交给流水线），但内容有可能
  已被回收，属于该分支的固有缺陷。需要绝对可靠就保持 Bayer 输出。
- vendor 析构无条件 `MV_CC_Finalize()`（进程级），所以仍然只允许一个实例。
- `CameraInit` 失败时 vendor 不回收已创建的句柄；这里把打开失败当作致命错误处理。

### 通用约定

- 大恒按序列号打开，支持 Mono/Bayer 的 8 位及非打包 10/12 位、RGB8/BGR8；
  不支持的像素格式返回 Error，不把未知格式解释成三通道图像。
- 输出帧拥有自己的像素，不受下一次采集或 close 影响。消费者只读共享；
  要绘制或原地修改的消费者先复制。
- 帧号从零开始，仅成功读帧递增。时间为主机单调时钟相对 open 的时间；海康路径
  在 `getFrame()` 返回后采样，因此包含颜色转换耗时，不是硬件曝光时间。
- Timeout 可重试；其他读取错误保持 Error，调用 close/open 后重新初始化。
- open/read/close 只能顺序调用。停止采集线程并 join 后再 close；大恒的取帧等待
  受 `timeout_ms` 限制，海康固定 100 ms；厂商 SDK 初始化、枚举和关闭耗时都不受
  该参数限制。
- 一个进程内只允许一个新的 CameraSource 打开，避免并行实例提前释放全局 SDK。

## 验证

开启 BUILD_TESTING 后，`test_video_source` 使用临时视频验证图像内存和时间轴；
`test_camera_source_disabled` 验证没有 SDK 的错误路径。
启用海康构建时，`test_camera_source_hik` 同时编译 production 的 `camera_source.cpp`
和 `rb26SDK/src/hik/hik.cpp`，用真实 SDK 头文件加桩函数验证：会话独占、空 Mat 的
重试/升级策略、Bayer 与 BGR8_Packed 两条路径的内存所有权、清理顺序、部分初始化
失败后的重开，不链接真库、不访问真实相机。

这些检查不替代海康、大恒设备上的曝光、颜色、掉线和连续采集测试。
