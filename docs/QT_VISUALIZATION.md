# Qt 可视化系统 (Qt Visualization with GPU Rendering)

## 1. 架构概览

Qt 可视化系统是一个 **独立进程** 的 ROS2 节点，负责对整个检测流水线的结果进行实时图形化展示。核心设计目标：**最小化 CPU 开销，最大化 GPU 利用**。

[qt_display_node.cpp](../src/tensorrt_detect/src/nodes/qt_display_node.cpp) — 总共 854 行，包含 3 个核心类。

```
┌─────────────────────────────────────────────────────────────┐
│                      独立进程 (qApp)                         │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐   │
│  │ DisplayWindow (QMainWindow)                          │   │
│  │                                                      │   │
│  │  ┌─ Top Bar ──────────────────────────────────────┐ │   │
│  │  │ Status Label │ Outpost Status │ Tactics Alert   │ │   │
│  │  │ car=3.2ms armor=1.5ms ... fps=25.3             │ │   │
│  │  └────────────────────────────────────────────────┘ │   │
│  │                                                      │   │
│  │  ┌─────── Content (HBox) ───────────────────────┐   │   │
│  │  │                                               │   │   │
│  │  │  ┌─────────────────┐  ┌────────────────────┐ │   │   │
│  │  │  │ GLVideoWidget   │  │ Radar Map (QLabel) │ │   │   │
│  │  │  │                 │  │                    │ │   │   │
│  │  │  │  OpenGL Texture │  │  cv::Mat → QPixmap │ │   │   │
│  │  │  │  BGR→RGB Shader │  │  KeepAspectRatio   │ │   │   │
│  │  │  │  GPU Bilinear   │  │  SmoothTransform   │ │   │   │
│  │  │  │  stretch: 3     │  │  stretch: 1        │ │   │   │
│  │  │  └─────────────────┘  └────────────────────┘ │   │   │
│  │  └───────────────────────────────────────────────┘   │   │
│  │                                                      │   │
│  │  ┌── Bottom Bar ───────────────────────────────────┐ │   │
│  │  │ [重新标定] [重新框定 ROI] [蓝方视角/红方视角切换] │ │   │
│  │  │      操作状态提示 ("标定成功 / 失败: ...")        │ │   │
│  │  └────────────────────────────────────────────────┘ │   │
│  └──────────────────────────────────────────────────────┘   │
│                                                             │
│  QtDisplayNode (ROS2 Node — 非 Qt 线程)                     │
│  ┌──────────────────────────────────────────────────────┐   │
│  │ Subscriptions:                                        │   │
│  │   /detected_image  → frame update (mutex protected)   │   │
│  │   /map_image       → map update                       │   │
│  │   /armor_detections → outpost status                  │   │
│  │   /map_tactics     → tactical analysis                │   │
│  │   /pipeline_timing → timing stats                     │   │
│  │                                                       │   │
│  │ Publish:                                              │   │
│  │   /flip_team (Bool) → team perspective switch         │   │
│  └──────────────────────────────────────────────────────┘   │
│                                                             │
│  QTimer (33ms ≈ 30 FPS) → DisplayWindow::refresh()          │
│     ├── fetchData() from QtDisplayNode (mutex lock)         │
│     ├── GLVideoWidget::setFrame() → update() → paintGL()    │
│     ├── QLabel::setPixmap() ← cvMatToQPixmap(map)          │
│     └── updateStatus() → timing + outpost + tactics         │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. GLVideoWidget — GPU 加速视频渲染

[qt_display_node.cpp:46-203](../src/tensorrt_detect/src/nodes/qt_display_node.cpp)

### 2.1 为什么需要 GLVideoWidget

传统的 `QLabel::setPixmap()` 路径：

```
cv::Mat BGR (GPU 上的结果)
    ↓ cudaMemcpy D2H                    — GPU→CPU 传输
    ↓ cv::cvtColor BGR→RGB              — CPU 像素转换
    ↓ QImage::QImage(uchar*)             — 浅拷贝包装
    ↓ QPixmap::fromImage()              — 深拷贝到 QPixmap
    ↓ QPixmap::scaled()                 — CPU 双线性缩放
    ↓ QLabel::setPixmap()               — 上传到 OpenGL 纹理 (Qt 内部)
    ↓ GPU 合成                          — Qt Widget 渲染
```

**问题**：3 次全帧 CPU 拷贝 + CPU 双线性缩放，在高分辨率下成为瓶颈。

GLVideoWidget 路径：

```
cv::Mat BGR (GPU 上的结果)
    ↓ cudaMemcpy D2H (仅一次)           — GPU→CPU 传输 (无可避免，数据在 GPU)
    ↓ std::memcpy → frame_data_         — 单次 CPU 拷贝到内部 buffer
    ↓ glTexSubImage2D GL_RGB            — DMA 上传到 GPU 纹理 (不改数据)
    ↓ Fragment Shader .bgra swizzle     — GPU 完成 BGR→RGB 交换 (零成本)
    ↓ GPU 双线性缩放                    — 纹理采样时硬件完成
    ↓ glDrawArrays GL_TRIANGLE_FAN      — 渲染到屏幕
```

**改进**：
- 消除 BGR→RGB 的 CPU 转换（shader 中 `.bgra` 交换通道）
- 消除 CPU 双线性缩放（GPU 纹理采样硬件实现）
- 纹理大小不变时使用 `glTexSubImage2D`，避免重新分配 GPU 存储

### 2.2 OpenGL 初始化

```cpp
void GLVideoWidget::initializeGL() {
    initializeOpenGLFunctions();

    // ===== 编译 Shader =====

    // Vertex Shader: 接受 NDC 坐标 [−1,1]²，输出纹理坐标 [0,1]²
    // OpenCV 图像原点在左上，OpenGL 纹理原点在左下 → 翻转 Y 轴
    static const char *vert_src =
        "attribute vec2 aPos;                 \n"
        "varying   vec2 vTexCoord;            \n"
        "void main() {                        \n"
        "    gl_Position = vec4(aPos, 0.0, 1.0);\n"
        "    vTexCoord = vec2(aPos.x*0.5+0.5, 1.0-(aPos.y*0.5+0.5));\n"
        "}";

    // Fragment Shader: 采样纹理 + BGR↔RGB 通道交换
    static const char *frag_src =
        "varying vec2 vTexCoord;              \n"
        "uniform sampler2D uTex;              \n"
        "void main() {                        \n"
        "    vec4 c = texture2D(uTex, vTexCoord);\n"
        "    gl_FragColor = c.bgra;           \n"  // ← 关键：BGR → RGB
        "}";

    shader_.addShaderFromSourceCode(Vertex, vert_src);
    shader_.addShaderFromSourceCode(Fragment, frag_src);
    shader_.link();

    // ===== 创建纹理 =====
    glGenTextures(1, &tex_id_);
    glBindTexture(GL_TEXTURE_2D, tex_id_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    // ===== 全屏 Quad VBO =====
    quad_vbo_.create();
    const float initial_quad[] = { -1,-1,  1,-1,  1,1,  -1,1 };
    quad_vbo_.allocate(initial_quad, sizeof(initial_quad));

    // 背景色 #1a1a1a — 与安装包界面风格一致
    glClearColor(0.102f, 0.102f, 0.102f, 1.0f);
}
```

### 2.3 纹理上传优化

```cpp
void GLVideoWidget::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT);

    // === 纹理上传 ===
    shader_.bind();
    glBindTexture(GL_TEXTURE_2D, tex_id_);

    // BGR 3 字节宽度可能不被 4 整除 → 设置对齐为 1 字节
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    if (tex_w_ != w || tex_h_ != h) {
        // 尺寸变化 → 重新分配纹理存储 (glTexImage2D)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0,
                     GL_RGB, GL_UNSIGNED_BYTE, frame_data_.data());
        tex_w_ = w;
        tex_h_ = h;
    } else {
        // 尺寸不变 → DMA 更新子区域，避免重新分配
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h,
                        GL_RGB, GL_UNSIGNED_BYTE, frame_data_.data());
    }

    // === 保持宽高比的 Quad 顶点 ===
    const float frame_aspect  = w / h;
    const float widget_aspect = width() / height();
    float qw, qh;
    if (frame_aspect > widget_aspect) {
        qw = 1.0f;  qh = widget_aspect / frame_aspect;
    } else {
        qw = frame_aspect / widget_aspect;  qh = 1.0f;
    }
    quad_vbo_.write(0, vertices, sizeof(vertices));

    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);  // 渲染 quad
}
```

**DMA 更新策略**：
- `glTexImage2D`: 分配新的 GPU 纹理存储 + 上传数据 → 只在尺寸变化时使用
- `glTexSubImage2D`: 仅更新纹理数据，不改变存储分配 → 每帧使用，最快的纹理更新路径

**宽高比保持**：计算使视频帧完整显示在 widget 内的最大 quad 尺寸，留黑边填充空白区域。

### 2.4 线程安全

```cpp
// setFrame() 由 Qt 主线程 (DisplayWindow::refresh()) 调用
void GLVideoWidget::setFrame(const cv::Mat &frame) {
    // 深拷贝数据到内部 buffer (GLVideoWidget 不持有原始帧引用)
    frame_data_.resize(frame.total() * frame.elemSize());
    std::memcpy(frame_data_.data(), frame.data, data_size);

    frame_width_  = frame.cols;
    frame_height_ = frame.rows;
    frame_channels_ = frame.channels();
    frame_updated_ = true;

    update();  // 触发 paintGL() (Qt 事件循环中异步调度)
}
```

**帧缓存所有权**：
- ROS2 subscriber 回调 (ROS2 线程) → `latest_frame_` (mutex 保护)
- Qt 定时器 (Qt 主线程) → `fetchData()` (加锁拷贝) → `setFrame()` (深拷贝到 GL widget buffer)
- `paintGL()` (Qt 渲染线程/主线程) → 从 `frame_data_` 读取

---

## 3. DisplayWindow — Qt 主窗口

[qt_display_node.cpp:208-536](../src/tensorrt_detect/src/nodes/qt_display_node.cpp)

### 3.1 布局结构

```
QMainWindow
  └── central QWidget
      └── QVBoxLayout
          ├── QHBoxLayout (Top Bar)
          │   ├── status_label_   "car=3.2ms armor=1.5ms ..."
          │   ├── outpost_label_  "前哨站: 存活" or "前哨站: 摧毁"
          │   └── tactics_label_  "战术: 敌方大攻 | 我方工程上岛"
          │
          ├── QHBoxLayout (Content, stretch=1)
          │   ├── GLVideoWidget (stretch=3) ← GPU 渲染
          │   └── QLabel map_label_ (stretch=1) ← 雷达地图
          │
          └── QHBoxLayout (Bottom Bar)
              ├── QPushButton "重新标定" (橙色 #e67e22)
              ├── QPushButton "重新框定 ROI" (紫色 #8e44ad)
              ├── QPushButton "蓝方视角/红方视角" (蓝色/红色切换，checkable)
              └── op_status_label_ (操作结果提示)
```

### 3.2 样式系统

```cpp
// 状态栏 (正常)
"color: #00ff88; background-color: #0d0d0d; font-size: 20px;"

// 状态栏 (威胁 — 敌方大攻 / 敌方上岛 / 敌方接近堡垒)
"color: #ff4444; background-color: #1a0505; font-size: 20px;"

// 状态栏 (有利 — 我方大攻 / 我方上岛)
"color: #44ff44; background-color: #051a05; font-size: 20px;"

// 前哨站存活
"color: #00ff88; background-color: #0d0d0d;"

// 前哨站摧毁
"color: #ff4444; background-color: #1a0505;"
```

所有样式使用 Qt Stylesheet (QSS)，字体 `Microsoft YaHei` (Windows) / `Consolas` (monospace fallback)。

### 3.3 交互功能

| 功能 | 触发方式 | 实现 |
|---|---|---|
| 重新标定 | 点击按钮 | `callServiceAsync("/calibration/start", "相机标定")` |
| 重新框定 ROI | 点击按钮 | `callServiceAsync("/roi_set/start", "ROI 框定")` |
| 阵营视角切换 | 点击 toggle 按钮 | `publishTeamFlip(checked)` → `/flip_team` topic |
| 暂停/恢复 | 空格键 | `setVideoPauseAsync(paused)` → `/video_node/set_pause` service |
| 关闭窗口 | X 按钮 / Alt+F4 | `close_cb_()` → `rclcpp::shutdown()` |

**防止重复操作**：`is_operation_running_` 原子标志，在标定/ROI 操作完成前阻止再次触发。

---

## 4. QtDisplayNode — ROS2 通信桥接

[qt_display_node.cpp:541-772](../src/tensorrt_detect/src/nodes/qt_display_node.cpp)

### 4.1 多线程模型

```
┌──────────────────┐     ┌──────────────────┐
│  Qt 主线程        │     │  ROS2 线程        │
│  · QApplication  │     │  · rclcpp::spin  │
│  · QTimer 30fps  │     │  · subscriptions │
│  · DisplayWindow │     │                    │
│  · GLVideoWidget │     │  Mutex protected   │
│  · paintGL       │     │    latest_frame_   │
│                  │◄───►│    latest_map_     │
│  fetchData()     │lock │    latest_timing_  │
│    ↓             │     │    latest_tactics_ │
│  updateVideo()   │     │                    │
│  updateMap()     │     │                    │
│  updateStatus()  │     │                    │
└──────────────────┘     └──────────────────┘
```

**线程安全设计**：
- 所有 ROS2 subscriber 回调在**ROS2 线程**执行，写入 `latest_*` 成员时持有 `mutex_`
- `fetchData()` 在**Qt 主线程**由 `QTimer` 驱动调用，加锁拷贝数据
- 服务调用 (如标定、暂停) 在独立的 `std::thread` 中执行，不阻塞任何主循环

### 4.2 fetchData — 数据获取

```cpp
void QtDisplayNode::fetchData(
    cv::Mat &frame, cv::Mat &map,
    PipelineTiming &timing,
    bool &outpost_alive,
    bool &engineer_on_island, bool &opponent_attack,
    bool &our_attack, bool &opponent_near_fortress,
    double &display_latency_ms)
{
    QMutexLocker lock(&mutex_);  // RAII 加锁

    frame = latest_frame_.clone();     // 深拷贝 (不能共享内存 — 跨线程)
    map = latest_map_.clone();
    timing = latest_timing_;           // 值类型，直接复制
    outpost_alive = latest_outpost_alive_;
    // ... 战术分析数据 ...
    display_latency_ms = latest_display_latency_ms_.load();
}
```

### 4.3 callServiceAsync — 异步服务调用

```cpp
void QtDisplayNode::callServiceAsync(
    const std::string &service_name,
    const std::string &operation_name)
{
    if (is_operation_running_.exchange(true)) {
        // 已在运行中被拒
        return;
    }

    // 在独立线程执行
    std::thread([this, service_name, operation_name]() {
        auto client = this->create_client<Trigger>(service_name);

        // 等待服务上线 (最多 5 秒)
        if (!client->wait_for_service(std::chrono::seconds(5))) {
            // 回到 Qt 主线程显示错误
            QMetaObject::invokeMethod(window_, [this, operation_name]() {
                window_->showOperationStatus(
                    QString::fromStdString(operation_name + " 失败: 服务未上线"),
                    false);
                window_->setCalibrateButtonsEnabled(true);
            });
            is_operation_running_ = false;
            return;
        }

        auto future = client->async_send_request(request);
        // 标定/框定可能需要 5 分钟 (用户手动点选)
        auto status = future.wait_for(std::chrono::seconds(300));

        auto result = future.get();
        // 回到 Qt 主线程更新 UI
        QMetaObject::invokeMethod(window_, [this, operation_name, result]() {
            window_->showOperationStatus(result.success ? ... : ...);
            window_->setCalibrateButtonsEnabled(true);
        });

        is_operation_running_ = false;
    }).detach();  // detach: 让线程在后台完成，不与 Qt 主线程 join
}
```

**Qt 跨线程 UI 更新**：`QMetaObject::invokeMethod(window_, ...)` 是 Qt 的安全跨线程调用机制，将 lambda 调度到目标对象的线程事件队列中执行。

---

## 5. 性能优化总结

### 5.1 视频渲染路径

| 操作 | QLabel 路径 | GLVideoWidget 路径 |
|---|---|---|
| BGR→RGB | CPU: `cv::cvtColor()` ~1ms | GPU: `.bgra` swizzle 零成本 |
| 缩放 | CPU: `QPixmap::scaled()` ~2ms | GPU: 纹理采样硬件双线性 |
| 内存拷贝 | 3 次全帧 (~18MB for 1080p) | 1 次全帧 memcpy + DMA |
| 纹理上传 | Qt 内部 (不可控) | `glTexSubImage2D` DMA (最小) |
| 总 CPU 开销 | ~4-6ms | ~1-2ms |

### 5.2 GLVideoWidget 优化技术

1. **OpenGL Compatibility Profile**：避免 Core Profile 要求的 VAO/VBO 绑定开销，使用兼容的 `glVertexAttribPointer` + `glDrawArrays`
2. **Double Buffer + VSync**：`setSwapInterval(1)` 开启垂直同步，避免画面撕裂
3. **No Depth/Stencil Buffer**：`setDepthBufferSize(0)` 禁用深度测试，节省显存和带宽
4. **GL_RGB 格式上传**：`glTexImage2D(..., GL_RGB, GL_UNSIGNED_BYTE, ...)` — 直接上传 BGR 数据为 "RGB"，shader 中 `.bgra` 交换回正确颜色
5. **glPixelStorei(GL_UNPACK_ALIGNMENT, 1)**：处理 BGR 3 字节宽度不被 4 整除的对齐问题

### 5.3 雷达地图绘制

雷达地图使用传统 `QLabel::setPixmap()` 路径，因为：
- 地图图像很小 (722×388)，2ms 的缩放开销可忽略
- 地图更新频率低 (同 pose_node 帧率)，不需要每帧全速刷新
- 简化实现，无需额外的 OpenGL widget

### 5.4 帧率控制

- Qt Timer 30 FPS (33ms interval)：刷新包括视频、地图、状态栏
- 视频实际帧率由 DetectNode 决定 (~60-100 FPS)，Qt 端取最新帧显示
- 如果 GPU 渲染因为 vsync 而慢于 30 FPS → Qt 会自然降帧，不会积压

---

## 6. Surface Format 配置

```cpp
// main() 中全局设置，在 QApplication 创建之前
QSurfaceFormat gl_fmt;
gl_fmt.setSwapBehavior(QSurfaceFormat::DoubleBuffer);  // 双缓冲 = 无撕裂
gl_fmt.setSwapInterval(1);        // vsync 开着
gl_fmt.setDepthBufferSize(0);     // 不需要深度缓冲
gl_fmt.setStencilBufferSize(0);   // 不需要模板缓冲
gl_fmt.setProfile(QSurfaceFormat::CompatibilityProfile); // 兼容旧 GLSL
QSurfaceFormat::setDefaultFormat(gl_fmt);
```

**为什么用 Compatibility Profile**：代码使用 `attribute vec2` + `varying vec2` 等 GLSL 1.20 语法，兼容性更好，驱动兼容性最佳，无需 GLSL 版本声明。

---

## 7. 独立运行模式 UI

[standalone_main.cpp](../src/tensorrt_detect/apps/standalone_main.cpp) 使用 OpenCV highgui 替代 Qt：

```
┌─────────────────────────────────────────────┐
│  OpenCV Window                              │
│  ┌──────────────┐  ┌──────────────────────┐ │
│  │              │  │                      │ │
│  │  Video Frame │  │  Radar Map           │ │
│  │  (带 BBox)    │  │  (OpenCV drawMap)   │ │
│  │              │  │                      │ │
│  └──────────────┘  └──────────────────────┘ │
│  FPS: 25.3  |  按键: q=退出, s=保存标定     │
└─────────────────────────────────────────────┘
```

用于在没有 ROS2 环境的场景下快速验证，视频 + 雷达地图通过 hconcat 水平拼接显示。

---

## 8. 调试与状态显示

### 8.1 状态栏格式

```
car=3.2 armor=1.5 cls=0.8 output=0.6 airplane=4.1 total=10.2 e2e=12.5 disp=1.8 fps=25.3
│      │         │      │         │           │         │        │        │
│      │         │      │         │           │         │        │        └─ 检测帧率
│      │         │      │         │           │         │        └─ Qt 显示延时
│      │         │      │         │           │         └─ 端到端延时(从图 像时间戳)
│      │         │      │         │           └─ 流水线总耗时
│      │         │      │         └─ 异步无人机检测耗时
│      │         │      └─ 前哨站检测耗时
│      │         └─ 分类耗时
│      └─ 装甲板检测耗时
└─ 车辆检测耗时
```

### 8.2 前哨站状态

| 状态 | 颜色 | 文字 |
|---|---|---|
| 存活 | 绿色 `#00ff88` | "前哨站: 存活" |
| 摧毁 | 红色 `#ff4444` | "前哨站: 摧毁" |

### 8.3 战术告警

| 条件 | 效果 |
|---|---|
| 敌方大攻 / 敌方上岛 / 敌方接近堡垒 | 红色背景，字体 `#ff4444` |
| 我方大攻 / 我方上岛 | 绿色背景，字体 `#44ff44` |
| 正常运行 | 默认绿色背景 |

战术标签内容示例：
```
"战术: 正常"
"我方工程上岛 | 敌方大攻"
"敌方工程上岛 | 敌方大攻 | 敌方接近堡垒!"
```
