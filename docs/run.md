# ROS2 运行指南

本文档说明如何构建和运行 ROS2 版本的视觉检测系统。

---

## 一、环境准备

确保已安装并 source ROS2 环境：

```bash
source /opt/ros/jazzy/setup.bash
```

> 根据你的 ROS2 发行版，路径可能是 `/opt/ros/humble/setup.bash` 等。

---

## 二、构建项目

在项目根目录执行：

```bash
cd /home/delphine/rm/tensorrt10_detect

# source ROS2 环境
source /opt/ros/jazzy/setup.bash

# 构建（包含消息包 + 检测包）
colcon build --packages-select tensorrt_detect_msgs tensorrt_detect

# source 构建结果
source install/setup.bash
```

> 每次修改代码或参数文件后，都需要重新 `colcon build` 并 `source install/setup.bash`，install 目录才会同步最新内容。

---

## 三、启动方式（推荐：Launch 文件）

项目提供了一键启动的 launch 文件，会同时拉起全部 5 个节点：

```bash
cd /home/delphine/rm/tensorrt10_detect
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch tensorrt_detect detect_pipeline.launch.py
```

按 `Q` 或 `ESC` 可退出 `display_node` 的显示窗口，`launch` 会随之停止所有节点。

---

## 四、Launch 文件与参数配置

`detect_pipeline.launch.py` 使用 `FindPackageShare` 自动定位参数文件，不依赖绝对路径：

```python
params_file = PathJoinSubstitution([
    FindPackageShare('tensorrt_detect'),
    'config',
    'ros2_params.yaml',
])
```

这意味着无论你把项目放在哪个目录，launch 文件都能正确找到参数配置。

所有节点的参数集中管理在：

```
src/tensorrt_detect/config/ros2_params.yaml
```

修改参数后**必须**重新 `colcon build`，因为 launch 运行时读取的是 `install/` 目录下的副本。

常用参数一览：

| 节点 | 参数 | 默认值 | 说明 |
|------|------|--------|------|
| video_node | `video_path` | `".../005.mp4"` | 视频文件路径 |
| video_node | `fps` | `30` | 发布帧率 |
| detect_node | `publish_debug_image` | `true` | 是否发布带框调试图像 |
| detect_node | `debug_output_max_width` | `1280` | 调试图像最大宽度 |
| display_node | `window_width` | `1920` | 显示窗口宽度 |
| display_node | `window_height` | `720` | 显示窗口高度 |

---

## 五、手动逐个启动节点

如果不使用 launch，也可以手动启动各节点（需要多个终端）：

**终端 1 —— 视频源：**
```bash
cd /home/delphine/rm/tensorrt10_detect
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run tensorrt_detect video_node
```

**终端 2 —— 检测：**
```bash
cd /home/delphine/rm/tensorrt10_detect
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run tensorrt_detect detect_node
```

**终端 3 —— 位姿解算：**
```bash
cd /home/delphine/rm/tensorrt10_detect
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run tensorrt_detect pose_node
```

**终端 4 —— 小地图：**
```bash
cd /home/delphine/rm/tensorrt10_detect
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run tensorrt_detect map_node
```

**终端 5 —— 显示：**
```bash
cd /home/delphine/rm/tensorrt10_detect
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run tensorrt_detect display_node
```

> 手动启动时，话题名必须和 launch 文件中的契约一致，节点才能互相发现。

---

## 六、常用调试命令

```bash
# 查看活跃节点
ros2 node list

# 查看话题列表
ros2 topic list

# 查看某个话题的消息内容
ros2 topic echo /armor_detections
ros2 topic echo /world_targets

# 查看话题带宽和发布频率
ros2 topic bw /image_raw
ros2 topic hz /armor_detections

# 查看某个节点的参数列表
ros2 param list /detect_node

# 动态修改参数（仅对支持运行时回调的参数有效）
ros2 param set /detect_node publish_debug_image false

# 录制所有话题（用于离线回放调试）
ros2 bag record -a

# 回放录制数据（不需要 video_node）
ros2 bag play <bag_name>

# 用 rviz2 查看图像流
rviz2
```

---

## 七、Standalone 模式

项目同时保留了非 ROS2 的单进程入口，适合快速验证算法本身：

```bash
# 方法 1：从 install 目录运行
./install/tensorrt_detect/lib/tensorrt_detect/standard

# 方法 2：从 build 目录运行
./build/tensorrt_detect/standard
```

Standalone 模式不依赖 ROS2，所有模块在一个进程内通过函数调用串联，调试和 profile 更直接。

| 模式 | 适用场景 |
|------|---------|
| ROS2 Launch | 正式部署、多机协同、长期运行 |
| 手动逐个启动 | 单独调试某个节点、排查问题 |
| Standalone | 算法验证、性能 profile、无 ROS2 环境 |
