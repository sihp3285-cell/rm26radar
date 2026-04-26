终端 1：发布视频帧

cd /home/delphine/rm/tensorrt10_detect

source /opt/ros/jazzy/setup.bash

source install/setup.bash

ros2 run tensorrt_detect video_node


终端 2：检测节点


cd /home/delphine/rm/tensorrt10_detect

source /opt/ros/jazzy/setup.bash

source install/setup.bash

ros2 run tensorrt_detect detect_node

终端 3：显示节点

cd /home/delphine/rm/tensorrt10_detect

source /opt/ros/jazzy/setup.bash

source install/setup.bash

ros2 run tensorrt_detect display_node

## 性能模式构建（Release）
为避免部署时误用 Debug，建议显式使用 Release 构建：

``` sh
cd /home/delphine/rm/tensorrt10_detect
source /opt/ros/jazzy/setup.bash

colcon build --packages-select tensorrt_detect --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

> 说明：`src/tensorrt_detect/CMakeLists.txt` 已在未指定 `CMAKE_BUILD_TYPE` 时默认使用 `Release`。

