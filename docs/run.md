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