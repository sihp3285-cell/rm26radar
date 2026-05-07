from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 统一参数文件路径：从 tensorrt_detect 包的 config 目录加载
    params_file = PathJoinSubstitution([
        FindPackageShare('tensorrt_detect'),
        'config',
        'ros2_params.yaml',
    ])

    return LaunchDescription([
        # 视频源节点：发布图像到 /image_raw
        Node(
            package='tensorrt_detect',
            executable='video_node',
            name='video_node',
            output='screen',
            parameters=[params_file],
        ),

        # 检测节点：订阅 /image_raw，发布 /detected_image 和 /armor_detections
        Node(
            package='tensorrt_detect',
            executable='detect_node',
            name='detect_node',
            output='screen',
            parameters=[params_file],
        ),

        # 位姿解算节点：将检测结果转换到世界坐标
        Node(
            package='tensorrt_detect',
            executable='pose_node',
            name='pose_node',
            output='screen',
            parameters=[params_file],
        ),

        # 小地图节点：绘制雷达地图
        Node(
            package='tensorrt_detect',
            executable='map_node',
            name='map_node',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='tensorrt_detect',
            executable='calibrate_node',
            name='calibrate_node',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='tensorrt_detect',
            executable='qt_display_node',
            name='qt_display_node',
            output='screen',
            parameters=[params_file],
        ),

        # ROI 设置节点：自动检测 outpost_roi 是否为空，为空则自动进入框定
        Node(
            package='tensorrt_detect',
            executable='roi_set_node',
            name='roi_set_node',
            output='screen',
            parameters=[params_file],
        )
    ])
