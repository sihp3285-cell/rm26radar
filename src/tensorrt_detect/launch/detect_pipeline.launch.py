from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 统一参数文件路径：从 tensorrt_detect 包的 config 目录加载
    params_file = PathJoinSubstitution([
        FindPackageShare('tensorrt_detect'),
        'config',
        'ros2_params.yaml',
    ])

    # 进程内零拷贝容器：核心流水线节点全部跑在同一个进程里
    # 使用单线程容器，避免 TensorRT 与 Open3D RaycastingScene 并发 CUDA 操作导致 SIGSEGV
    pipeline_container = ComposableNodeContainer(
        name='detect_pipeline_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='tensorrt_detect',
                plugin='VideoNode',
                name='video_node',
                parameters=[params_file],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='tensorrt_detect',
                plugin='DetectNode',
                name='detect_node',
                parameters=[params_file],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='tensorrt_detect',
                plugin='PoseNode',
                name='pose_node',
                parameters=[params_file],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='tensorrt_detect',
                plugin='MapNode',
                name='map_node',
                parameters=[params_file],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        pipeline_container,

        # 标定节点（独立进程，含交互式 OpenCV 窗口）
        Node(
            package='tensorrt_detect',
            executable='calibrate_node',
            name='calibrate_node',
            output='screen',
            parameters=[params_file],
        ),

        # Qt 显示节点（独立进程，含 Qt 事件循环）
        Node(
            package='tensorrt_detect',
            executable='qt_display_node',
            name='qt_display_node',
            output='screen',
            parameters=[params_file],
        ),

        # ROI 设置节点（独立进程，含交互式 OpenCV 窗口）
        Node(
            package='tensorrt_detect',
            executable='roi_set_node',
            name='roi_set_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
