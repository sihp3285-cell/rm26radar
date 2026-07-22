from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """
    OpaqueFunction 回调：在 launch 时解析参数，根据 mode 选择
    加载 CameraNode 或 VideoNode 作为图像源。
    """
    mode = LaunchConfiguration('mode').perform(context)

    params_file = PathJoinSubstitution([
        FindPackageShare('tensorrt_detect'),
        'config',
        'ros2_params.yaml',
    ])
    prior_params_file = PathJoinSubstitution([
        FindPackageShare('position_prior'),
        'config',
        'position_prior.yaml',
    ])

    # ── 根据 mode 选择图像源节点 ──
    if mode == 'camera':
        source_composable = ComposableNode(
            package='tensorrt_detect',
            plugin='tensorrt_detect::CameraNode',
            name='camera_node',
            parameters=[params_file],
            extra_arguments=[{'use_intra_process_comms': True}],
        )
    else:  # 默认 video
        source_composable = ComposableNode(
            package='tensorrt_detect',
            plugin='VideoNode',
            name='video_node',
            parameters=[params_file],
            extra_arguments=[{'use_intra_process_comms': True}],
        )

    # ── 进程内零拷贝容器 ──
    pipeline_container = ComposableNodeContainer(
        name='detect_pipeline_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            source_composable,
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
        emulate_tty=True,
    )

    return [
        pipeline_container,

        # 独立 shadow 节点：只发布先验消息与日志，不回灌 tracker。
        Node(
            package='position_prior',
            executable='position_prior_node',
            name='position_prior_node',
            output='screen',
            parameters=[prior_params_file],
        ),

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
            # 从 Snap 版 VS Code 启动时，这些变量会让系统 Qt 误加载
            # /snap/core20 的旧 GTK/glibc 依赖，表现为 libpthread 符号错误。
            # 只隔离 Qt 子进程，不影响相机 SDK、TensorRT 或其他 ROS 节点。
            additional_env={
                'GTK_PATH': '',
                'LOCPATH': '',
                'QT_ACCESSIBILITY': '0',
            },
        ),

        # ROI 设置节点（独立进程，含交互式 OpenCV 窗口）
        Node(
            package='tensorrt_detect',
            executable='roi_set_node',
            name='roi_set_node',
            output='screen',
            parameters=[params_file],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='video',
            description="图像源模式: 'video' (视频文件) 或 'camera' (工业相机)"),
        OpaqueFunction(function=launch_setup),
    ])
