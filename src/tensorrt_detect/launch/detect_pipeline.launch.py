# RoboMaster 雷达站的 ROS2 总启动入口。
#
# 运行时数据链（箭头表示 Topic，而不是 launch 的启动先后关系）：
#   CameraNode/VideoNode --/image_raw--> DetectNode
#   DetectNode --/armor_detections--> PoseNode --/world_targets--> MapNode
#                                       |              |
#                                       +--------------+--> PositionPriorNode
#   PositionPriorNode --/prior_predictions--> MapNode --/map_image--> QtDisplayNode
#
# 标定与 ROI 节点也订阅 /image_raw，但属于旁路交互工具，不阻塞主链。launch
# 没有为上述节点建立“某节点初始化完成后再启动下一节点”的时序依赖；ROS2 Topic
# 允许发布者和订阅者按任意顺序完成发现。主链四个 component 放进同一 container，
# 是为了允许 rclcpp 在发布 unique_ptr 消息时使用 intra-process 通道，减少大图像在
# 进程内的序列化/反序列化。Qt、OpenCV 交互窗口和先验节点保留独立进程，各自拥有
# 事件循环或故障边界。

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

    # FindPackageShare 从安装空间定位 package share 目录；因此正常使用前需要先
    # colcon build/source install/setup.bash。PathJoinSubstitution 延迟到 launch
    # 展开路径，避免把源码树绝对路径写进启动脚本。
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

    # ── 根据 mode 选择唯一图像源节点 ──
    # 两个源统一发布 /image_raw，使下游无需知道帧来自文件还是工业相机。
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

    # ── 主链组件容器 ──
    # component_container 动态加载 CMake 注册的 plugin 共享库。这里启用
    # use_intra_process_comms 只是允许进程内优化；cv_bridge::toImageMsg 等显式
    # 图像转换仍会发生真实拷贝，不能把整个链路理解为“绝对零拷贝”。
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

        # 独立 shadow 节点：只发布先验消息与日志，不回灌 tracker。这样统计先验
        # 的收益或失败不会改变 /world_targets 中 Kalman/Tracker 的基线状态。
        Node(
            package='position_prior',
            executable='position_prior_node',
            name='position_prior_node',
            output='screen',
            parameters=[prior_params_file],
        ),

        # 标定节点（独立进程，含交互式 OpenCV 窗口）。它与主链并行订阅原图，
        # 写入外参后由 PoseNode 的 reload service 显式重载，而非隐式共享内存。
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

        # ROI 设置节点（独立进程，含交互式 OpenCV 窗口）。保存配置后通过
        # DetectNode service 重载，避免在交互期间重启 TensorRT 模型。
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
