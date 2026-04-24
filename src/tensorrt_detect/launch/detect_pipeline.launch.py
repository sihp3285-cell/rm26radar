from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 视频源节点：发布图像到 /image_raw
        Node(
            package='tensorrt_detect',
            executable='video_node',
            name='video_node',
            output='screen',
            parameters=[{
                'video_path': '/home/delphine/rm/car_project/test/005.mp4',
                'topic_name': '/image_raw',
                'fps': 30,
            }],
        ),

        # 检测节点：订阅 /image_raw，发布 /detected_image 和 /armor_detections
        Node(
            package='tensorrt_detect',
            executable='detect_node',
            name='detect_node',
            output='screen',
            parameters=[{
                'config_dir': '/home/delphine/rm/tensorrt10_detect/configs',
                'input_topic': '/image_raw',
                'output_topic': '/detected_image',
            }],
        ),

        # 显示节点：订阅 /detected_image 做可视化
        Node(
            package='tensorrt_detect',
            executable='display_node',
            name='display_node',
            output='screen',
            parameters=[{
                'topic': '/detected_image',
                'window_name': 'Detection View',
                'window_width': 1280,
                'window_height': 720,
            }],
        ),

        Node(
            package='tensorrt_detect',
            executable='pose_node',
            name='pose_node',
            output='screen',
            parameters=[{
                'config_dir': '/home/delphine/rm/tensorrt10_detect/configs',
                'input_topic': '/armor_detections',
                'output_topic': '/world_targets',
            }],
        ),
    ])
