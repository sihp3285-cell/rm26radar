"""Add this process to an already running radar. No shutdown handlers or actuation."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bundle_dir',default_value='/home/delphine/rm/tensorrt10_detect/models/neural_sentry/sentry_v2'),
        DeclareLaunchArgument('initial_flip_team',default_value='false'),
        Node(package='neural_sentry_decision',executable='neural_sentry_node',
             name='neural_sentry_shadow',output='screen',respawn=False,
             parameters=[PathJoinSubstitution([FindPackageShare('neural_sentry_decision'),'config','shadow.yaml']),
                         {'bundle_dir':LaunchConfiguration('bundle_dir'),
                          'initial_flip_team':ParameterValue(LaunchConfiguration('initial_flip_team'),value_type=bool)}]),
    ])
