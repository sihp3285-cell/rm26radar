from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


def generate_launch_description():
    try:
        map_config = get_package_share_directory('radar27_bringup') + '/config/default/map.yaml'
    except PackageNotFoundError:
        map_config = ''
    params = {'host': ('127.0.0.1', str), 'port': ('8765', int),
              'map_config': (map_config, str),
              'radar_hz': ('5.0', float), 'performance_hz': ('1.0', float),
              'stale_after_s': ('2.0', float), 'world_z_toward_blue': ('true', bool),
              'field_length': ('28.0', float), 'field_width': ('15.0', float)}
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v[0]) for k, v in params.items()],
        Node(package='radar27_dashboard', executable='dashboard_node', output='screen',
             parameters=[{k: ParameterValue(LaunchConfiguration(k), value_type=v[1])
                          for k, v in params.items()}]),
    ])
