"""Compose package-owned components. Algorithms never import bringup.

Engines/video are external inputs. Calibration/ROI are copied once to runtime_dir;
installed defaults remain read-only. UI consumers run outside the real-time container.
"""
import os
import shutil
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def share(package):
    return Path(get_package_share_directory(package))


def require_input_file(path, label):
    path = Path(path).expanduser().resolve()
    try:
        if not path.is_file():
            raise ValueError(f'{label} 文件不存在或不是普通文件: {path}')
        with path.open('rb') as stream:
            if not stream.read(1):
                raise ValueError(f'{label} 文件为空: {path}')
    except OSError as exc:
        raise ValueError(f'{label} 文件无法读取: {path}: {exc}') from exc
    return str(path)


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    def flag(name):
        return value(name).lower() in ('true', '1', 'yes', 'on')
    mode = value('mode')
    if mode not in ('camera', 'video'):
        raise ValueError('mode must be camera or video')
    video_path = ''
    if mode == 'video':
        if not value('video_path'):
            raise ValueError('视频模式需要 video_path:=实际视频文件的绝对路径；示例路径不能直接使用')
        video_path = require_input_file(value('video_path'), 'video_path')
    root = share('radar27_bringup')
    config = Path(value('config_dir') or root / 'config' / 'default').resolve()
    assets = Path(value('assets_dir') or root / 'assets').resolve()
    model_dir = str(Path(value('model_dir')).expanduser().resolve()) if value('model_dir') else ''
    model_config = yaml.safe_load((config / 'model.yaml').read_text())
    for key in ('modelPath', 'armorModelPath', 'classifyModelPath', 'airplaneModelPath'):
        name = model_config.get(key, '')
        if not name:
            if key == 'airplaneModelPath':
                continue
            raise ValueError(f'model.yaml 中 {key} 不能为空')
        path = Path(model_dir) / Path(name).name if model_dir else config / name
        require_input_file(path, f'{key}（请检查 model_dir 或 model.yaml）')
    runtime = Path(value('runtime_dir')).expanduser().resolve()
    runtime.mkdir(parents=True, exist_ok=True)
    for filename in ('calib_result.yaml', 'outpost_roi.yaml'):
        if not (runtime / filename).exists():
            shutil.copy2(config / filename, runtime / filename)
    def params(package, node, **overrides):
        path = share(package) / 'config' / 'params.yaml'
        data = yaml.safe_load(path.read_text())[node]['ros__parameters'] or {}
        return [dict(data, **overrides)]
    def component(package, plugin, node, **overrides):
        return ComposableNode(package=package, plugin=plugin, name=node,
            parameters=params(package, node, **overrides),
            extra_arguments=[{'use_intra_process_comms': True}])
    red = value('own_team') == 'red'
    if value('own_team') not in ('red', 'blue'):
        raise ValueError('own_team must be red or blue')
    world_blue = flag('world_z_toward_blue')
    debug = flag('rviz_debug_enabled') or flag('enable_rviz')
    source = (component('radar27_input', 'VideoNode', 'video_node', video_path=video_path)
              if mode == 'video' else component('radar27_input', 'radar27_input::CameraNode', 'camera_node', record_path=str(runtime / 'recordings')))
    map_config = yaml.safe_load((config / 'map.yaml').read_text())
    width, height = map_config['map_size']
    nodes = [source,
        component('radar27_detection', 'DetectNode', 'detect_node', config_dir=str(config),
            model_dir=model_dir, roi_path=str(runtime / 'outpost_roi.yaml'), publish_debug_image=flag('enable_qt_display')),
        component('radar27_localization', 'PoseNode', 'pose_node', config_dir=str(config),
            calibration_path=str(runtime / 'calib_result.yaml'), rviz_debug_enabled=debug,
            gully_region_path=str(assets / 'generated' / 'gully.yaml'), gully_field_x_flip=not world_blue),
        component('radar27_tracking', 'TrackingNode', 'tracking_node', config_dir=str(config)),
        component('radar27_decision', 'DecisionNode', 'decision_node', own_team=1 if red else 2,
            world_z_toward_blue=world_blue, field_length=float(map_config['race_size'][0]),
            field_width=float(map_config['race_size'][1]), map_width=height, map_height=width)]
    actions = [ComposableNodeContainer(name='radar27_pipeline', namespace='', package='rclcpp_components',
        executable='component_container', composable_node_descriptions=nodes, output='screen'),
        Node(package='radar27_decision', executable='match_state_node', parameters=[{'own_team': 1 if red else 2}], output='screen'),
        Node(package='radar27_fusion', executable='fusion_node', parameters=params('radar27_fusion', 'fusion_node'), output='screen')]
    prior = yaml.safe_load((share('position_prior') / 'config' / 'position_prior.yaml').read_text())['position_prior_node']['ros__parameters']
    prior.update(model_path=str(assets / 'prior/run_v1/04_rmuc2026_position_prior_v1.yaml'),
        navgrid_path=str(assets / 'generated/RB2026_navgrid_v1.json'),
        common_blind_zone_paths=[str(assets / 'generated' / f) for f in ('home.yaml', 'gully.yaml')],
        engineer_blind_zone_path=str(assets / 'generated/engineer.yaml'),
        engineer_home_path=str(assets / 'generated/engineer_home.yaml'),
        other_home_path=str(assets / 'generated/other_home.yaml'),
        shadow_log_path=str(runtime / 'position_prior_shadow.csv'),
        initial_flip_team=red, world_z_toward_blue=world_blue)
    actions.append(Node(package='position_prior', executable='position_prior_node', parameters=[prior], output='screen'))
    if flag('enable_tools'):
        for name, extra in [('calibrate_node', {'calib_result_path': str(runtime / 'calib_result.yaml')}),
                            ('roi_set_node', {'outpost_roi_path': str(runtime / 'outpost_roi.yaml'), 'calibration_path': str(runtime / 'calib_result.yaml')})]:
            # OpenCV HighGUI also loads Qt/GTK plugins. Do not inherit Snap's
            # GTK modules (and their private glibc) from an IDE terminal.
            actions.append(Node(package='radar27_tools', executable=name, parameters=params('radar27_tools', name, config_dir=str(config), **extra), output='screen',
                additional_env={'GTK_PATH': '', 'LOCPATH': '', 'QT_ACCESSIBILITY': '0'}))
    if flag('enable_qt_display'):
        actions.append(Node(package='radar27_visualization', executable='map_node', parameters=params('radar27_visualization','map_node',config_dir=str(config),flip_team=red),output='screen'))
        actions.append(Node(package='radar27_visualization', executable='qt_display_node', parameters=params('radar27_visualization','qt_display_node'), output='screen',
            additional_env={'GTK_PATH': '', 'LOCPATH': '', 'QT_ACCESSIBILITY': '0'}))
    if debug:
        actions.append(Node(package='radar27_visualization', executable='rviz_debug_node', output='screen',
            parameters=params('radar27_visualization','rviz_debug_node', initial_flip_team=red,
                world_z_toward_blue=world_blue, mesh_path=str(config / 'RB2026_rmuc.ply'),
                navgrid_path=str(assets / 'generated/RB2026_navgrid_v1.json'),
                blind_zone_paths=[str(assets / 'generated' / f) for f in ('home.yaml','gully.yaml','engineer.yaml','engineer_home.yaml','other_home.yaml')])))
    if flag('enable_rviz'):
        actions.append(Node(package='rviz2', executable='rviz2', arguments=['-d',str(share('radar27_visualization') / 'config/radar_debug.rviz')],output='screen'))
    return actions


def generate_launch_description():
    defaults = dict(mode='video', video_path='', model_dir=os.environ.get('RADAR27_MODEL_DIR',''),
        config_dir='', assets_dir='', runtime_dir=os.environ.get('RADAR27_RUNTIME_DIR',str(Path.home()/'.local/state/radar27')),
        own_team='blue', world_z_toward_blue='true', enable_qt_display='true', enable_tools='true',
        rviz_debug_enabled='false', enable_rviz='false')
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()] + [OpaqueFunction(function=setup)])
