#!/usr/bin/env python3
"""Real tools + offscreen HighGUI; synthetic clicks, isolated files and ROS domain.

Mocks image source, video pause and reload responders. Does not validate the live
GPU consumers or desktop mouse handling. Requires g++, OpenCV development files.
"""
import importlib.util
import os
from pathlib import Path
import shlex
import signal
import subprocess
import tempfile
import time

os.environ.setdefault('ROS_DOMAIN_ID', str(100 + os.getpid() % 80))
os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
import rclpy
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, SetBool
from launch import LaunchContext
from launch.utilities import perform_substitutions


def run():
    with tempfile.TemporaryDirectory(prefix='radar27-tools-') as directory:
        root = Path(directory)
        os.environ['ROS_LOG_DIR'] = directory
        shim = root / 'gui_input.so'
        flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs', 'opencv4'], text=True))
        subprocess.run(['g++', '-shared', '-fPIC', str(Path(__file__).parent / 'fixtures/tools_gui_input.cpp'), '-o', str(shim), *flags, '-ldl'], check=True)
        (root / 'camera.yaml').write_text(yaml.safe_dump(dict(
            cameraMatrix=[100., 0., 100., 0., 100., 100., 0., 0., 1.],
            distCoeffs=[0.] * 5, requirePointsNum=6,
            worldPoints=[[0., 0., 0.], [1., 0., 0.], [0., 1., 0.],
                         [1., 1., 0.], [0., 0., 5.], [1., 1., 5.]])))
        calib = root / 'calib_result.yaml'
        calib.write_text(yaml.safe_dump(dict(r=[1.,0.,0.,0.,1.,0.,0.,0.,1.], t=[0.,0.,-5.])))
        roi = root / 'outpost_roi.yaml'
        roi.write_text(yaml.safe_dump(dict(outpost_roi=[1,2,3,4], outpost_enabled=True)))
        controls = root / 'input.txt'
        # Derive GUI overrides from the installed deployment, not a test-only fix.
        launch_path = Path(get_package_share_directory('radar27_bringup')) / 'launch/detect_pipeline.launch.py'
        spec = importlib.util.spec_from_file_location('pipeline', launch_path)
        launch = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(launch)
        ctx = LaunchContext()
        ctx.launch_configurations.update(dict(mode='camera', video_path='', model_dir='',
            config_dir=str(root), assets_dir='', runtime_dir=str(root / 'runtime'),
            own_team='blue', world_z_toward_blue='true', enable_qt_display='false',
            enable_tools='true', rviz_debug_enabled='false', enable_rviz='false'))
        (root / 'model.yaml').write_text(yaml.safe_dump(dict(modelPath='test.engine', armorModelPath='test.engine', classifyModelPath='test.engine')))
        (root / 'test.engine').write_bytes(b'test fixture; never deserialized')
        (root / 'map.yaml').write_text(yaml.safe_dump(dict(map_size=[640,480], race_size=[28.,15.])))
        actions = launch.setup(ctx)
        gui_envs = [{perform_substitutions(ctx, k): perform_substitutions(ctx, v)
                     for k, v in action.additional_env} for action in actions[-2:]]
        processes, logs = [], []
        rclpy.init()
        node = rclpy.create_node('tools_service_test')
        pauses, reloads = [], []
        def pause(req, res):
            pauses.append(req.data)
            res.success, res.message = True, 'paused' if req.data else 'playing'
            return res
        def reload(name):
            def respond(req, res):
                reloads.append(name)
                res.success, res.message = True, 'test reload accepted'
                return res
            return respond
        services = [node.create_service(SetBool, '/video_node/set_pause', pause),
                    node.create_service(Trigger, '/pose_node/reload_calibration', reload('pose')),
                    node.create_service(Trigger, '/detect_node/reload_roi', reload('detect'))]
        pub = node.create_publisher(Image, '/image_raw', 1)
        frame = Image(height=240, width=320, encoding='bgr8', step=960, data=bytes(320*240*3))
        timer = node.create_timer(.05, lambda: pub.publish(frame))
        try:
            for name, override in zip(('calibrate_node', 'roi_set_node'), gui_envs):
                assert override == {'GTK_PATH':'', 'LOCPATH':'', 'QT_ACCESSIBILITY':'0'}
                env = dict(os.environ, GTK_PATH='/snap/code/current/usr/lib/x86_64-linux-gnu/gtk-3.0',
                           LOCPATH='/snap/core20/current/usr/lib/locale', QT_ACCESSIBILITY='1',
                           QT_QPA_PLATFORM='offscreen', LD_PRELOAD=str(shim), RADAR27_TEST_INPUT=str(controls))
                env.update(override)
                executable = Path(get_package_prefix('radar27_tools')) / 'lib/radar27_tools' / name
                log = (root / (name + '.log')).open('w+')
                logs.append(log)
                args = [str(executable), '--ros-args', '-p', f'config_dir:={root}',
                        '-p', f'calib_result_path:={calib}', '-p', f'calibration_path:={calib}',
                        '-p', f'outpost_roi_path:={roi}', '-p', 'auto_calibrate:=false', '-p', 'auto_set_roi:=false']
                processes.append(subprocess.Popen(args, env=env, stdout=log, stderr=log))
            def invoke(service, content, success, message):
                controls.write_text(content)
                client = node.create_client(Trigger, service)
                assert client.wait_for_service(timeout_sec=10), service
                before = len(pauses)
                future = client.call_async(Trigger.Request())
                deadline = time.monotonic() + 20
                while not future.done() and time.monotonic() < deadline:
                    assert all(p.poll() is None for p in processes), 'tool process exited'
                    rclpy.spin_once(node, timeout_sec=.02)
                assert future.done(), f'{service} timeout'
                result = future.result()
                assert result.success == success and message in result.message, result
                assert pauses[before:] == [True, False], pauses[before:]
                node.destroy_client(client)
                print('PASS:', service, message, flush=True)
            for service in ('/calibration/start', '/roi_set/start'):
                before = (calib.read_bytes(), roi.read_bytes())
                invoke(service, 'cancel\n', False, '取消')
                invoke(service, 'throw\n', False, 'injected GUI error')
                invoke(service, 'cancel\n', False, '取消')
                assert before == (calib.read_bytes(), roi.read_bytes())
            invoke('/calibration/start', '\n'.join(f'click {x} {y}' for x,y in [(100,100),(120,100),(100,120),(120,120),(100,100),(110,110)]), True, '标定成功')
            saved = yaml.safe_load(calib.read_text())
            assert all(abs(a-b)<1e-4 for a,b in zip(saved['t'], [0,0,-5])), saved
            for _ in range(2):
                invoke('/roi_set/start', 'click 20 30\nclick 80 90\n', True, 'ROI 框定成功')
                assert yaml.safe_load(roi.read_text())['outpost_roi'] == [20,30,60,60]
            assert reloads == ['pose', 'detect', 'detect'], reloads
            assert not list(root.glob('*.tmp'))
            print('PASS: saved geometry/ROI, reload requests, repeat calls, video resume')
        finally:
            for p in processes:
                if p.poll() is None: p.send_signal(signal.SIGINT)
                try: p.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    p.kill()
                    p.wait()
            node.destroy_node()
            rclpy.shutdown()
            for log in logs:
                log.seek(0)
                print(log.read())
                log.close()


if __name__ == '__main__':
    run()
