#!/usr/bin/env python3
"""Projection and calibration reload with an analytic flat ground fixture; no detector/UI."""
from pathlib import Path
import os
import signal
import subprocess
import tempfile
import time

os.environ.setdefault('ROS_DOMAIN_ID', str(100 + os.getpid() % 80))
os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
import rclpy
import yaml
from ament_index_python.packages import get_package_prefix
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_srvs.srv import Trigger
from radar27_interfaces.msg import DetectionArray, DetectionBox, WorldMeasurementArray, CalibrationState


def run():
    with tempfile.TemporaryDirectory(prefix='radar27-projection-') as directory:
        root = Path(directory)
        os.environ['ROS_LOG_DIR'] = directory
        (root / 'camera.yaml').write_text(yaml.safe_dump(dict(
            cameraMatrix=[100., 0., 100., 0., 100., 100., 0., 0., 1.],
            distCoeffs=[0., 0., 0., 0., 0.], requirePointsNum=1,
            worldPoints=[[0., 0., 0.]], meshPath='')))
        calibration = dict(r=[1., 0., 0., 0., 1., 0., 0., 0., 1.], t=[0., -1., 0.])
        path = root / 'calib_result.yaml'
        path.write_text(yaml.safe_dump(calibration))
        executable = Path(get_package_prefix('radar27_localization')) / 'lib/radar27_localization/pose_node'
        with (root / 'pose.log').open('w+') as log:
            process = subprocess.Popen([str(executable), '--ros-args', '-p', f'config_dir:={directory}'], stdout=log, stderr=log)
            node = None
            try:
                rclpy.init()
                node = rclpy.create_node('projection_contract_test')
                qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
                durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
                received, states = [], []
                sub = node.create_subscription(WorldMeasurementArray, '/world_measurements', received.append, qos)
                state_sub = node.create_subscription(CalibrationState, '/calibration_state', states.append, durable)
                publisher = node.create_publisher(DetectionArray, '/armor_detections', qos)

                def until(predicate):
                    deadline = time.monotonic() + 10
                    while time.monotonic() < deadline:
                        assert process.poll() is None, 'localization exited'
                        rclpy.spin_once(node, timeout_sec=.03)
                        if predicate():
                            return
                    raise AssertionError('projection timeout')

                until(lambda: publisher.get_subscription_count() == 1 and states)
                msg = DetectionArray()
                msg.header.stamp.sec = 10
                msg.header.frame_id = 'camera_frame'
                det = DetectionBox(idx=2, armor_color=1, x=90, y=110, width=20, height=10,
                                   confidence=.9, class_conf=.9, class_margin=.8)
                msg.detections = [det]

                def sample(sec):
                    msg.header.stamp.sec = sec
                    publisher.publish(msg)
                    until(lambda: received and received[-1].header.stamp.sec == sec)
                    return received[-1]

                out = sample(10)
                measurement = out.measurements[0]
                assert out.header.frame_id == 'world' and measurement.valid
                # Ray at (100,120) from origin (0,-1,0) hits y=0 at world (0,5).
                assert abs(measurement.world_x) < 1e-4 and abs(measurement.world_z - 5.) < 1e-3
                assert measurement.covariance_valid
                epoch = out.calibration_version
                assert states[-1].version == epoch
                reload = node.create_client(Trigger, '/pose_node/reload_calibration')
                assert reload.wait_for_service(timeout_sec=5)
                calibration['t'][0] = 1.
                path.write_text(yaml.safe_dump(calibration))
                future = reload.call_async(Trigger.Request())
                until(future.done)
                assert future.result().success
                out = sample(11)
                assert out.calibration_version > epoch and abs(out.measurements[0].world_x - 1.) < 1e-4
                new_epoch = out.calibration_version
                path.write_text('r: [1]\nt: [0, 0, 0]\n')
                future = reload.call_async(Trigger.Request())
                until(future.done)
                assert not future.result().success
                out = sample(12)
                assert out.calibration_version == new_epoch and abs(out.measurements[0].world_x - 1.) < 1e-4
                msg.detections = []
                out = sample(13)
                assert not out.measurements, 'empty frames must propagate to tracker'
                print('PASS: analytic projection/covariance, world frame, calibration snapshot/epoch, failed reload retention and empty frames')
            except BaseException:
                log.flush()
                log.seek(0)
                print(log.read()[-6000:])
                raise
            finally:
                if node is not None:
                    node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()


if __name__ == '__main__':
    run()
