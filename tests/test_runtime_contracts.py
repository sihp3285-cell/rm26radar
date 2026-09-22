#!/usr/bin/env python3
"""CPU-only process integration: measurement -> tracking -> decision/fusion and durable team state.
Run after sourcing the new install space. No camera, engine, mesh, Qt or display is opened.
"""
from pathlib import Path
import os
import signal
import subprocess
import tempfile
import time

os.environ.setdefault('ROS_DOMAIN_ID', str(100 + os.getpid() % 80))
os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from radar27_interfaces.msg import (
    WorldMeasurement, WorldMeasurementArray, WorldTargetArray, WorldTarget,
    RadarMap, FusedTargetArray, FusedTarget, MatchState,
)


def run():
    with tempfile.TemporaryDirectory(prefix='radar27-contracts-') as directory:
        os.environ['ROS_LOG_DIR'] = directory
        processes, logs = [], []
        node = None
        try:
            # A standalone tracker needs only its own configuration (defaults here).
            for package, executable, args in [
                ('radar27_tracking', 'tracking_node', ['--ros-args', '-p', f'config_dir:={directory}']),
                ('radar27_decision', 'decision_node', []),
                ('radar27_decision', 'match_state_node', []),
                ('radar27_fusion', 'fusion_node', []),
            ]:
                path = Path(get_package_prefix(package)) / 'lib' / package / executable
                log = open(Path(directory) / (executable + '.log'), 'w+')
                logs.append(log)
                processes.append(subprocess.Popen([str(path), *args], stdout=log, stderr=log))
            rclpy.init()
            node = rclpy.create_node('architecture_contract_test')
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            received = {'world': [], 'map': [], 'fused': [], 'team': []}
            subscriptions = [
                node.create_subscription(WorldTargetArray, '/world_targets', lambda m: received['world'].append(m), qos),
                node.create_subscription(RadarMap, '/radar_map', lambda m: received['map'].append(m), 10),
                node.create_subscription(FusedTargetArray, '/fused_targets', lambda m: received['fused'].append(m), qos),
            ]
            publisher = node.create_publisher(WorldMeasurementArray, '/world_measurements', qos)

            def until(predicate, timeout=8.0):
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    assert all(p.poll() is None for p in processes), 'a node exited'
                    rclpy.spin_once(node, timeout_sec=0.03)
                    if predicate():
                        return
                raise AssertionError('timed out waiting for runtime contract')

            until(lambda: publisher.get_subscription_count() == 1 and
                  node.count_publishers('/world_targets') == 1 and
                  node.count_subscribers('/world_targets') >= 3)

            def send(sec, nanosec=0, epoch=100, measurements=()):
                msg = WorldMeasurementArray()
                msg.header.stamp.sec, msg.header.stamp.nanosec = sec, nanosec
                msg.header.frame_id = 'test_world'
                msg.calibration_version = epoch
                msg.measurements = list(measurements)
                # Discovery can lag between graph notification and the first delivery.
                for _ in range(3):
                    publisher.publish(msg)
                    deadline = time.monotonic() + .3
                    while time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=.02)
                        if received['world'] and received['world'][-1].header.stamp == msg.header.stamp and received['world'][-1].calibration_version == epoch:
                            return received['world'][-1]
                raise AssertionError('measurement was not processed')

            m = WorldMeasurement()
            m.valid = True
            m.world_x, m.world_z = 1.0, 2.0
            m.covariance = [0.2, 0.05, 0.05, 0.3]
            m.covariance_valid = True
            d = m.detection
            d.idx, d.armor_color = 2, 1  # red hero -> slot 0
            d.x, d.y, d.width, d.height = 100, 100, 30, 20
            d.confidence, d.class_conf, d.class_margin = .99, .99, .95
            for frame in range(12):
                world = send(10, frame * 50_000_000, measurements=[m])
            t = world.targets[0]
            assert len(world.targets) == 11 and t.valid and t.observed
            assert t.class_id == 2 and t.team_id == 1 and t.track_id >= 0
            assert t.position_source == WorldTarget.POSITION_TRACKED
            assert t.measurement_covariance_valid and abs(t.measurement_covariance[1] - .05) < 1e-6
            assert world.header.frame_id == 'test_world' and world.calibration_version == 100
            until(lambda: received['map'] and received['map'][-1].header.stamp == world.header.stamp and
                  received['fused'] and received['fused'][-1].header.stamp == world.header.stamp)
            out = received['map'][-1]
            assert abs(out.red_x[0] - (t.world_x * 388 / 15 + 194)) < 1e-3
            assert abs(out.red_y[0] - (t.world_z * 722 / 28 + 361)) < 1e-3
            assert received['fused'][-1].targets[0].source == FusedTarget.SOURCE_TRACKED

            empty = send(11, epoch=101)
            assert not any(t.valid for t in empty.targets), 'calibration change retained old tracks'
            for frame in range(12):
                send(12, frame * 50_000_000, epoch=101, measurements=[m])
            rewind = send(9, epoch=101)
            assert not any(t.valid for t in rewind.targets), 'time rollback retained tracks'

            outpost = WorldMeasurement()
            outpost.valid = True
            outpost.detection.idx = 7
            outpost.detection.width = outpost.detection.height = 10
            special = send(14, epoch=101, measurements=[outpost])
            assert special.targets[10].valid and special.targets[10].class_id == 7
            negative = WorldMeasurement()
            negative.valid = negative.is_negative = True
            negative.detection.idx = 1
            negative.detection.is_dead = True
            negative.detection.width = negative.detection.height = 10
            special = send(14, 50_000_000, epoch=101, measurements=[negative])
            assert len(special.targets) == 12 and special.targets[11].is_dead
            assert not any(t.valid for t in special.targets[:10])

            client = node.create_client(SetBool, '/match/set_red_team')
            assert client.wait_for_service(timeout_sec=5)
            future = client.call_async(SetBool.Request(data=True))
            until(future.done)
            assert future.result().success
            # Subscriber created after the change must receive the latched current state.
            subscriptions.append(node.create_subscription(MatchState, '/match_state', lambda m: received['team'].append(m), durable))
            until(lambda: bool(received['team']))
            state = received['team'][-1]
            assert state.own_team == 1 and state.revision == 1
            display = node.create_publisher(Bool, '/display_flip', durable)
            display.publish(Bool(data=False))
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=.03)
            assert received['team'][-1] == state, 'display rotation changed own team'
            print('PASS: observation/covariance, headless business/fusion, epoch reset, time rollback, special targets, durable team state and independent display')
        except BaseException:
            for log in logs:
                log.flush()
                log.seek(0)
                print(log.read()[-5000:])
            raise
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            for process in processes:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
            for process in processes:
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            for log in logs:
                log.close()


if __name__ == '__main__':
    run()
