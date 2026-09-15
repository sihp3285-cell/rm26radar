"""Expendable ROS2 shadow process. Its only outputs are suggestion JSON and RViz markers."""
import json
import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String, Bool
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from tensorrt_detect_msgs.msg import WorldTargetArray
from .runtime import ShadowRuntime, HistoryBuffer, UnavailableHistory

# rm_field/robot_class.hpp is the radar semantic contract. Never use slot/track IDs as robot IDs.
CLASS_TO_TYPE = {2:1,3:2,4:3,5:4,8:6,6:7}


class NeuralSentryNode(Node):
    def __init__(self):
        super().__init__('neural_sentry_shadow')
        defaults = {'enabled':True, 'bundle_dir':'', 'world_targets_topic':'/world_targets',
            'suggestion_topic':'/neural_sentry/shadow/suggestion',
            'marker_topic':'/neural_sentry/shadow/markers', 'world_frame':'world',
            'initial_flip_team':False, 'inference_hz':2.0, 'input_timeout_s':0.75,
            'display_ttl_s':1.5, 'inference_timeout_s':0.15, 'max_consecutive_errors':3,
            'minimum_identity_confidence':0.5, 'snapshot_tolerance_s':0.25}
        for key,value in defaults.items():
            self.declare_parameter(key,value)
        self.p = {key:self.get_parameter(key).value for key in defaults}
        for key in ('suggestion_topic','marker_topic'):
            if not self.p[key].startswith('/neural_sentry/shadow/'):
                raise ValueError('Shadow outputs must remain under /neural_sentry/shadow/')
        for key in ('inference_hz','input_timeout_s','display_ttl_s','inference_timeout_s'):
            if not math.isfinite(self.p[key]) or self.p[key] <= 0:
                raise ValueError(f'Invalid {key}')
        if self.p['inference_hz'] > 5 or self.p['display_ttl_s'] > 2:
            raise ValueError('Shadow rate/TTL exceeds bounded display budget')
        if not 0 < self.p['max_consecutive_errors'] <= 10:
            raise ValueError('Invalid error budget')
        self.qos = QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
        self.suggestion_pub = self.create_publisher(String,self.p['suggestion_topic'],self.qos)
        self.marker_pub = self.create_publisher(MarkerArray,self.p['marker_topic'],self.qos)
        self.flip = self.p['initial_flip_team']
        self.runtime = None
        self.history = None
        self.last_input = -math.inf
        self.last_inferred = -math.inf
        self.errors = 0
        self.disabled_reason = 'Disabled by configuration'
        self.last_status = None
        if self.p['enabled']:
            try:
                self.runtime = ShadowRuntime(self.p['bundle_dir'],self.p['inference_timeout_s'])
                self.history = HistoryBuffer(self.runtime.cfg,self.p['snapshot_tolerance_s'])
                self.get_logger().info('Shadow ONNX ready (CPU, one inference thread); no command publishers.')
            except Exception as exc:
                self.disabled_reason = f'Model unavailable: {type(exc).__name__}: {exc}'
        self.create_subscription(WorldTargetArray,self.p['world_targets_topic'],self.on_targets,self.qos)
        self.create_subscription(Bool,'/flip_team',self.on_flip,QoSProfile(depth=1))
        self.create_timer(1/self.p['inference_hz'],self.tick)

    def on_flip(self,msg):
        if self.flip != msg.data:
            self.flip = msg.data
            if self.history:
                self.history.clear()
            self.last_input = self.last_inferred = -math.inf
            self.publish_invalid('Team changed; rebuilding history')

    def on_targets(self,msg):
        if self.runtime is None:
            return
        try:
            if len(msg.targets) > 64:
                raise ValueError('Oversize target snapshot')
            # PoseNode copies the image header (video_frame/camera frame_id), but these
            # message fields are explicitly world x/z; do not interpret that header as TF.
            positions = np.full((12,2),np.nan,np.float32)
            observed = np.zeros(12,bool)
            seen,duplicates = set(),set()
            for target in msg.targets:
                # Explicit visual allowlist: no HP/death/referee/events, no prior or fused targets.
                if (not target.valid or not target.observed or target.position_source not in (1,2) or
                    not math.isfinite(target.stable_class_conf) or
                    target.stable_class_conf < self.p['minimum_identity_confidence'] or
                    target.team_id not in (1,2) or target.stable_class_id not in CLASS_TO_TYPE):
                    continue
                robot = CLASS_TO_TYPE[target.stable_class_id] + (100 if target.team_id == 2 else 0)
                idx = int(np.flatnonzero(self.runtime.robot_ids == robot)[0])
                if idx in seen:
                    duplicates.add(idx)
                seen.add(idx)
                xy = np.asarray([target.world_z+14,target.world_x+7.5],np.float32)
                if not np.isfinite(xy).all() or not np.all((xy >= 0)&(xy <= [28,15])):
                    continue
                if self.runtime.cfg['data']['zero_is_missing'] and np.all(xy == 0):
                    continue
                positions[idx],observed[idx] = xy,True
            for idx in duplicates:
                positions[idx],observed[idx] = np.nan,False
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            previous = self.history.frames[-1][0] if self.history.frames else -math.inf
            if self.history.append(stamp,positions,observed):
                self.last_input = time.monotonic()
                if stamp < previous:
                    self.last_inferred = -math.inf
                    self.publish_invalid('Source clock reset; rebuilding history')
        except Exception as exc:
            self.history.clear()
            self.publish_invalid(f'Visual input rejected: {exc}')

    def publish_invalid(self,reason):
        payload = {'version':1,'shadow_only':True,'valid':False,'reason':str(reason)[:180]}
        self.suggestion_pub.publish(String(data=json.dumps(payload)))
        clear = Marker(); clear.action = Marker.DELETEALL
        self.marker_pub.publish(MarkerArray(markers=[clear]))
        if self.last_status != payload['reason']:
            self.get_logger().info(payload['reason'])
            self.last_status = payload['reason']

    def tick(self):
        try:
            self._tick()
        except Exception as exc:
            self.errors += 1
            reason = f'Shadow inference error {self.errors}: {type(exc).__name__}: {exc}'
            if self.errors >= self.p['max_consecutive_errors']:
                self.runtime = None
                self.disabled_reason = reason+'; circuit open, restart shadow node to retry'
            self.publish_invalid(reason)

    def _tick(self):
        if self.runtime is None:
            self.publish_invalid(self.disabled_reason)
            return
        if time.monotonic()-self.last_input > self.p['input_timeout_s']:
            self.publish_invalid('Visual input stale / replay paused')
            return
        try:
            stamp,positions,observed = self.history.sample()
        except ValueError as exc:
            self.publish_invalid(str(exc))
            return
        if stamp <= self.last_inferred:
            return
        self.last_inferred = stamp
        ego_id = 7 if self.flip else 107
        ego_index = int(np.flatnonzero(self.runtime.robot_ids == ego_id)[0])
        # Input availability is not a model failure and must not exhaust the error budget.
        if not observed[-1,ego_index] or observed[:,ego_index].mean() < self.runtime.cfg['data']['min_history_coverage']:
            self.publish_invalid('Own sentry missing / insufficient visual history')
            return
        if self.runtime.candidates.component(positions[-1,ego_index]) < 0:
            self.publish_invalid('Own sentry outside traversable candidate component')
            return
        try:
            result = self.runtime.predict(positions,observed,ego_index)
        except UnavailableHistory as exc:
            self.publish_invalid(str(exc))
            return
        if time.monotonic()-self.last_input > self.p['input_timeout_s']:
            self.publish_invalid('Input expired during inference')
            return
        self.errors = 0
        result.update(version=1,shadow_only=True,valid=True,source_stamp=stamp,
                      team_id=1 if self.flip else 2,robot_id=ego_id,flip_team=self.flip,
                      ttl_s=self.p['display_ttl_s'],reason='BC suggestion; no actuation')
        self.suggestion_pub.publish(String(data=json.dumps(result,allow_nan=False)))
        self.publish_markers(result)
        if self.last_status != 'running':
            self.get_logger().info(f'Shadow suggestions active for robot {ego_id}')
            self.last_status = 'running'

    def publish_markers(self,result):
        markers = []
        for i,kind in enumerate((Marker.SPHERE,Marker.LINE_STRIP,Marker.TEXT_VIEW_FACING)):
            m = Marker()
            m.header.frame_id = self.p['world_frame']
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'neural_sentry/shadow'; m.id = i; m.type = kind; m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.8; m.color.a = 1.0
            ttl = self.p['display_ttl_s']
            m.lifetime.sec = int(ttl); m.lifetime.nanosec = int((ttl-int(ttl))*1e9)
            m.pose.position = Point(x=result['world_x'],y=0.35,z=result['world_z'])
            if kind == Marker.SPHERE:
                m.scale.x = m.scale.y = m.scale.z = 0.45
            elif kind == Marker.LINE_STRIP:
                m.pose.position = Point()
                m.scale.x = 0.07
                m.points = [Point(x=result['ego_world_x'],y=0.3,z=result['ego_world_z']),
                            Point(x=result['world_x'],y=0.3,z=result['world_z'])]
            else:
                m.pose.position.y = 1.0; m.scale.z = 0.3
                m.text = (f"SHADOW S{result['robot_id']} {result['action']} {result['candidate_id']}\n"
                          f"field ({result['field_x']:.2f}, {result['field_y']:.2f}) m\n"
                          f"P(move)={result['move_probability']:.2f}; BC only")
            markers.append(m)
        self.marker_pub.publish(MarkerArray(markers=markers))


def main():
    rclpy.init()
    node = None
    try:
        node = NeuralSentryNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
