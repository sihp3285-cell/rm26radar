#!/usr/bin/python3
"""Replay exported visual histories in an isolated ROS domain; capture real ONNX suggestions.

Only run on an otherwise unused ROS_DOMAIN_ID (required, never domain 0).
Publishes WorldTargetArray for the replay, never radar commands or fused decisions.
"""
import argparse
import json
import os
from pathlib import Path
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Bool
from tensorrt_detect_msgs.msg import WorldTarget, WorldTargetArray


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bundle',required=True)
    ap.add_argument('--output',required=True)
    ap.add_argument('--case',type=int,default=0)
    args = ap.parse_args()
    if int(os.environ.get('ROS_DOMAIN_ID','0')) == 0:
        ap.error('Choose an unused nonzero ROS_DOMAIN_ID to isolate the replay from the radar.')
    manifest = json.loads((Path(args.bundle)/'manifest.json').read_text())
    arrays = np.load(Path(args.bundle)/'replay_inputs.npz',allow_pickle=False)
    n = args.case
    pos,obs = arrays[f'positions_{n}'],arrays[f'observed_{n}']
    ego = arrays[f'ego_index_{n}'].item()
    ids = manifest['runtime_config']['data']['robot_ids']
    dt = manifest['runtime_config']['data']['sample_period']
    rclpy.init(); node = Node('neural_sentry_isolated_replay')
    qos = QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    pub = node.create_publisher(WorldTargetArray,'/world_targets',qos)
    flip = node.create_publisher(Bool,'/flip_team',1)
    received = []
    node.create_subscription(String,'/neural_sentry/shadow/suggestion',
                             lambda msg: received.append(json.loads(msg.data)),qos)
    def spin_for(seconds):
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            rclpy.spin_once(node,timeout_sec=0.03)
    try:
        spin_for(1.5)
        flip.publish(Bool(data=ids[ego] < 100)); spin_for(0.5)
        type_to_class = {1:2,2:3,3:4,4:5,6:8,7:6}
        base = 1000.0
        # Real validation trajectories, accelerated source clock; no invented future input.
        for j in range(len(pos)):
            message = WorldTargetArray()
            message.header.frame_id = 'video_frame'  # PoseNode inherits this source header.
            stamp = base+j*dt
            message.header.stamp.sec = int(stamp)
            message.header.stamp.nanosec = int((stamp-int(stamp))*1e9)
            for k,rid in enumerate(ids):
                target = WorldTarget()
                target.team_id = 1 if rid < 100 else 2
                target.class_id = target.stable_class_id = type_to_class[rid%100]
                target.stable_class_conf = 1.0
                target.valid = target.observed = bool(obs[j,k])
                target.position_source = 1
                if target.observed:
                    target.world_x = float(pos[j,k,1]-7.5)
                    target.world_z = float(pos[j,k,0]-14)
                message.targets.append(target)
            pub.publish(message); spin_for(0.55)
        spin_for(1.8)  # Demonstrate input-timeout invalidation without sending more frames.
        valid = [s for s in received if s.get('valid')]
        expired = bool(valid) and any(not s.get('valid') and 'stale' in s.get('reason','') for s in received[received.index(valid[-1])+1:])
        report = {'ros_domain_id':int(os.environ['ROS_DOMAIN_ID']),'visual_replay_case':n,
                  'valid_suggestions':valid,'stale_input_cleared':expired,
                  'all_messages':received}
        Path(args.output).parent.mkdir(parents=True,exist_ok=True)
        Path(args.output).write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k!='all_messages'},indent=2))
        if not valid or not expired:
            raise RuntimeError('No valid inference or no stale-input invalidation; see report')
        if valid[-1]['candidate_index'] != arrays[f'predicted_index_{n}'].item():
            raise RuntimeError('Live ROS adapter decision differs from exported replay reference')
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
