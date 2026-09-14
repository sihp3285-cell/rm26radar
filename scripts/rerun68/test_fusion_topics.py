#!/usr/bin/env python3
"""Standalone ROS topic integration test. Run explicitly in an isolated domain."""
import os,pathlib,signal,subprocess,time

def main():
 import rclpy
 from rclpy.qos import QoSProfile,ReliabilityPolicy
 from tensorrt_detect_msgs.msg import WorldTargetArray,WorldTarget,PriorPredictionArray,PriorPrediction,FusedTargetArray,FusedTarget
 root=pathlib.Path(__file__).resolve().parents[2]
 env=os.environ.copy()
 proc=subprocess.Popen([str(root/'install/tensorrt_detect/lib/tensorrt_detect/fusion_node')],env=env,stdout=subprocess.DEVNULL,start_new_session=True)
 rclpy.init();node=rclpy.create_node('fusion_topic_test');received=[]
 qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
 wp=node.create_publisher(WorldTargetArray,'/world_targets',qos);pp=node.create_publisher(PriorPredictionArray,'/prior_predictions',qos)
 sub=node.create_subscription(FusedTargetArray,'/fused_targets',received.append,qos)
 def wait_for(pred):
  deadline=time.monotonic()+5
  while time.monotonic()<deadline:
   rclpy.spin_once(node,timeout_sec=.03)
   if pred():return
  raise AssertionError('topic condition timed out')
 def output(sec,valid,source=None):
  return any(m.header.stamp.sec==sec and m.targets[1].valid==valid and (source is None or m.targets[1].source==source) for m in received)
 try:
  wait_for(lambda:wp.get_subscription_count()>0 and pp.get_subscription_count()>0 and node.count_publishers('/fused_targets')>0)
  w=WorldTargetArray();w.header.stamp.sec=100;w.targets=[WorldTarget() for _ in range(10)]
  t=w.targets[1];t.team_id=1;t.class_id=3;t.track_id=7;t.valid=True;t.observed=True;t.position_source=WorldTarget.POSITION_TRACKED;t.world_x=1.;t.world_z=2.;t.tracking_confidence=.9;t.last_observed_time.sec=80
  p=PriorPredictionArray();p.header.stamp.sec=100;p.model_enabled=True;q=PriorPrediction();q.slot_idx=1;q.team_id=1;q.role_class_id=3;q.track_id=7;q.valid=True;q.prior_world_x=5.;q.prior_world_z=6.;q.prior_confidence=.7;q.last_observed_time.sec=80;p.predictions=[q]
  pp.publish(p);wp.publish(w);wait_for(lambda:output(100,True,FusedTarget.SOURCE_TRACKED))
  w.header.stamp.sec=101;t.observed=False;t.position_source=WorldTarget.POSITION_PREDICTED;wp.publish(w);wait_for(lambda:output(101,False))
  p.header.stamp.sec=101;pp.publish(p);wait_for(lambda:output(101,True,FusedTarget.SOURCE_PRIOR))
  got=[m for m in received if m.header.stamp.sec==101 and m.targets[1].valid][-1].targets[1]
  assert got.world_x==5 and got.source_stamp.sec==101
  w.header.stamp.sec=102;t.observed=True;t.position_source=WorldTarget.POSITION_TRACKED;t.last_observed_time.sec=102;wp.publish(w);wait_for(lambda:output(102,True,FusedTarget.SOURCE_TRACKED))
  w.header.stamp.sec=103;t.observed=False;wp.publish(w);p.header.stamp.sec=103;pp.publish(p);wait_for(lambda:output(103,False))
  q.last_observed_time.sec=102;pp.publish(p);wait_for(lambda:output(103,True,FusedTarget.SOURCE_PRIOR))
  received.clear();p.model_enabled=False;pp.publish(p);wait_for(lambda:output(103,False))
  assert all(len(m.targets)==10 for m in received)
  print('PASS ROS fusion topics: observed priority, delayed prior update, no CV fallback, reacquisition, source timestamp, model disabled')
 finally:
  node.destroy_node();rclpy.shutdown();os.killpg(proc.pid,signal.SIGINT);proc.wait(timeout=10)

if __name__=='__main__':main()
