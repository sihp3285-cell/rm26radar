#!/usr/bin/python3
"""Shadow Mode 隔离性回归测试：输出话题约束、熔断、降级存活。

用法（需先 source ROS 与工作区，且包已 colcon build）：
    export ROS_DOMAIN_ID=77        # 任意未占用域
    python3 scripts/neural_sentry/test_shadow_isolation.py [--bundle DIR]

测试在进程内构造 neural_sentry_shadow 节点，不依赖相机、TensorRT 引擎或雷达主链。
"""
import argparse
import json
import math
import os
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
BUNDLE_DEFAULT = REPO / 'models/neural_sentry/sentry_v2'
SHADOW_PY = REPO / 'install/neural_sentry_decision/lib/neural_sentry_decision'

failures = []


def check(ok, what):
    print(f"{what:<64} {'PASS' if ok else 'FAIL'}")
    if not ok:
        failures.append(what)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bundle', default=str(BUNDLE_DEFAULT))
    args = ap.parse_args()
    bundle = Path(args.bundle)
    if not (bundle / 'manifest.json').is_file():
        ap.error(f'no shadow bundle at {bundle}; export it first (see package README)')
    if not SHADOW_PY.is_dir():
        ap.error(f'{SHADOW_PY} not found; run colcon build --packages-select neural_sentry_decision')

    sys.path.insert(0, str(SHADOW_PY))
    import rclpy
    from neural_sentry.node import NeuralSentryNode
    from neural_sentry.runtime import ShadowRuntime, HistoryBuffer
    from tensorrt_detect_msgs.msg import WorldTarget, WorldTargetArray

    rclpy.init(args=['--ros-args', '-p', f'bundle_dir:={bundle}'])
    node = NeuralSentryNode()

    def feed(stamp):
        """一帧视觉快照：己方哨兵在场地内小幅移动，其余为固定队友/敌人。"""
        msg = WorldTargetArray()
        msg.header.stamp.sec = int(stamp)
        msg.header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
        for rid, cls in ((7, 6), (107, 6), (3, 4), (103, 4), (4, 5), (104, 5)):
            t = WorldTarget()
            t.team_id = 1 if rid < 100 else 2
            t.class_id = t.stable_class_id = cls
            t.stable_class_conf = 1.0
            t.valid = t.observed = True
            t.position_source = 1
            if rid % 100 == 7:
                t.world_x = 3.0 + 0.2 * math.cos(stamp)
                t.world_z = 1.9 + 0.2 * math.sin(stamp)
            else:
                t.world_x = 0.0
                t.world_z = -10.0
            msg.targets.append(t)
        node.on_targets(msg)

    try:
        pub_names = [n for n, _ in node.get_publisher_names_and_types_by_node(
            node.get_name(), node.get_namespace())]
        app_pub = sorted(n for n in pub_names if n.startswith('/neural_sentry'))
        check(app_pub == ['/neural_sentry/shadow/markers', '/neural_sentry/shadow/suggestion'],
              '节点只发布 shadow 建议/标记话题')
        check(not any(k in n for n in pub_names for k in ('cmd', 'serial', 'fused', 'control')),
              '不存在任何控制量发布者')
        check(node.runtime is not None, 'bundle 正常加载')

        stamp = 1000.0
        for _ in range(node.runtime.steps + 2):
            feed(stamp)
            stamp += 1.0
        node.tick()
        check(node.last_status == 'running', '正常输入下产生有效建议')

        calls = {'n': 0}

        def boom(*a, **k):
            calls['n'] += 1
            raise RuntimeError('injected ONNX failure')

        node.runtime.predict = boom
        reasons = []
        node.suggestion_pub.publish = lambda m: reasons.append(json.loads(m.data)['reason'])
        node.marker_pub.publish = lambda m: None

        budget = node.p['max_consecutive_errors']
        for _ in range(budget):
            feed(stamp)
            stamp += 1.0
            node.tick()
        check(calls['n'] == budget, f'恰好尝试推理 {budget} 次（错误预算生效）')
        check(node.runtime is None, '熔断打开并释放 runtime')
        before = calls['n']
        feed(stamp)
        stamp += 1.0
        node.tick()
        check(calls['n'] == before, '熔断后不再调用模型')
        check('circuit open' in reasons[-1], '停用原因可传达给显示层')
        check(node.suggestion_pub is not None and node.marker_pub is not None,
              '节点保持存活并继续发布失效状态')

        node.runtime = ShadowRuntime(str(bundle), node.p['inference_timeout_s'])
        node.history = HistoryBuffer(node.runtime.cfg, node.p['snapshot_tolerance_s'])
        node.errors = 0
        reasons.clear()
        stamp = 2000.0
        for _ in range(node.runtime.steps + 1):
            feed(stamp)
            stamp += 1.0
        node._tick()
        check(node.errors == 0 and reasons and reasons[-1].startswith('BC suggestion'),
              '恢复 runtime 后重新出建议（预算清零）')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print('\n' + ('FAILED: ' + '; '.join(failures) if failures else 'ALL CHECKS PASSED'))
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
