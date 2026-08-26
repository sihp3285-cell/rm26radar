#!/usr/bin/env bash
# 自动测试 v3：不经 launch/pty，直接以独立可执行文件运行主链四节点 + 先验节点，
# 不加载 Qt 显示节点，录制 /radar_map + /prior_predictions。
# 用法: ./scripts/run_noqt_test.sh [qt_on|qt_off] [录制秒数]  (默认 qt_off, 480s)
set -u
MODE="${1:-qt_off}"
DURATION="${2:-480}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT" || exit 1

set +u
source /opt/ros/jazzy/setup.bash
source "$REPO_ROOT/install/setup.bash"
set -u
export ROS_HOME="$REPO_ROOT/log/.ros_noqt"
mkdir -p "$ROS_HOME/log"
export ROS_LOG_DIR="$ROS_HOME/log" ROS_PYTHON_LOG_DIR="$ROS_HOME/log"

TS="$(date +%Y%m%d_%H%M%S)"
OUT_ROOT="$REPO_ROOT/log/bags"
BAG="$OUT_ROOT/noqt_${TS}_bag"
LOG="$OUT_ROOT/noqt_${TS}_logs"
mkdir -p "$LOG"
PARAMS="$REPO_ROOT/install/tensorrt_detect/share/tensorrt_detect/config/ros2_params.yaml"
PRIOR_PARAMS="$REPO_ROOT/install/position_prior/share/position_prior/config/position_prior.yaml"
echo "[test] $(date +%H:%M:%S) mode=$MODE bag=$BAG"
echo "[test] $(date +%H:%M:%S) logs=$LOG"

PIDS=""
# 1) 录制（-o 目录必须不存在）
ros2 bag record -o "$BAG" --topics \
  /armor_detections /world_targets /radar_map /prior_predictions /pipeline_timing \
  > "$LOG/record.log" 2>&1 &
REC_PID=$!

# 2) 主链四节点（独立进程，无 Qt）
VIDEO_EXTRA=""
if [ "${SYNTHETIC_STAMP:-0}" = "1" ]; then
  VIDEO_EXTRA="-p synthetic_stamp:=true"
fi
ros2 run tensorrt_detect video_node --ros-args -r __node:=video_node --params-file "$PARAMS" $VIDEO_EXTRA > "$LOG/video.log" 2>&1 &
PIDS="$PIDS $!"
# detect: UI 关闭时不发布调试图；确定性采样由环境变量控制
DETECT_EXTRA="-p publish_debug_image:=false"
if [ "${SYNTHETIC_STAMP:-0}" = "1" ]; then
  DETECT_EXTRA="$DETECT_EXTRA -p frame_sampling_enabled:=true -p frame_sampling_step:=${SAMPLE_STEP:-2} -p frame_sampling_period_ms:=50"
fi
ros2 run tensorrt_detect detect_node --ros-args -r __node:=detect_node --params-file "$PARAMS" $DETECT_EXTRA > "$LOG/detect.log" 2>&1 &
PIDS="$PIDS $!"
ros2 run tensorrt_detect pose_node --ros-args -r __node:=pose_node --params-file "$PARAMS" > "$LOG/pose.log" 2>&1 &
PIDS="$PIDS $!"
ros2 run tensorrt_detect map_node --ros-args -r __node:=map_node --params-file "$PARAMS" > "$LOG/map.log" 2>&1 &
PIDS="$PIDS $!"

# 3) Qt on 模式: 额外启动 qt_display_node（独立进程显示）
if [ "$MODE" = "qt_on" ]; then
  ros2 run tensorrt_detect qt_display_node --ros-args -r __node:=qt_display_node --params-file "$PARAMS" > "$LOG/qt.log" 2>&1 &
  PIDS="$PIDS $!"
fi

# 4) 先验节点（提供 /prior_predictions）
ros2 run position_prior position_prior_node --ros-args -r __node:=position_prior_node --params-file "$PRIOR_PARAMS" > "$LOG/prior.log" 2>&1 &
PIDS="$PIDS $!"

# 5) 等待 /radar_map 就绪（最多 180s）
READY=0
for i in $(seq 1 36); do
  if ros2 topic list 2>/dev/null | grep -qx /radar_map; then
    READY=1
    echo "[test] $(date +%H:%M:%S) /radar_map 已发布 (${i}x5s)"
    break
  fi
  sleep 5
done
if [ "$READY" -ne 1 ]; then
  echo "[test] $(date +%H:%M:%S) 失败: /radar_map 未出现"
  echo "[test] FAILED" > "$LOG/result.txt"
  kill -INT "$REC_PID" 2>/dev/null
  for p in $PIDS; do kill -9 "$p" 2>/dev/null; done
  exit 1
fi

# 6) 录制 DURATION 秒
START=$(date +%s)
while :; do
  NOW=$(date +%s)
  ELAPSED=$((NOW - START))
  if [ "$ELAPSED" -ge 75 ]; then
    SIZE=$(du -sb "$BAG"/*.mcap 2>/dev/null | cut -f1 | head -1)
    SIZE="${SIZE:-0}"
    if [ "$SIZE" -lt 100000 ]; then
      echo "[test] $(date +%H:%M:%S) 警告: bag 数据过小(${SIZE}B)"
    fi
  fi
  if [ "$ELAPSED" -ge "$DURATION" ]; then
    echo "[test] $(date +%H:%M:%S) 录制时长到 (${ELAPSED}s)"
    break
  fi
  sleep 5
done

# 7) 收尾
kill -INT "$REC_PID" 2>/dev/null
sleep 3
wait "$REC_PID" 2>/dev/null || true
for p in $PIDS; do kill -INT "$p" 2>/dev/null; done
sleep 3
for p in $PIDS; do kill -9 "$p" 2>/dev/null || true; done

echo "[test] $(date +%H:%M:%S) 完成: $BAG"
echo "$BAG" > "$REPO_ROOT/log/noqt_last_bag.txt"
echo "OK" > "$LOG/result.txt"