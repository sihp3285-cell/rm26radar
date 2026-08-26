#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# 把 rosbag 中的话题导出为 CSV（ros2 bag play + ros2 topic echo --csv）。
#
# 用法:
#   ./scripts/export_bag_to_csv.sh <bag目录>                 # 导出 bag 内全部话题
#   ./scripts/export_bag_to_csv.sh <bag目录> /radar_map /prior_predictions
#
# 输出: <bag目录>/csv/<话题名>.csv
# ─────────────────────────────────────────────────────────────────────────────
set -euo pipefail

BAG="${1:?用法: ./scripts/export_bag_to_csv.sh <bag目录> [topic...]}"; shift || true

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# ROS 的 setup 脚本不兼容 set -u，source 期间临时关闭 nounset
set +u

# 1. 准备 ROS2 环境
ROS_SETUP="$(ls -d /opt/ros/*/setup.bash 2>/dev/null | head -1 || true)"
if [[ -n "$ROS_SETUP" ]]; then
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
fi
if [[ -f "$REPO_ROOT/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$REPO_ROOT/install/setup.bash"
fi

set -u

# 将 ROS 日志指向可写目录，避免 ~/.ros 权限问题导致 play 段错误
export ROS_HOME="${ROS_HOME:-$REPO_ROOT/log/.ros}"
export ROS_LOG_DIR="$ROS_HOME/log"
export ROS_PYTHON_LOG_DIR="$ROS_HOME/log"
mkdir -p "$ROS_LOG_DIR"


if ! command -v ros2 >/dev/null 2>&1; then
    echo "错误: 找不到 ros2 命令" >&2
    exit 1
fi

# 2. 确定话题：命令行指定，或从 bag 元数据读取全部
if [[ $# -gt 0 ]]; then
    TOPICS=("$@")
else
    mapfile -t TOPICS < <(ros2 bag info "$BAG" 2>/dev/null | grep -oE "Topic: [^ ]+" | awk '{print $2}')
    if [[ ${#TOPICS[@]} -eq 0 ]]; then
        echo "错误: 未能从 bag 解析话题列表" >&2
        exit 1
    fi
fi

# 3. 逐话题导出
OUT_DIR="$BAG/csv"
mkdir -p "$OUT_DIR"

for t in "${TOPICS[@]}"; do
    name="$(basename "$t")"
    # 从 bag 元数据解析消息类型（echo 需显式类型，避免启动时无发布者而失败）
    msgtype="$(ros2 bag info "$BAG" 2>/dev/null | grep -E "Topic: $t \| Type: " | sed -E 's/.*Type: ([^ ]+).*/\1/' | head -1)"
    if [[ -z "$msgtype" ]]; then
        echo "错误: 无法解析 [$t] 的消息类型" >&2
        exit 1
    fi
    out="$OUT_DIR/$name.csv"
    echo "==> 导出 [$t] -> $out"

    # 先启动 echo 订阅，等发现完成后再播放，避免丢帧
    PYTHONUNBUFFERED=1 ros2 topic echo "$t" "$msgtype" --csv > "$out" 2>"$out.stderr" &
    ECHO_PID=$!
    sleep 5

    # 50 倍速回放该话题；结束后给 echo 一点时间收尾再中断
    ros2 bag play "$BAG" --topics "$t" --rate 50 --disable-loan-message > "$OUT_DIR/play_$name.log" 2>&1 || true
    sleep 2
    kill -INT "$ECHO_PID" 2>/dev/null || true
    wait "$ECHO_PID" 2>/dev/null || true
    echo "    完成: $(wc -l < "$out") 行（含表头）"
    rm -f "$out.stderr" "$OUT_DIR/play_$name.log"
    # 清理 FastDDS 日志行并生成表头（数组列按最长帧对齐补齐）
    python3 "$SCRIPT_DIR/csv_cleanup.py" "$out" "$out" "${msgtype##*/}" || true
done

echo ""
echo "CSV 已导出到: $OUT_DIR"
ls -lh "$OUT_DIR"