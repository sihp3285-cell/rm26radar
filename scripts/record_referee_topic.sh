#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# 录制最终提交给裁判系统的输出话题，以及猜点话题。
#
# 默认录制 5 个话题（含分层诊断所需，bag 不含图像，体积很小）:
#   /armor_detections   - DetectNode 检测输出（帧号/类别/框），定位分叉层用
#   /world_targets      - PoseNode 跟踪输出（世界坐标目标）
#   /radar_map          - MapNode 发布给裁判系统的固定槽位地图坐标（最终输出）
#   /prior_predictions  - Position Prior 猜点输出（主猜点 + Top-K 候选）
#   /pipeline_timing    - 各阶段耗时与 fps（对比 Qt on/off 负载用）
# 传参可覆盖为任意话题组合。
#
# 用法:
#   ./scripts/record_referee_topic.sh                       # 录制默认两个话题
#   ./scripts/record_referee_topic.sh /radar_map            # 只录制指定话题
#   ./scripts/record_referee_topic.sh /a /b /c              # 录制多个话题
#   OUT_DIR=/path/to/bags ./scripts/record_referee_topic.sh # 指定输出目录
#
# 输出: <OUT_DIR>/referee_YYYYmmdd_HHMMSS/  (rosbag，含 metadata.yaml)
# 查看: ros2 bag info <bag目录>
# 回放: ros2 bag play <bag目录>
# 快速转 CSV: ros2 topic echo /prior_predictions --csv > prior.csv
# ─────────────────────────────────────────────────────────────────────────────
set -euo pipefail

# ── 可配置项 ────────────────────────────────────────────────
# 默认话题：裁判系统输出 + 猜点；命令行传参会整体覆盖
if [[ $# -gt 0 ]]; then
    TOPICS=("$@")
else
    TOPICS=(/armor_detections /world_targets /radar_map /prior_predictions /pipeline_timing)
fi
# bag 输出根目录；可用环境变量 OUT_DIR 覆盖，默认仓库 log/bags
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
OUT_ROOT="${OUT_DIR:-$REPO_ROOT/log/bags}"
# ─────────────────────────────────────────────────────────────

# 1. 确保 ros2 可用（优先使用工作空间环境）
if ! command -v ros2 >/dev/null 2>&1; then
    if [[ -f "$REPO_ROOT/install/setup.bash" ]]; then
        # shellcheck disable=SC1091
        source "$REPO_ROOT/install/setup.bash"
    else
        echo "错误: 找不到 ros2 命令，请先 source 工作空间环境" >&2
        echo "  source $REPO_ROOT/install/setup.bash" >&2
        exit 1
    fi
fi

# 2. 提醒缺失的话题（主链/先验节点是否已启动），不阻塞录制
for t in "${TOPICS[@]}"; do
    if ! ros2 topic list 2>/dev/null | grep -qx "$t"; then
        echo "警告: 当前没有节点发布 [$t]，请确认 detect_pipeline 主链已启动" >&2
    fi
done

# 3. 创建输出目录并录制
mkdir -p "$OUT_ROOT"
BAG_DIR="$OUT_ROOT/referee_$(date +%Y%m%d_%H%M%S)"
echo "==> 开始录制: ${TOPICS[*]}"
echo "    -> $BAG_DIR"
echo "    按 Ctrl+C 停止录制"

# Ctrl+C 时 ros2 bag record 会保存并退出；忽略其退出码以便打印 bag 信息
ros2 bag record -o "$BAG_DIR" --topics "${TOPICS[@]}" || true

# 4. 停止后打印 bag 摘要
if [[ -d "$BAG_DIR" ]]; then
    echo ""
    echo "==> 录制完成，bag 信息:"
    ros2 bag info "$BAG_DIR"
    echo ""
    echo "bag 位置: $BAG_DIR"
else
    echo "错误: bag 未生成（$BAG_DIR 不存在）" >&2
    exit 1
fi