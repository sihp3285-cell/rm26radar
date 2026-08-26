#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
清理 ros2 topic echo --csv 的输出：
  1. 去掉 FastDDS/rclpy 混入 stdout 的日志行（含 ANSI 颜色码）
  2. 根据消息定义生成 CSV 表头（数组按索引展开，行按最长对齐补齐）
用法: csv_cleanup.py <输入.csv> <输出.csv> <RadarMap|PriorPredictionArray>
"""
import csv
import re
import sys

ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
LOG_LINE_RE = re.compile(r"^\d{4}-\d{2}-\d{2} |^WARNING: topic |RTPS_")

# ── 消息字段结构（与 ros2 topic echo --csv 的递归展平顺序一致） ──
def radar_map_columns(n_blue=6, n_red=6):
    cols = ["header.stamp.sec", "header.stamp.nanosec", "header.frame_id"]
    for i in range(n_blue):
        cols.append(f"blue_x[{i}]")
    for i in range(n_blue):
        cols.append(f"blue_y[{i}]")
    for i in range(n_red):
        cols.append(f"red_x[{i}]")
    for i in range(n_red):
        cols.append(f"red_y[{i}]")
    return cols

PRIOR_PRED_COLS = [
    "slot_idx", "track_id", "team_id", "role_class_id", "valid",
    "rejection_code", "rejection_reason",
    "last_observed_time.sec", "last_observed_time.nanosec",
    "lost_duration_s", "horizon_seconds",
    "last_world_x", "last_world_z",
    "tracker_world_x", "tracker_world_z",
    "prior_world_x", "prior_world_z",
    "prior_field_x", "prior_field_y",
    "prior_canonical_x", "prior_canonical_y",
    "prior_confidence", "normalized_entropy", "reachable_probability_mass",
    "fallback_level", "sample_count",
    "motion_gated", "mesh_used", "blind_zone_biased",
    "blind_zone_probability_mass", "stay_anchor_probability_mass",
]

PRIOR_CAND_COLS = [
    "grid_index", "prior_probability", "fused_probability",
    "canonical_x", "canonical_y", "field_x", "field_y", "world_x", "world_z",
    "reachable", "blocked", "from_blind_zone", "stay_anchor",
    "distance_from_last_m", "straight_distance_from_last_m",
]

def prior_prediction_array_columns(max_cols):
    # 前 5 列固定：header(3) + model_enabled + model_status
    # 结构：最多 5 个预测（enabled_roles 全开），每个预测 31 字段 + 最多 5 个候选 x 15 字段
    # 5 + 5*(31 + 5*15) = 535
    cols = ["header.stamp.sec", "header.stamp.nanosec", "header.frame_id",
            "model_enabled", "model_status"]
    n_pred = 5
    n_cand = 5
    for p in range(n_pred):
        for fld in PRIOR_PRED_COLS:
            cols.append(f"predictions[{p}].{fld}")
        for c in range(n_cand):
            for fld in PRIOR_CAND_COLS:
                cols.append(f"predictions[{p}].candidates[{c}].{fld}")
    return cols

def main():
    if len(sys.argv) != 4:
        print("用法: csv_cleanup.py <输入.csv> <输出.csv> <RadarMap|PriorPredictionArray>",
              file=sys.stderr)
        return 1
    src, dst, msgtype = sys.argv[1], sys.argv[2], sys.argv[3]

    rows = []
    with open(src, "r", encoding="utf-8", errors="replace") as fh:
        for raw in fh:
            line = ANSI_RE.sub("", raw).rstrip("\n")
            if not line.strip() or LOG_LINE_RE.search(line):
                continue
            # 跳过重复的表头行（重复运行清理时产生）
            if line.split(',')[0] == 'header.stamp.sec':
                continue
            rows.append(line)

    if not rows:
        print(f"错误: {src} 没有数据行", file=sys.stderr)
        return 1

    max_cols = max(r.count(",") + 1 for r in rows)
    if msgtype == "RadarMap":
        header = radar_map_columns()
        if len(header) != max_cols:
            print(f"警告: RadarMap 期望 {len(header)} 列，实际最大 {max_cols}", file=sys.stderr)
    elif msgtype == "PriorPredictionArray":
        header = prior_prediction_array_columns(max_cols)
        if len(header) != max_cols:
            print(f"警告: 表头 {len(header)} 列 vs 最大数据 {max_cols} 列，请检查", file=sys.stderr)
    else:
        print(f"错误: 不支持的消息类型 {msgtype}", file=sys.stderr)
        return 1

    with open(dst, "w", encoding="utf-8", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(header)
        for r in rows:
            fields = r.split(",")
            if len(fields) < len(header):
                fields += [""] * (len(header) - len(fields))
            writer.writerow(fields[:len(header)])

    print(f"{msgtype}: 数据行 {len(rows)}，列数 {len(header)} -> {dst}")
    return 0

if __name__ == "__main__":
    sys.exit(main())