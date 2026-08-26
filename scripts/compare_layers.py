#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分层对比: 两份诊断bag的 /armor_detections /pipeline_timing
用法: compare_layers.py <bagA_dir> <bagB_dir> [aA bA aB bB]
输出: 帧序列统计 / 共同内容帧检测一致性 / 耗时对比
"""
import csv, sys, numpy as np, os

ROLE_NAMES = {2: "英雄", 3: "工程", 4: "步兵3", 5: "步兵4", 6: "哨兵", 7: "前哨站"}

import re
ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")

def load_csv(path):
    """读取 ros2 topic echo --csv 原始输出（无表头，含 ANSI 日志行）"""
    raw = []
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            line = ANSI_RE.sub("", line).rstrip("\n")
            if not line.strip() or re.match(r"^\d{4}-\d{2}-\d{2} ", line) or line.startswith("WARNING:"):
                continue
            raw.append(line)
    rows = list(csv.reader(raw))
    # 若含表头（header.stamp.sec 开头）则跳过
    while rows and rows[0] and rows[0][0] == "header.stamp.sec":
        rows.pop(0)
    if not rows:
        return [], None
    T0 = float(rows[0][0]) + float(rows[0][1]) * 1e-9
    def ts(r):
        return float(r[0]) + float(r[1]) * 1e-9 - T0
    return rows, ts

def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 1
    bagA, bagB = sys.argv[1], sys.argv[2]
    aA = float(sys.argv[3]) if len(sys.argv) > 3 else 0.854
    bA = float(sys.argv[4]) if len(sys.argv) > 4 else 25.2
    aB = float(sys.argv[5]) if len(sys.argv) > 5 else 0.760
    bB = float(sys.argv[6]) if len(sys.argv) > 6 else 15.5

    def arm(csvdir, a, b):
        p = os.path.join(csvdir, "armor_detections.csv")
        rows, ts = load_csv(p)
        # 每帧: (match_t, [(idx, x, y, w, h, conf)])  按 detections[k] 前缀聚合
        frames = {}
        N_FIELDS = 17  # DetectionBox 字段数
        for r in rows:
            mt = (ts(r) - b) / a
            if mt < 1.0 or mt > 420.0:
                continue
            dets = []
            n_det = (len(r) - 3) // N_FIELDS
            for k in range(n_det):
                base = 3 + k * N_FIELDS
                if base + 8 > len(r):
                    break
                try:
                    dets.append((int(float(r[base])),       # idx
                                 float(r[base+2]), float(r[base+3]),   # x, y
                                 float(r[base+4]), float(r[base+5])))  # w, h
                except (ValueError, IndexError):
                    continue
            frames[mt] = dets
        return frames

    fa = arm(os.path.join(bagA, "csv"), aA, bA)
    fb = arm(os.path.join(bagB, "csv"), aB, bB)
    ta = np.array(sorted(fa)); tb = np.array(sorted(fb))
    out = []
    out.append("== 帧序列统计 ==")
    out.append("bagA: %d 帧, 帧率%.2fHz, 间隔CV=%.3f" % (len(ta), 1/np.diff(ta).mean(), np.diff(ta).std()/np.diff(ta).mean()))
    out.append("bagB: %d 帧, 帧率%.2fHz, 间隔CV=%.3f" % (len(tb), 1/np.diff(tb).mean(), np.diff(tb).std()/np.diff(tb).mean()))
    # 共同内容覆盖: 每0.05s一格, 是否两bag都处理了帧
    grid = np.arange(1, 420, 0.05)
    ca = np.zeros(len(grid), bool); cb = np.zeros(len(grid), bool)
    for t in ta:
        ca[(grid >= t-0.025) & (grid < t+0.025)] = True
    for t in tb:
        cb[(grid >= t-0.025) & (grid < t+0.025)] = True
    both = (ca & cb).sum(); onlyA = (ca & ~cb).sum(); onlyB = (~ca & cb).sum()
    out.append("覆盖: 共同=%.1fs, 仅A=%.1fs, 仅B=%.1fs" % (both*0.05, onlyA*0.05, onlyB*0.05))

    # 共同帧检测一致性
    agree = 0; total = 0; det_diff = 0
    for tA in ta:
        # 找B中最近的帧
        i = np.searchsorted(tb, tA)
        cand = []
        if i > 0: cand.append(tb[i-1])
        if i < len(tb): cand.append(tb[i])
        if not cand: continue
        tB = min(cand, key=lambda x: abs(x-tA))
        if abs(tB - tA) > 0.05: continue
        total += 1
        da = fa[tA]; db = fb[tB]
        # 按 idx 归类比较 bbox 中心
        def norm(dets):
            m = {}
            for d in dets:
                idx, x, y, w, h = d
                m.setdefault(idx, []).append((x+w/2, y+h/2))
            return m
        ma, mb = norm(da), norm(db)
        if set(ma) == set(mb):
            pos_ok = True
            for k in ma:
                for (px, py) in ma[k]:
                    if not any(abs(px - qx) < 30 and abs(py - qy) < 30 for (qx, qy) in mb[k]):
                        pos_ok = False
                        break
                if not pos_ok:
                    break
            agree += 1 if pos_ok else 0
        det_diff += abs(len(da) - len(db))
    out.append("共同帧数: %d | 类别集合一致率: %.1f%% | 每帧检测数平均差: %.2f" % (total, 100.0*agree/max(1,total), det_diff/max(1,total)))

    # pipeline_timing
    for nm, bag in (("A", bagA), ("B", bagB)):
        p = os.path.join(bag, "csv", "pipeline_timing.csv")
        if not os.path.exists(p):
            continue
        rows, ts = load_csv(p)
        if rows:
            fps = np.array([float(r[10]) for r in rows])       # fps 列
            e2e = np.array([float(r[9]) for r in rows])        # end_to_end_ms 列
            out.append("%s timing: fps中位%.1f(p10-%.1f) e2e中位%.1fms" % (nm, np.median(fps), np.percentile(fps,10), np.median(e2e)))
    print("\n".join(out))
    return 0

if __name__ == "__main__":
    sys.exit(main())