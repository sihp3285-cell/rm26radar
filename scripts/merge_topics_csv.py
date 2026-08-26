#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把导出的 radar_map.csv 与 prior_predictions.csv 按时间戳融合成一份。
用法: merge_topics_csv.py <csv目录> [输出路径]
默认输出 <csv目录>/merged_video.csv
"""
import csv
import bisect
import sys
import os

def load(p):
    with open(p, newline="") as f:
        rd = csv.DictReader(f)
        return [dict(r) for r in rd], rd.fieldnames

def main():
    if len(sys.argv) < 2:
        print("用法: merge_topics_csv.py <csv目录> [输出路径]", file=sys.stderr)
        return 1
    cdir = sys.argv[1]
    out_path = sys.argv[2] if len(sys.argv) > 2 else os.path.join(cdir, "merged_video.csv")
    radar, rcols = load(os.path.join(cdir, "radar_map.csv"))
    prior, pcols = load(os.path.join(cdir, "prior_predictions.csv"))

    def key(row):
        return (int(row["header.stamp.sec"]), int(row["header.stamp.nanosec"]))

    def ts(row):
        return int(row["header.stamp.sec"]) + int(row["header.stamp.nanosec"]) * 1e-9

    prior_sorted = sorted(prior, key=ts)
    pts = [ts(x) for x in prior_sorted]
    prior_by_key = {key(x): x for x in prior}

    HEADER_COLS = ("header.stamp.sec", "header.stamp.nanosec", "header.frame_id")
    prior_cols = [c for c in pcols if c not in HEADER_COLS]
    out_cols = rcols + ["prior_" + c for c in prior_cols]

    def nearest_prior(t):
        i = bisect.bisect_left(pts, t)
        cand = []
        if i > 0:
            cand.append(prior_sorted[i-1])
        if i < len(pts):
            cand.append(prior_sorted[i])
        if not cand:
            return None
        best = min(cand, key=lambda x: abs(ts(x) - t))
        return best if abs(ts(best) - t) <= 0.05 else None

    n = 0
    nm = 0
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(out_cols)
        for r in radar:
            p = prior_by_key.get(key(r)) or nearest_prior(ts(r))
            row = [r[c] for c in rcols]
            if p is not None:
                nm += 1
                row.extend(p[c] for c in prior_cols)
            else:
                row.extend([""] * len(prior_cols))
            w.writerow(row)
            n += 1
    print("merged rows=%d, 匹配先验=%d (%.1f%%) -> %s" % (n, nm, 100.0 * nm / n, out_path))
    return 0

if __name__ == "__main__":
    sys.exit(main())
