#!/usr/bin/env python3
"""Build a sparse role-conditioned position prior from the RMUC SQLite dataset.

The database is always opened read-only. The generated .yaml file deliberately
uses JSON syntax, which is valid YAML 1.2 and can be loaded by yaml-cpp.
This script requires only the Python standard library.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import random
import sqlite3
import statistics
import sys
import time
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Iterator, List, Optional, Sequence, Tuple


ROLE_BY_ROBOT_ID = {
    1: "hero", 101: "hero",
    2: "engineer", 102: "engineer",
    3: "infantry3", 103: "infantry3",
    4: "infantry4", 104: "infantry4",
    7: "sentry", 107: "sentry",
}

DISPLAY_NAMES = {
    "hero": "英雄",
    "engineer": "工程",
    "infantry": "步兵合并",
    "infantry3": "步兵3",
    "infantry4": "步兵4",
    "sentry": "哨兵",
}

MODEL_ROLES = ("hero", "engineer", "infantry", "infantry3", "infantry4", "sentry")
SPECIFIC_ROLES = ("hero", "engineer", "infantry3", "infantry4", "sentry")


@dataclass(frozen=True)
class Point:
    t: int
    x: float
    y: float


@dataclass(frozen=True)
class EvalSample:
    role: str
    t: int
    horizon: int
    current_x: float
    current_y: float
    previous_x: float
    previous_y: float
    truth_x: float
    truth_y: float


class Reservoir:
    def __init__(self, capacity: int, rng: random.Random) -> None:
        self.capacity = capacity
        self.rng = rng
        self.items: List[EvalSample] = []
        self.seen = 0

    def add(self, item: EvalSample) -> None:
        self.seen += 1
        if len(self.items) < self.capacity:
            self.items.append(item)
            return
        replacement = self.rng.randrange(self.seen)
        if replacement < self.capacity:
            self.items[replacement] = item


def open_readonly(path: Path) -> sqlite3.Connection:
    absolute = path.resolve()
    con = sqlite3.connect(f"file:{absolute.as_posix()}?mode=ro", uri=True)
    con.execute("PRAGMA query_only=ON")
    return con


def write_json(path: Path, value: object, pretty: bool = True) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as handle:
        json.dump(
            value,
            handle,
            ensure_ascii=False,
            indent=2 if pretty else None,
            separators=None if pretty else (",", ":"),
            sort_keys=False,
        )
        handle.write("\n")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        while True:
            block = handle.read(8 * 1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def percentile(values: Sequence[float], probability: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    position = (len(ordered) - 1) * probability
    low = int(math.floor(position))
    high = int(math.ceil(position))
    if low == high:
        return ordered[low]
    fraction = position - low
    return ordered[low] * (1.0 - fraction) + ordered[high] * fraction


def scan_database(con: sqlite3.Connection, db_path: Path) -> dict:
    cur = con.cursor()
    schema = []
    for kind, name, table_name, sql in cur.execute(
        "SELECT type,name,tbl_name,sql FROM sqlite_master "
        "WHERE type IN ('table','view','index') ORDER BY type,name"
    ):
        schema.append({"type": kind, "name": name, "table": table_name, "sql": sql})

    counts = {
        "matches": cur.execute("SELECT COUNT(*) FROM matches").fetchone()[0],
        "events": cur.execute("SELECT COUNT(*) FROM events").fetchone()[0],
        "timeseries": cur.execute("SELECT COUNT(*) FROM timeseries").fetchone()[0],
    }
    games = cur.execute("SELECT COUNT(DISTINCT game_id) FROM matches").fetchone()[0]
    schools = cur.execute(
        'SELECT COUNT(*) FROM ('
        'SELECT "红方学校" AS school FROM matches '
        'UNION SELECT "蓝方学校" FROM matches)'
    ).fetchone()[0]

    region_rows = cur.execute(
        'SELECT "赛区",COUNT(DISTINCT game_id),COUNT(DISTINCT "学校名") '
        'FROM timeseries GROUP BY "赛区" ORDER BY "赛区"'
    ).fetchall()
    regions = [
        {"name": region, "games": region_games, "schools": region_schools}
        for region, region_games, region_schools in region_rows
    ]

    robot_rows = cur.execute(
        'SELECT robot_id,"机器人类型",COUNT(*),COUNT(DISTINCT game_id),'
        'MIN(x),MAX(x),MIN(y),MAX(y),MIN(z),MAX(z) '
        'FROM timeseries GROUP BY robot_id,"机器人类型" ORDER BY robot_id'
    ).fetchall()
    robots = [
        {
            "robot_id": rid, "type": rtype, "rows": rows, "games": robot_games,
            "x_min": xmin, "x_max": xmax, "y_min": ymin, "y_max": ymax,
            "z_min": zmin, "z_max": zmax,
        }
        for rid, rtype, rows, robot_games, xmin, xmax, ymin, ymax, zmin, zmax in robot_rows
    ]

    ids = tuple(ROLE_BY_ROBOT_ID)
    placeholders = ",".join("?" for _ in ids)
    mobile = cur.execute(
        f'SELECT COUNT(*),'
        f'SUM(NOT (x BETWEEN 0 AND 28 AND y BETWEEN 0 AND 15)),'
        f'SUM(x=0 AND y=0),SUM(x IS NULL OR y IS NULL) '
        f'FROM timeseries WHERE robot_id IN ({placeholders})', ids
    ).fetchone()
    sampling = cur.execute(
        'SELECT MIN(n),AVG(n),MAX(n),MIN(dt),AVG(dt),MAX(dt) FROM ('
        'SELECT game_id,robot_id,COUNT(*) n,'
        '(MAX("时刻秒")-MIN("时刻秒"))/NULLIF(COUNT(*)-1,0) dt '
        'FROM timeseries GROUP BY game_id,robot_id)'
    ).fetchone()

    events = [
        {"type": event_type, "rows": rows}
        for event_type, rows in cur.execute(
            'SELECT "事件类型",COUNT(*) FROM events GROUP BY "事件类型" ORDER BY COUNT(*) DESC'
        )
    ]

    return {
        "database": {
            "path": str(db_path.resolve()),
            "bytes": db_path.stat().st_size,
            "last_modified_epoch": db_path.stat().st_mtime,
        },
        "counts": counts,
        "distinct_games": games,
        "distinct_schools": schools,
        "regions": regions,
        "robots": robots,
        "mobile_quality": {
            "rows": mobile[0],
            "out_of_field": mobile[1],
            "exact_zero_zero": mobile[2],
            "null_xy": mobile[3],
        },
        "sampling_per_game_robot": {
            "count_min": sampling[0], "count_mean": sampling[1], "count_max": sampling[2],
            "dt_min": sampling[3], "dt_mean": sampling[4], "dt_max": sampling[5],
        },
        "event_types": events,
        "schema": schema,
    }


class Geometry:
    def __init__(self, config: dict) -> None:
        self.field_x = float(config["field_width_x"])
        self.field_y = float(config["field_width_y"])
        self.zone_x = float(config["current_zone_size_x"])
        self.zone_y = float(config["current_zone_size_y"])
        self.grid_x = float(config["endpoint_grid_size_x"])
        self.grid_y = float(config["endpoint_grid_size_y"])
        self.zone_nx = int(math.ceil(self.field_x / self.zone_x))
        self.zone_ny = int(math.ceil(self.field_y / self.zone_y))
        self.grid_nx = int(math.ceil(self.field_x / self.grid_x))
        self.grid_ny = int(math.ceil(self.field_y / self.grid_y))

    def valid(self, x: object, y: object) -> bool:
        if x is None or y is None:
            return False
        try:
            xf = float(x)
            yf = float(y)
        except (TypeError, ValueError):
            return False
        if not math.isfinite(xf) or not math.isfinite(yf):
            return False
        if xf == 0.0 and yf == 0.0:
            return False
        return 0.0 <= xf <= self.field_x and 0.0 <= yf <= self.field_y

    def canonical(self, robot_id: int, faction: str, x: float, y: float) -> Tuple[float, float]:
        is_blue = faction == "蓝" or robot_id >= 100
        if is_blue:
            return self.field_x - x, self.field_y - y
        return x, y

    def zone_index(self, x: float, y: float) -> int:
        ix = min(self.zone_nx - 1, max(0, int(x / self.zone_x)))
        iy = min(self.zone_ny - 1, max(0, int(y / self.zone_y)))
        return iy * self.zone_nx + ix

    def grid_index(self, x: float, y: float) -> int:
        ix = min(self.grid_nx - 1, max(0, int(x / self.grid_x)))
        iy = min(self.grid_ny - 1, max(0, int(y / self.grid_y)))
        return iy * self.grid_nx + ix

    def grid_center(self, index: int) -> Tuple[float, float]:
        iy, ix = divmod(index, self.grid_nx)
        return (ix + 0.5) * self.grid_x, (iy + 0.5) * self.grid_y


def phase_name(config: dict, t: int) -> str:
    for item in config["phase_ranges"]:
        if int(item["start"]) <= t < int(item["end"]):
            return str(item["name"])
    return "all_phase"


def role_variants(role: str) -> Tuple[str, ...]:
    if role in ("infantry3", "infantry4"):
        return role, "infantry"
    return (role,)


def game_splits(con: sqlite3.Connection, config: dict) -> Tuple[dict, Dict[int, str]]:
    games = [row[0] for row in con.execute("SELECT DISTINCT game_id FROM matches ORDER BY game_id")]
    rng = random.Random(int(config["split_seed"]))
    rng.shuffle(games)
    train_end = int(len(games) * float(config["train_ratio"]))
    valid_end = int(len(games) * (float(config["train_ratio"]) + float(config["validation_ratio"])))
    split_lists = {
        "train": sorted(games[:train_end]),
        "validation": sorted(games[train_end:valid_end]),
        "test": sorted(games[valid_end:]),
    }
    lookup = {game_id: name for name, ids in split_lists.items() for game_id in ids}
    return {
        "seed": int(config["split_seed"]),
        "counts": {name: len(ids) for name, ids in split_lists.items()},
        "games": split_lists,
    }, lookup


def iter_raw_sequences(con: sqlite3.Connection) -> Iterator[Tuple[Tuple[int, int, str, str], List[Tuple[int, object, object]]]]:
    ids = tuple(ROLE_BY_ROBOT_ID)
    placeholders = ",".join("?" for _ in ids)
    query = (
        'SELECT game_id,robot_id,"机器人类型","阵营","时刻秒",x,y '
        f'FROM timeseries WHERE robot_id IN ({placeholders}) '
        'ORDER BY game_id,robot_id,"时刻秒"'
    )
    current_key: Optional[Tuple[int, int, str, str]] = None
    rows: List[Tuple[int, object, object]] = []
    for game_id, robot_id, robot_type, faction, t, x, y in con.execute(query, ids):
        key = (int(game_id), int(robot_id), str(robot_type), str(faction))
        if current_key is not None and key != current_key:
            yield current_key, rows
            rows = []
        rows.append((int(round(float(t))), x, y))
        current_key = key
    if current_key is not None:
        yield current_key, rows


def clean_segments(
    key: Tuple[int, int, str, str],
    rows: Sequence[Tuple[int, object, object]],
    geometry: Geometry,
    config: dict,
    quality: Counter,
) -> Iterator[List[Point]]:
    _, robot_id, _, faction = key
    max_speed = float(config["max_physical_speed_mps"])
    minimum = int(config["minimum_segment_points"])
    segment: List[Point] = []

    def finish() -> Optional[List[Point]]:
        nonlocal segment
        result = segment if len(segment) >= minimum else None
        if segment and result is None:
            quality["short_segment_points"] += len(segment)
        segment = []
        return result

    for t, raw_x, raw_y in rows:
        quality["raw_mobile_rows"] += 1
        if not geometry.valid(raw_x, raw_y):
            if raw_x == 0 and raw_y == 0:
                quality["zero_zero"] += 1
            elif raw_x is None or raw_y is None:
                quality["null_xy"] += 1
            else:
                quality["invalid_or_out_of_field"] += 1
            completed = finish()
            if completed:
                yield completed
            continue

        x, y = geometry.canonical(robot_id, faction, float(raw_x), float(raw_y))
        point = Point(t=t, x=x, y=y)
        if segment:
            dt = point.t - segment[-1].t
            distance = math.hypot(point.x - segment[-1].x, point.y - segment[-1].y)
            if dt <= 0 or dt > 1 or distance / dt > max_speed:
                if dt > 1:
                    quality["time_gap_breaks"] += 1
                elif dt <= 0:
                    quality["non_monotonic_breaks"] += 1
                else:
                    quality["speed_jump_breaks"] += 1
                completed = finish()
                if completed:
                    yield completed
        segment.append(point)
        quality["valid_rows"] += 1

    completed = finish()
    if completed:
        yield completed


def smooth_counter(source: Counter, geometry: Geometry) -> Counter:
    """Apply a small deterministic 3x3 kernel to a sparse grid."""
    result: Counter = Counter()
    kernel = (
        (-1, -1, 1.0), (0, -1, 2.0), (1, -1, 1.0),
        (-1, 0, 2.0), (0, 0, 4.0), (1, 0, 2.0),
        (-1, 1, 1.0), (0, 1, 2.0), (1, 1, 1.0),
    )
    for index, count in source.items():
        iy, ix = divmod(index, geometry.grid_nx)
        valid_neighbors = [
            (ix + dx, iy + dy, weight)
            for dx, dy, weight in kernel
            if 0 <= ix + dx < geometry.grid_nx and 0 <= iy + dy < geometry.grid_ny
        ]
        weight_sum = sum(item[2] for item in valid_neighbors)
        for nx, ny, weight in valid_neighbors:
            result[ny * geometry.grid_nx + nx] += count * weight / weight_sum
    return result


def normalize_counter(source: Counter) -> Dict[int, float]:
    total = float(sum(source.values()))
    if total <= 0.0:
        return {}
    return {index: value / total for index, value in source.items() if value > 0.0}


def mix_distributions(local: Dict[int, float], global_dist: Dict[int, float], local_weight: float) -> Dict[int, float]:
    result: Dict[int, float] = {}
    for index in set(local) | set(global_dist):
        result[index] = local_weight * local.get(index, 0.0) + (1.0 - local_weight) * global_dist.get(index, 0.0)
    total = sum(result.values())
    return {index: value / total for index, value in result.items()} if total > 0 else {}


def candidate_list(distribution: Dict[int, float], geometry: Geometry, top_k: int) -> List[dict]:
    ranked = sorted(distribution.items(), key=lambda item: (-item[1], item[0]))[:top_k]
    output = []
    for index, probability in ranked:
        x, y = geometry.grid_center(index)
        output.append({
            "grid_index": index,
            "x": round(x, 3),
            "y": round(y, 3),
            "p": round(probability, 8),
        })
    retained = sum(item["p"] for item in output)
    if retained > 0:
        for item in output:
            item["p"] = round(item["p"] / retained, 8)
    return output


def retained_probability_mass(distribution: Dict[int, float], top_k: int) -> float:
    """Return how much of the unpruned distribution is retained by top-k."""
    return round(sum(sorted(distribution.values(), reverse=True)[:top_k]), 8)


def train_and_collect(
    con: sqlite3.Connection,
    config: dict,
    geometry: Geometry,
    split_lookup: Dict[int, str],
) -> Tuple[dict, Counter, Dict[Tuple[str, str, int], Reservoir]]:
    global_counts: Dict[Tuple[str, str], Counter] = defaultdict(Counter)
    transition_counts: Dict[Tuple[str, str, int, int], Counter] = defaultdict(Counter)
    transition_samples: Counter = Counter()
    stay_counts: Counter = Counter()
    quality: Counter = Counter()

    rng = random.Random(int(config["split_seed"]) + 1)
    reservoirs: Dict[Tuple[str, str, int], Reservoir] = {}
    for split in ("validation", "test"):
        for role in SPECIFIC_ROLES:
            for horizon in config["horizons_seconds"]:
                reservoirs[(split, role, int(horizon))] = Reservoir(
                    int(config["evaluation_samples_per_role_horizon"]), rng
                )

    horizons = tuple(int(value) for value in config["horizons_seconds"])
    lookback = int(config["velocity_lookback_seconds"])
    stride = int(config["evaluation_stride_seconds"])
    stay_radius = float(config["stay_radius_m"])

    for key, raw_rows in iter_raw_sequences(con):
        game_id, robot_id, _, _ = key
        role = ROLE_BY_ROBOT_ID[robot_id]
        split = split_lookup[game_id]
        quality[f"raw_sequences_{split}"] += 1
        for segment in clean_segments(key, raw_rows, geometry, config, quality):
            quality[f"segments_{split}"] += 1
            quality[f"segment_points_{split}"] += len(segment)
            by_t = {point.t: point for point in segment}

            if split == "train":
                for point in segment:
                    context = phase_name(config, point.t)
                    for variant in role_variants(role):
                        endpoint = geometry.grid_index(point.x, point.y)
                        global_counts[(variant, "all_phase")][endpoint] += 1
                        global_counts[(variant, context)][endpoint] += 1

                        zone = geometry.zone_index(point.x, point.y)
                        for horizon in horizons:
                            future = by_t.get(point.t + horizon)
                            if future is None:
                                continue
                            future_index = geometry.grid_index(future.x, future.y)
                            for model_context in ("all_phase", context):
                                transition_counts[(variant, model_context, zone, horizon)][future_index] += 1
                                transition_samples[(variant, model_context, zone, horizon)] += 1
                                if math.hypot(future.x - point.x, future.y - point.y) <= stay_radius:
                                    stay_counts[(variant, model_context, zone, horizon)] += 1
            else:
                for point in segment:
                    if point.t % stride != 0:
                        continue
                    previous = by_t.get(point.t - lookback)
                    if previous is None:
                        continue
                    for horizon in horizons:
                        future = by_t.get(point.t + horizon)
                        if future is None:
                            continue
                        reservoirs[(split, role, horizon)].add(EvalSample(
                            role=role,
                            t=point.t,
                            horizon=horizon,
                            current_x=point.x,
                            current_y=point.y,
                            previous_x=previous.x,
                            previous_y=previous.y,
                            truth_x=future.x,
                            truth_y=future.y,
                        ))

    return {
        "global_counts": global_counts,
        "transition_counts": transition_counts,
        "transition_samples": transition_samples,
        "stay_counts": stay_counts,
    }, quality, reservoirs


def build_model(training: dict, config: dict, geometry: Geometry, metadata: dict) -> dict:
    global_counts = training["global_counts"]
    transition_counts = training["transition_counts"]
    transition_samples = training["transition_samples"]
    stay_counts = training["stay_counts"]
    top_k = int(config["top_candidates"])
    shrinkage = float(config["shrinkage_samples"])
    minimum = int(config["minimum_export_samples"])

    contexts = ["all_phase"] + [str(item["name"]) for item in config["phase_ranges"]]
    global_distributions: Dict[Tuple[str, str], Dict[int, float]] = {}
    for role in MODEL_ROLES:
        for context in contexts:
            counts = global_counts.get((role, context), Counter())
            if not counts and context != "all_phase":
                counts = global_counts.get((role, "all_phase"), Counter())
            global_distributions[(role, context)] = normalize_counter(smooth_counter(counts, geometry))

    model = {
        "metadata": metadata,
        "geometry": {
            "field_size": [geometry.field_x, geometry.field_y],
            "current_zone_size": [geometry.zone_x, geometry.zone_y],
            "current_zone_shape": [geometry.zone_nx, geometry.zone_ny],
            "endpoint_grid_size": [geometry.grid_x, geometry.grid_y],
            "endpoint_grid_shape": [geometry.grid_nx, geometry.grid_ny],
            "canonical_team": "red",
            "blue_transform": "x'=28-x,y'=15-y",
        },
        "role_aliases": DISPLAY_NAMES,
        "horizons_seconds": [int(value) for value in config["horizons_seconds"]],
        "default_context": "all_phase",
        "roles": {},
    }

    for role in MODEL_ROLES:
        role_output = {"contexts": {}}
        for context in contexts:
            global_dist = global_distributions[(role, context)]
            original_global = global_counts.get((role, context), Counter())
            context_output = {
                "global": {
                    "samples": int(sum(original_global.values())),
                    "retained_probability_mass": retained_probability_mass(global_dist, top_k),
                    "candidates": candidate_list(global_dist, geometry, top_k),
                },
                "zones": {},
            }
            for zone in range(geometry.zone_nx * geometry.zone_ny):
                zone_output = {}
                for horizon in config["horizons_seconds"]:
                    horizon = int(horizon)
                    key = (role, context, zone, horizon)
                    samples = int(transition_samples.get(key, 0))
                    if samples < minimum:
                        continue
                    local_dist = normalize_counter(smooth_counter(transition_counts[key], geometry))
                    local_weight = samples / (samples + shrinkage)
                    mixed = mix_distributions(local_dist, global_dist, local_weight)
                    zone_output[f"h{horizon}"] = {
                        "samples": samples,
                        "local_weight": round(local_weight, 6),
                        "stay_probability": round(stay_counts.get(key, 0) / samples, 6),
                        "retained_probability_mass": retained_probability_mass(mixed, top_k),
                        "candidates": candidate_list(mixed, geometry, top_k),
                    }
                if zone_output:
                    context_output["zones"][str(zone)] = zone_output
            role_output["contexts"][context] = context_output
        model["roles"][role] = role_output
    return model


def candidates_for(model: dict, geometry: Geometry, role: str, context: str, x: float, y: float, horizon: int, conditional: bool) -> List[dict]:
    role_data = model["roles"].get(role)
    if role_data is None:
        return []
    context_data = role_data["contexts"].get(context) or role_data["contexts"]["all_phase"]
    if conditional:
        zone = str(geometry.zone_index(x, y))
        horizon_data = context_data["zones"].get(zone, {}).get(f"h{horizon}")
        if horizon_data:
            return horizon_data["candidates"]
    return context_data["global"]["candidates"]


def predict_from_candidates(candidates: Sequence[dict], cv_x: float, cv_y: float, sigma: float) -> Tuple[float, float]:
    if not candidates:
        return cv_x, cv_y
    scores = []
    for candidate in candidates:
        distance2 = (candidate["x"] - cv_x) ** 2 + (candidate["y"] - cv_y) ** 2
        scores.append(math.log(max(float(candidate["p"]), 1e-12)) - distance2 / (2.0 * sigma * sigma))
    maximum = max(scores)
    weights = [math.exp(score - maximum) for score in scores]
    total = sum(weights)
    return (
        sum(weight * candidate["x"] for weight, candidate in zip(weights, candidates)) / total,
        sum(weight * candidate["y"] for weight, candidate in zip(weights, candidates)) / total,
    )


def evaluate(model: dict, geometry: Geometry, config: dict, reservoirs: Dict[Tuple[str, str, int], Reservoir], split: str) -> dict:
    speed_cap = float(config["prediction_speed_cap_mps"])
    lookback = float(config["velocity_lookback_seconds"])
    metrics: Dict[Tuple[str, int, str], List[float]] = defaultdict(list)
    seen = Counter()

    for role in SPECIFIC_ROLES:
        for horizon_value in config["horizons_seconds"]:
            horizon = int(horizon_value)
            reservoir = reservoirs[(split, role, horizon)]
            seen[(role, horizon)] = reservoir.seen
            sigma = float(config["motion_sigma_m"][str(horizon)])
            motion_gate = float(config["motion_gate_mps"][str(horizon)])
            for sample in reservoir.items:
                vx = (sample.current_x - sample.previous_x) / lookback
                vy = (sample.current_y - sample.previous_y) / lookback
                speed = math.hypot(vx, vy)
                if speed > speed_cap:
                    scale = speed_cap / speed
                    vx *= scale
                    vy *= scale
                cv_x = min(geometry.field_x, max(0.0, sample.current_x + vx * horizon))
                cv_y = min(geometry.field_y, max(0.0, sample.current_y + vy * horizon))

                global_candidates = candidates_for(
                    model, geometry, role, "all_phase", sample.current_x, sample.current_y, horizon, False
                )
                local_candidates = candidates_for(
                    model, geometry, role, "all_phase", sample.current_x, sample.current_y, horizon, True
                )
                global_x, global_y = predict_from_candidates(global_candidates, cv_x, cv_y, sigma)
                local_x, local_y = predict_from_candidates(local_candidates, cv_x, cv_y, sigma)

                predictions = {
                    "hold": (sample.current_x, sample.current_y),
                    "constant_velocity": (cv_x, cv_y),
                    "global_prior": (global_x, global_y),
                    "conditional_prior": (local_x, local_y),
                    "motion_gated_prior": (
                        (local_x, local_y) if speed > motion_gate else (sample.current_x, sample.current_y)
                    ),
                }
                for method, (px, py) in predictions.items():
                    error = math.hypot(px - sample.truth_x, py - sample.truth_y)
                    metrics[(role, horizon, method)].append(error)

    def summarize(values: Sequence[float]) -> dict:
        return {
            "samples": len(values),
            "mean_m": round(statistics.fmean(values), 4) if values else None,
            "median_m": round(statistics.median(values), 4) if values else None,
            "p90_m": round(percentile(values, 0.90), 4) if values else None,
            "within_1m": round(sum(value <= 1.0 for value in values) / len(values), 4) if values else None,
            "within_2m": round(sum(value <= 2.0 for value in values) / len(values), 4) if values else None,
        }

    output = {"split": split, "roles": {}, "overall": {}}
    methods = ("hold", "constant_velocity", "global_prior", "conditional_prior", "motion_gated_prior")
    for role in SPECIFIC_ROLES:
        role_output = {}
        for horizon_value in config["horizons_seconds"]:
            horizon = int(horizon_value)
            role_output[f"h{horizon}"] = {
                "available_samples_before_reservoir": seen[(role, horizon)],
                "methods": {method: summarize(metrics[(role, horizon, method)]) for method in methods},
            }
        output["roles"][role] = role_output

    for horizon_value in config["horizons_seconds"]:
        horizon = int(horizon_value)
        output["overall"][f"h{horizon}"] = {
            method: summarize([
                value
                for role in SPECIFIC_ROLES
                for value in metrics[(role, horizon, method)]
            ])
            for method in methods
        }
    return output


def golden_cases(model: dict, geometry: Geometry, reservoirs: Dict[Tuple[str, str, int], Reservoir], limit: int = 24) -> list:
    cases = []
    for role in SPECIFIC_ROLES:
        for horizon in model["horizons_seconds"]:
            items = reservoirs[("test", role, int(horizon))].items
            if not items:
                continue
            sample = items[0]
            candidates = candidates_for(
                model, geometry, role, "all_phase", sample.current_x, sample.current_y, int(horizon), True
            )
            cases.append({
                "role": role,
                "context": "all_phase",
                "horizon_seconds": int(horizon),
                "canonical_current": [round(sample.current_x, 6), round(sample.current_y, 6)],
                "expected_zone_index": geometry.zone_index(sample.current_x, sample.current_y),
                "expected_candidates": candidates[:5],
            })
            if len(cases) >= limit:
                return cases
    return cases


def cleaning_report(quality: Counter) -> dict:
    return {
        "counts": dict(sorted(quality.items())),
        "rules": {
            "zero_zero_removed": True,
            "field_bounds": "0<=x<=28, 0<=y<=15",
            "blue_normalization": "x'=28-x,y'=15-y",
            "maximum_speed_mps": quality.get("configured_max_speed", None),
            "invalid_points_break_segments": True,
            "time_gaps_break_segments": True,
        },
    }


def build_command(args: argparse.Namespace) -> int:
    db_path = Path(args.db)
    config_path = Path(args.config)
    output_dir = Path(args.output_dir)
    if not db_path.is_file():
        raise FileNotFoundError(f"SQLite file not found: {db_path}")
    with config_path.open("r", encoding="utf-8") as handle:
        config = json.load(handle)
    geometry = Geometry(config)
    output_dir.mkdir(parents=True, exist_ok=True)

    started = time.time()
    print("[1/6] Scanning database...", flush=True)
    con = open_readonly(db_path)
    scan = scan_database(con, db_path)
    write_json(output_dir / "01_scan_report.json", scan)

    print("[2/6] Creating deterministic game split...", flush=True)
    split_report, split_lookup = game_splits(con, config)
    write_json(output_dir / "02_game_split.json", split_report)

    print("[3/6] Cleaning trajectories and counting transitions...", flush=True)
    training, quality, reservoirs = train_and_collect(con, config, geometry, split_lookup)
    quality["configured_max_speed"] = float(config["max_physical_speed_mps"])
    write_json(output_dir / "03_cleaning_report.json", cleaning_report(quality))

    print("[4/6] Hashing source database and exporting sparse model...", flush=True)
    metadata = {
        "model_version": int(config["model_version"]),
        "season": int(config["season"]),
        "map_id": str(config["map_id"]),
        "created_utc_epoch": time.time(),
        "database_bytes": db_path.stat().st_size,
        "database_sha256": sha256_file(db_path),
        "split_seed": int(config["split_seed"]),
        "training_games": split_report["counts"]["train"],
        "validation_games": split_report["counts"]["validation"],
        "test_games": split_report["counts"]["test"],
        "model_semantics": "role/current-zone/horizon endpoint prior",
        "file_syntax": "JSON document compatible with YAML 1.2 and yaml-cpp",
    }
    # Keep the model itself bit-for-bit reproducible. The wall-clock creation
    # time belongs in the external metadata report, not in the runtime asset.
    runtime_metadata = dict(metadata)
    runtime_metadata.pop("created_utc_epoch", None)
    model = build_model(training, config, geometry, runtime_metadata)
    model_path = output_dir / "04_rmuc2026_position_prior_v1.yaml"
    write_json(model_path, model, pretty=False)
    write_json(output_dir / "04_model_metadata.json", metadata)
    model_sha256 = hashlib.sha256(model_path.read_bytes()).hexdigest()
    (output_dir / "04_model.sha256").write_text(
        f"{model_sha256}  {model_path.name}\n", encoding="ascii", newline="\n"
    )

    print("[5/6] Evaluating simulated losses...", flush=True)
    validation = evaluate(model, geometry, config, reservoirs, "validation")
    test = evaluate(model, geometry, config, reservoirs, "test")
    evaluation = {
        "methodology": {
            "split_unit": "game_id",
            "loss_horizons_seconds": config["horizons_seconds"],
            "evaluation_stride_seconds": config["evaluation_stride_seconds"],
            "notes": [
                "Public SQLite positions are referee telemetry, not camera detections.",
                "Losses are simulated by hiding future ground-truth positions.",
                "Results measure behavioral-prior quality, not camera visibility quality."
            ],
        },
        "validation": validation,
        "test": test,
    }
    write_json(output_dir / "05_evaluation_report.json", evaluation)

    print("[6/6] Exporting cross-platform golden cases...", flush=True)
    write_json(output_dir / "06_golden_cases.json", golden_cases(model, geometry, reservoirs))
    con.close()

    summary = {
        "model": model_path.name,
        "model_bytes": model_path.stat().st_size,
        "model_sha256": model_sha256,
        "elapsed_seconds": round(time.time() - started, 3),
        "artifacts": sorted(
            {path.name for path in output_dir.iterdir() if path.is_file()} | {"00_run_summary.json"}
        ),
        "test_overall": test["overall"],
    }
    write_json(output_dir / "00_run_summary.json", summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
    return 0


def scan_command(args: argparse.Namespace) -> int:
    db_path = Path(args.db)
    con = open_readonly(db_path)
    report = scan_database(con, db_path)
    con.close()
    if args.output:
        write_json(Path(args.output), report)
    else:
        print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    scan_parser = subparsers.add_parser("scan", help="Scan the SQLite database without building a model")
    scan_parser.add_argument("--db", required=True, help="Path to rmuc_2026_region_dataset.sqlite")
    scan_parser.add_argument("--output", help="Optional JSON report path")
    scan_parser.set_defaults(func=scan_command)

    build_parser = subparsers.add_parser("build", help="Build and evaluate the complete prior model")
    build_parser.add_argument("--db", required=True, help="Path to rmuc_2026_region_dataset.sqlite")
    build_parser.add_argument("--config", required=True, help="Path to builder_config.json")
    build_parser.add_argument("--output-dir", required=True, help="Directory for generated artifacts")
    build_parser.set_defaults(func=build_command)
    return parser


def main() -> int:
    args = make_parser().parse_args()
    try:
        return int(args.func(args))
    except KeyboardInterrupt:
        print("Interrupted", file=sys.stderr)
        return 130
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise


if __name__ == "__main__":
    raise SystemExit(main())
