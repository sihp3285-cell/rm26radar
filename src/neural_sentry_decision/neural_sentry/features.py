"""Allowlisted causal radar features shared by dataset loading and deployment.

No access to telemetry, events, label, quality_score, timestamps, or future rows.
"""
import numpy as np

ROBOT_TYPES = [1, 2, 3, 4, 6, 7]
HISTORY_FEATURES = ["x", "y", "vx", "vy", "observed", "observation_age", "velocity_known",
    "same_team", "is_ego", "hero", "engineer", "infantry3", "infantry4", "aerial", "sentry",
    "relative_x", "relative_y"]


def causal_observations(positions, observed, cfg):
    """Input contains only rows <= current t; forward-fill expires after configured age."""
    d = cfg["data"]
    n, robots, _ = positions.shape
    filled = np.zeros_like(positions)
    age = np.full((n, robots), d["max_observation_age"] + d["sample_period"], np.float32)
    last = np.zeros((robots, 2), np.float32)
    last_t = np.full(robots, -np.inf)
    for i in range(n):
        good = observed[i] & np.isfinite(positions[i]).all(-1)
        last[good] = positions[i, good]
        last_t[good] = i * d["sample_period"]
        age[i] = i * d["sample_period"] - last_t
        known = age[i] <= d["max_observation_age"]
        filled[i, known] = last[known]
    known = age <= d["max_observation_age"]
    velocity = np.zeros_like(filled)
    velocity_known = np.zeros((n, robots), bool)
    velocity_known[1:] = observed[1:] & observed[:-1]
    velocity[1:] = np.where(velocity_known[1:, :, None], (filled[1:]-filled[:-1]) / d["sample_period"], 0)
    return filled, known, age, velocity, velocity_known


def model_inputs(history_positions, history_observed, robot_ids, ego_index, candidates, cfg):
    """Returns model-only arrays. Caller must slice history before this boundary."""
    positions, known, age, vel, vel_known = causal_observations(history_positions, history_observed, cfg)
    length, robots, _ = positions.shape
    x = np.zeros((length, robots, len(HISTORY_FEATURES)), np.float32)
    field = np.asarray(cfg["data"]["field_size"])
    x[:, :, :2] = positions / field
    x[:, :, 2:4] = vel / cfg["data"]["max_speed"]
    x[:, :, 4] = history_observed
    age_scale = cfg["data"]["max_observation_age"] + cfg["data"]["sample_period"]
    x[:, :, 5] = np.minimum(age / age_scale, 1)
    x[:, :, 6] = vel_known
    x[:, :, 7] = (robot_ids > 100) == (robot_ids[ego_index] > 100)
    x[:, ego_index, 8] = 1
    for i, typ in enumerate(ROBOT_TYPES):
        x[:, :, 9+i] = robot_ids % 100 == typ
    relative = (positions - positions[:, ego_index:ego_index+1]) / field
    relative[~(known & known[:, ego_index:ego_index+1])] = 0
    x[:, :, 15:17] = relative
    cf, reachable, _ = candidates.features(positions[-1], known[-1], robot_ids, ego_index)
    robot_mask = known.any(0)
    # Stable missing-ego token avoids all-masked attention for inspection-only samples.
    robot_mask[ego_index] = True
    return {"history": x.transpose(1, 0, 2).copy(), "robot_mask": robot_mask,
            "candidate_features": cf, "candidate_mask": reachable, "ego_index": np.int64(ego_index)}
