"""Frozen NumPy candidate provider from the training project; no DB or mesh IO."""
import numpy as np

CANDIDATE_FEATURES = ["dx", "dy", "distance", "enemy_nearest_distance", "enemy_mean_dx",
    "enemy_mean_dy", "enemy_near_fraction", "ally_nearest_distance", "ally_mean_dx",
    "ally_mean_dy", "ally_near_fraction", "region_home", "region_mid", "region_away",
    "reachable", "is_hold", "enemy_present", "ally_present"]


class CandidateGenerator:
    """Replace this provider for tactical/dynamic points, retaining [K,F] features."""
    def __init__(self, artifact, cfg):
        self.artifact = artifact
        self.cfg = cfg
        self.points = np.asarray(artifact["points"], np.float32)
        self.grid = np.asarray(artifact["components"], np.int32)
        self.res = artifact["resolution"]
        self.ids = artifact["candidate_ids"]
        self.hold_index = len(self.points)
        self.point_components = np.array([self.component(p) for p in self.points])

    def component(self, xy):
        if not np.isfinite(xy).all():
            return -1
        c, r = np.floor(np.asarray(xy) / self.res).astype(int)
        if 0 <= r < self.grid.shape[0] and 0 <= c < self.grid.shape[1]:
            return int(self.grid[r, c])
        return -1

    def get(self, ego):
        comp = self.component(ego)
        mask = np.r_[(self.point_components == comp) & (comp >= 0), True]
        points = np.vstack((self.points, np.nan_to_num(ego))).astype(np.float32)
        return points, mask

    def features(self, positions, known, robot_ids, ego_index):
        ego = positions[ego_index]
        points, reachable = self.get(ego if known[ego_index] else np.array([np.nan, np.nan]))
        field = np.asarray(self.cfg["data"]["field_size"])
        diag = float(np.linalg.norm(field))
        result = np.zeros((len(points), len(CANDIDATE_FEATURES)), np.float32)
        result[:, :2] = (points - ego) / field
        result[:, 2] = np.linalg.norm(points - ego, axis=1) / diag
        same = (robot_ids > 100) == (robot_ids[ego_index] > 100)
        for start, group, flag in [(3, ~same & known, 16), (7, same & known & (np.arange(len(known)) != ego_index), 17)]:
            if np.any(group):
                rel = positions[group][None, :, :] - points[:, None, :]
                dist = np.linalg.norm(rel, axis=-1)
                result[:, start] = dist.min(1) / diag
                result[:, start+1:start+3] = rel.mean(1) / field
                result[:, start+3] = (dist < self.cfg["candidates"]["relation_radius"]).mean(1)
                result[:, flag] = 1
            else:
                result[:, start] = 1
        x = points[:, 0] / field[0]
        if robot_ids[ego_index] > 100:
            x = 1 - x
        zones = np.clip((x*3).astype(int), 0, 2)
        result[np.arange(len(points)), 11 + zones] = 1
        result[:, 14] = reachable
        result[-1, 15] = 1
        return result, reachable, points
