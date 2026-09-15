"""NumPy/ONNX-only runtime. No ROS, SQLite, telemetry, or command publication."""
from collections import deque
import hashlib
import json
from pathlib import Path
import threading
import time
import numpy as np
from .features import HISTORY_FEATURES, model_inputs
from .candidates import CANDIDATE_FEATURES, CandidateGenerator


def fingerprint(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,ensure_ascii=False).encode()).hexdigest()


class UnavailableHistory(ValueError):
    pass


class ShadowRuntime:
    def __init__(self, bundle, timeout_s=0.15):
        # Native inference lives only in this expendable process, never in the radar container.
        import onnxruntime as ort
        self.ort = ort
        self.timeout_s = float(timeout_s)
        root = Path(bundle).resolve()
        def read(name, limit):
            p = root / name
            if p.stat().st_size > limit:
                raise ValueError(f'Oversize bundle file: {name}')
            return p.read_bytes()
        self.manifest = m = json.loads(read('manifest.json', 262144))
        if m.get('version') != 1 or m.get('mode') != 'shadow_only':
            raise ValueError('Unsupported shadow bundle')
        if fingerprint({k:v for k,v in m.items() if k != 'fingerprint'}) != m['fingerprint']:
            raise ValueError('Bundle manifest checksum mismatch')
        if m['history_features'] != HISTORY_FEATURES or m['candidate_features'] != CANDIDATE_FEATURES:
            raise ValueError('Runtime feature contract mismatch; re-export required')
        if m['coordinate_contract'] != 'field_x=world_z+14; field_y=world_x+7.5; no team mirroring':
            raise ValueError('Unsupported coordinate contract')
        raw = read('candidates.json', 2097152)
        if hashlib.sha256(raw).hexdigest() != m['candidate_file_sha256']:
            raise ValueError('Candidate file checksum mismatch')
        artifact = json.loads(raw)
        if (fingerprint({k:v for k,v in artifact.items() if k!='fingerprint'}) != artifact['fingerprint'] or
                artifact['fingerprint'] != m['candidate_fingerprint']):
            raise ValueError('Candidate identity mismatch')
        self.cfg = m['runtime_config']
        self.robot_ids = np.asarray(self.cfg['data']['robot_ids'], np.int64)
        if self.robot_ids.tolist() != [1,2,3,4,6,7,101,102,103,104,106,107]:
            raise ValueError('Unsupported robot slot contract')
        if self.cfg['data']['field_size'] != [28.0,15.0]:
            raise ValueError('Unsupported field dimensions')
        self.candidates = CandidateGenerator(artifact, self.cfg)
        if self.candidates.hold_index != 64 or np.any(self.candidates.point_components < 0):
            raise ValueError('Expected the trained 64 legal candidates + HOLD')
        self.steps = int(round(self.cfg['data']['history_length']/self.cfg['data']['sample_period']))+1
        expected_shapes = {'history':[1,12,self.steps,len(HISTORY_FEATURES)], 'robot_mask':[1,12],
                           'candidate_features':[1,65,len(CANDIDATE_FEATURES)], 'ego_index':[1]}
        if m['input_shapes'] != expected_shapes:
            raise ValueError('Bundle input shapes do not match runtime')
        model = read('sentry.onnx', 32*1024*1024)
        if hashlib.sha256(model).hexdigest() != m['model_sha256']:
            raise ValueError('ONNX checksum mismatch')
        options = ort.SessionOptions()
        options.intra_op_num_threads = options.inter_op_num_threads = 1
        options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        options.add_session_config_entry('session.intra_op.allow_spinning', '0')
        options.add_session_config_entry('session.inter_op.allow_spinning', '0')
        self.session = ort.InferenceSession(model, options, providers=['CPUExecutionProvider'])
        if {x.name:x.shape for x in self.session.get_inputs()} != expected_shapes:
            raise ValueError('ONNX input signature mismatch')
        if [(x.name,x.shape) for x in self.session.get_outputs()] != [('motion_logits',[1,2]),('candidate_logits',[1,64])]:
            raise ValueError('ONNX output signature mismatch')

    def predict(self, positions, observed, ego_index):
        if positions.shape != (self.steps,12,2) or observed.shape != (self.steps,12):
            raise ValueError('Invalid causal history shape')
        if not observed[-1,ego_index] or not np.isfinite(positions[-1,ego_index]).all():
            raise UnavailableHistory('Sentry is not currently observed')
        if observed[:,ego_index].mean() < self.cfg['data']['min_history_coverage']:
            raise UnavailableHistory('Insufficient observed sentry history')
        both = observed[1:,ego_index] & observed[:-1,ego_index]
        speeds = np.linalg.norm(np.diff(positions[:,ego_index],axis=0),axis=1)/self.cfg['data']['sample_period']
        if np.any(speeds[both] > self.cfg['data']['max_speed']):
            raise UnavailableHistory('Abnormal sentry motion / identity transition')
        inputs = model_inputs(positions,observed,self.robot_ids,ego_index,self.candidates,self.cfg)
        if not inputs['candidate_mask'][:-1].any():
            raise UnavailableHistory('Sentry is outside the trained traversable candidate component')
        feed = {k: np.asarray(inputs[k])[None] for k in self.manifest['input_shapes']}
        if any(not np.isfinite(v).all() for v in feed.values()):
            raise ValueError('Nonfinite visual input')
        run_options = self.ort.RunOptions()
        def cancel():
            run_options.terminate = True
        watchdog = threading.Timer(self.timeout_s, cancel)
        watchdog.daemon = True
        before = time.monotonic()
        watchdog.start()
        try:
            motion, ranking = self.session.run(None,feed,run_options=run_options)
        finally:
            watchdog.cancel()
        elapsed = time.monotonic()-before
        if elapsed > self.timeout_s:
            raise TimeoutError('Inference deadline exceeded; result discarded')
        if not np.isfinite(motion).all() or not np.isfinite(ranking).all():
            raise ValueError('Nonfinite model output')
        probabilities = np.exp(motion[0]-motion[0].max()); probabilities /= probabilities.sum()
        legal = inputs['candidate_mask'][:-1]
        masked = np.where(legal, ranking[0], -np.inf)
        conditional = np.exp(masked-masked.max()); conditional /= conditional.sum()
        move = probabilities[1] >= self.cfg['move_threshold']
        choice = int(masked.argmax()) if move else self.candidates.hold_index
        field_xy = self.candidates.points[choice] if move else positions[-1,ego_index]
        if not np.isfinite(field_xy).all() or not np.all((field_xy >= 0)&(field_xy <= [28,15])):
            raise ValueError('Invalid suggested coordinate')
        return {'candidate_index':choice,'candidate_id':self.candidates.ids[choice],
                'action':'MOVE' if move else 'HOLD','field_x':float(field_xy[0]),'field_y':float(field_xy[1]),
                'world_x':float(field_xy[1]-7.5),'world_z':float(field_xy[0]-14),
                'ego_world_x':float(positions[-1,ego_index,1]-7.5),
                'ego_world_z':float(positions[-1,ego_index,0]-14),
                'move_probability':float(probabilities[1]),
                'bc_score':float(probabilities[1]*conditional[choice] if move else probabilities[0]),
                'inference_ms':elapsed*1000}


class HistoryBuffer:
    """Bounded source-time snapshots; each grid tick reads only an earlier frame."""
    def __init__(self, config, snapshot_tolerance_s=0.25):
        self.cfg = config['data']
        self.tolerance = snapshot_tolerance_s
        self.frames = deque(maxlen=2048)

    def clear(self):
        self.frames.clear()

    def append(self, stamp, positions, observed):
        if not np.isfinite(stamp) or stamp <= 0:
            raise ValueError('Invalid source timestamp')
        if self.frames and stamp <= self.frames[-1][0]:
            if stamp < self.frames[-1][0]-0.5:  # replay seek / clock restart
                self.clear()
            else:
                return False
        self.frames.append((stamp,positions.copy(),observed.copy()))
        minimum = stamp-self.cfg['history_length']-self.tolerance-1
        while self.frames and self.frames[0][0] < minimum:
            self.frames.popleft()
        return True

    def sample(self):
        if not self.frames:
            raise ValueError('Waiting for visual observations')
        t = self.frames[-1][0]
        if t-self.frames[0][0]+1e-6 < self.cfg['history_length']:
            raise ValueError('Warming up causal history (10 seconds)')
        count = int(round(self.cfg['history_length']/self.cfg['sample_period']))+1
        grid = t-np.arange(count-1,-1,-1)*self.cfg['sample_period']
        frames = list(self.frames)
        times = np.asarray([f[0] for f in frames])
        indices = np.searchsorted(times,grid+1e-8,side='right')-1
        p = np.full((count,12,2),np.nan,np.float32)
        o = np.zeros((count,12),bool)
        for j,k in enumerate(indices):
            if k >= 0 and grid[j]-times[k] <= self.tolerance:
                p[j],o[j] = frames[k][1],frames[k][2]
        return t,p,o
