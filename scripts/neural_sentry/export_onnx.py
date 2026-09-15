#!/usr/bin/env python3
"""Export the trained v1 checkpoint and a self-contained, hashed shadow bundle.

Run with the training Python environment. Does not train or query SQLite.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--training-root', default='/home/delphine/rm/train_shao')
    ap.add_argument('--checkpoint', default='runs/sentry_v2/best.pt')
    ap.add_argument('--output', required=True)
    args = ap.parse_args()
    root = Path(args.training_root).resolve()
    sys.path.insert(0, str(root))
    import numpy as np
    import torch
    import onnx
    import onnxruntime as ort
    from common import fingerprint
    from dataset import BattleDataset
    from training_utils import dataset_signature
    from models import TargetPointModel
    from data.features import HISTORY_FEATURES
    from data.candidate_generator import CANDIDATE_FEATURES

    path = Path(args.checkpoint)
    path = path if path.is_absolute() else root / path
    ckpt = torch.load(path, map_location='cpu', weights_only=True)
    cfg = ckpt['config']
    if not cfg['model'].get('motion_head'):
        raise ValueError('This export requires the v1 HOLD/MOVE head.')
    ds = BattleDataset(cfg, 'val', 'ranking')
    if ckpt['dataset_signature'] != dataset_signature(ds.manifest):
        raise ValueError('Checkpoint and dataset signatures disagree.')
    out = Path(args.output).resolve()
    if out.exists() and any(out.iterdir()):
        raise FileExistsError(f'Preserving nonempty bundle directory: {out}')
    out.mkdir(parents=True, exist_ok=True)
    model = TargetPointModel(cfg).eval()
    model.load_state_dict(ckpt['model_state'], strict=True)
    torch.set_num_threads(1)
    torch.backends.mha.set_fastpath_enabled(False)

    class ExportHeads(torch.nn.Module):
        def __init__(self, trained):
            super().__init__()
            self.trained = trained

        def forward(self, history, robot_mask, candidate_features, ego_index):
            battle = self.trained.encoder(history, robot_mask, ego_index)
            context = battle[:, None].expand(-1, candidate_features.shape[1], -1)
            scores = self.trained.scorer.mlp(torch.cat([context, candidate_features], -1)).squeeze(-1)
            # Reachability and explicit motion gating stay in the checked runtime.
            return self.trained.motion_head(battle), scores[:, :-1]

    names = ['history', 'robot_mask', 'candidate_features', 'ego_index']
    selected = []
    for action in (0, 1):
        match = np.flatnonzero((ds.index['robot_role'][ds.rows] == 'sentry') &
                               (ds.index['motion_label'][ds.rows] == action))
        selected.extend(match[:2].tolist())
    if len(selected) < 2:
        raise ValueError('Need real held-out sentry samples to verify the export.')
    example = ds[selected[0]]['inputs']
    tensors = tuple(example[k].unsqueeze(0) for k in names)
    wrapper = ExportHeads(model).eval()
    onnx_path = out / 'sentry.onnx'
    with torch.no_grad():
        torch.onnx.export(wrapper, tensors, str(onnx_path), input_names=names,
                          output_names=['motion_logits', 'candidate_logits'],
                          opset_version=17, dynamo=False, do_constant_folding=True)
    onnx.checker.check_model(str(onnx_path))
    options = ort.SessionOptions()
    options.intra_op_num_threads = options.inter_op_num_threads = 1
    options.add_session_config_entry('session.intra_op.allow_spinning', '0')
    session = ort.InferenceSession(str(onnx_path), options, providers=['CPUExecutionProvider'])
    checks, replay = [], {}
    for n, item in enumerate(selected):
        sample = ds[item]
        batch = {k: v.unsqueeze(0) for k, v in sample['inputs'].items()}
        feed = {k: batch[k].numpy() for k in names}
        before = time.perf_counter()
        actual = session.run(None, feed)
        elapsed = (time.perf_counter()-before)*1000
        with torch.no_grad():
            expected = wrapper(*(batch[k] for k in names))
            full = model(**batch)
        differences = []
        for a, b in zip(actual, expected):
            np.testing.assert_allclose(a, b.numpy(), rtol=1e-4, atol=1e-4)
            differences.append(float(np.max(np.abs(a-b.numpy()))))
        legal = batch['candidate_mask'].numpy()[0, :-1]
        logits = actual[0][0]
        probs = np.exp(logits-logits.max()); probs /= probs.sum()
        pred = (int(np.where(legal, actual[1][0], -np.inf).argmax())
                if probs[1] >= cfg['model']['move_threshold'] and legal.any() else len(legal))
        if pred != full['predicted_index'].item():
            raise ValueError('ONNX and trained motion-gated decisions disagree.')
        row = sample['row_index']
        game = ds.game(int(ds.index['game_id'][row]))
        t = int(ds.index['time_index'][row]); ego = int(ds.index['ego_index'][row])
        h = int(round(cfg['data']['history_length']/cfg['data']['sample_period']))
        replay[f'positions_{n}'] = game['positions'][t-h:t+1]
        replay[f'observed_{n}'] = game['observed'][t-h:t+1]
        replay[f'ego_index_{n}'] = np.int64(ego)
        replay[f'predicted_index_{n}'] = np.int64(pred)
        for key in names:
            replay[f'{key}_{n}'] = feed[key]
        replay[f'mask_{n}'] = batch['candidate_mask'].numpy()[0]
        checks.append({'dataset_row': row, 'game_id': int(ds.index['game_id'][row]),
                       'robot_id': int(ds.index['robot_id'][row]),
                       'predicted_index': pred, 'inference_ms': elapsed,
                       'maximum_absolute_logit_errors': differences})
    candidate = ds.candidates.artifact
    (out / 'candidates.json').write_text(json.dumps(candidate, ensure_ascii=False), encoding='utf-8')
    np.savez_compressed(out / 'replay_inputs.npz', **replay)
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    manifest = {'version': 1, 'mode': 'shadow_only', 'model': 'sentry.onnx',
        'model_sha256': sha(onnx_path), 'checkpoint_sha256': sha(path),
        'checkpoint': str(path), 'epoch': int(ckpt['epoch']),
        'dataset_signature': ckpt['dataset_signature'],
        'candidate_fingerprint': candidate['fingerprint'],
        'candidate_file_sha256': sha(out/'candidates.json'),
        'history_features': HISTORY_FEATURES, 'candidate_features': CANDIDATE_FEATURES,
        'input_shapes': {k: list(v.shape) for k,v in zip(names, tensors)},
        'coordinate_contract': 'field_x=world_z+14; field_y=world_x+7.5; no team mirroring',
        'runtime_config': {'data': {k: cfg['data'][k] for k in
            ['history_length', 'sample_period', 'max_observation_age', 'max_speed',
             'field_size', 'robot_ids', 'min_history_coverage', 'zero_is_missing']},
            'candidates': {'relation_radius': cfg['candidates']['relation_radius']},
            'move_threshold': cfg['model']['move_threshold']},
        'runtime_sources': {k: sha(root/k) for k in ['data/features.py', 'data/candidate_generator.py']},
        'export_versions': {'torch': torch.__version__, 'onnx': onnx.__version__, 'onnxruntime': ort.__version__}}
    manifest['fingerprint'] = fingerprint(manifest)
    (out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n', encoding='utf-8')
    report = {'status': 'passed', 'checkpoint_epoch': int(ckpt['epoch']), 'samples': checks,
              'note': 'Bounded export/inference equivalence; no training and no tactical-success claim.'}
    (out/'export_report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report, indent=2))
    print(f'Exported self-contained bundle: {out}')


if __name__ == '__main__':
    main()
