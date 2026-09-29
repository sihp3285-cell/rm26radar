import json
from pathlib import Path
import threading
import time
from urllib.request import urlopen

from radar27_interfaces.msg import (WorldTarget, WorldTargetArray, WorldMeasurement,
    WorldMeasurementArray, PriorPrediction, PriorPredictionArray, PriorCandidate,
    FusedTarget, FusedTargetArray, PipelineTiming, MatchState)
from radar27_dashboard.adapters import vision
from radar27_dashboard.dashboard_state import DashboardState
from radar27_dashboard.http_server import DashboardServer


def test_invalid_and_nonfinite_positions_are_not_origin():
    msg = WorldMeasurementArray(measurements=[
        WorldMeasurement(valid=False, world_x=0.0, world_z=0.0),
        WorldMeasurement(valid=True, world_x=float('nan'), world_z=1.0),
        WorldMeasurement(valid=True, world_x=1.0, world_z=float('inf')),
        WorldMeasurement(valid=True, world_x=0.0, world_z=0.0)])
    points = [m['position'] for m in vision.measurements(msg)]
    assert points == [None, None, None, [0.0, 0.0]]
    json.dumps(points, allow_nan=False)


def test_tracking_preserves_raw_measurement_and_identity():
    t = WorldTarget(track_id=17, class_id=3, team_id=1, valid=True,
                    tracking_state=2, position_source=3, observed=False,
                    world_x=1.0, world_z=2.0, measurement_x=0.5, measurement_z=1.5,
                    measurement_covariance_valid=True, lost_duration_s=0.2)
    item = vision.tracking(WorldTargetArray(targets=[t]))[0]
    assert item['key'] == '0:17:1:3'
    assert item['tracking_state'] == 'PREDICTED'
    assert item['observed'] is False
    assert item['measurement_position'] == [0.5, 1.5]
    assert item['position'] == [1.0, 2.0]
    t.valid = False
    assert vision.tracking(WorldTargetArray(targets=[t]))[0]['position'] is None


def test_prior_early_rejection_keeps_reason_not_default_positions():
    p = PriorPrediction(track_id=17, role_class_id=3, team_id=1,
                        rejection_code=1, rejection_reason='waiting_for_prior_window')
    p.last_observed_time.sec = 123
    p.last_world_x = 1.0
    item = vision.priors(PriorPredictionArray(predictions=[p]))[0]
    assert item['position'] is None
    assert item['tracker_position'] is None
    assert item['last_position'] == [1.0, 0.0]
    assert item['rejection_reason'] == 'waiting_for_prior_window'
    assert item['key'] == '0:17:1:3'


def test_prior_candidates_and_fusion_keep_upstream_values():
    p = PriorPrediction(valid=True, prior_world_x=4.0, prior_world_z=-2.0,
                        horizon_seconds=3, motion_gated=True, blind_zone_biased=True,
                        candidates=[PriorCandidate(world_x=2.0, world_z=3.0,
                            reachable=False, blocked=True, from_blind_zone=True)])
    item = vision.priors(PriorPredictionArray(predictions=[p]))[0]
    assert item['position'] == [4.0, -2.0]
    assert item['motion_gate'] and item['blind_zone_bias']
    assert item['candidates'][0]['blocked']
    f = FusedTarget(track_id=17, valid=False, source=0)
    assert vision.fused(FusedTargetArray(targets=[f]))[0]['position'] is None
    f.valid, f.source, f.world_x, f.world_z = True, 3, 4.0, -2.0
    assert vision.fused(FusedTargetArray(targets=[f]))[0]['source'] == 'PRIOR'


def test_state_bounded_and_precision_and_freshness():
    state = DashboardState(stale_after=0.01)
    for i in range(100):
        m = WorldTargetArray(calibration_version=18446744073709551610)
        m.header.stamp.sec, m.header.stamp.nanosec = 1700000000, i
        state.update('tracking', m)
    assert len(state.samples) == 1
    s = state.snapshot()
    assert s['sources']['tracking']['calibration_version'] == '18446744073709551610'
    assert s['sources']['tracking']['stamp_ns'] == '1700000000000000099'
    assert s['modules']['Tracking']['status'] == 'LIVE'
    assert s['modules']['LiDAR']['status'] == 'NOT IMPLEMENTED'
    time.sleep(0.015)
    assert state.snapshot()['modules']['Tracking']['status'] == 'STALE'


def test_performance_nonfinite_and_disabled_prior():
    state = DashboardState()
    state.update('timing', PipelineTiming(fps=float('nan'), dropped_count=123))
    state.update('prior', PriorPredictionArray(model_enabled=False, model_status='missing'))
    snap = state.snapshot()
    assert snap['modules']['Prior']['status'] == 'DISABLED'
    perf = state.snapshot('performance')['performance']
    assert perf['data']['fps'] is None
    assert perf['data']['dropped_count'] == 123
    json.dumps(snap, allow_nan=False)


def test_own_team_is_durable_configuration_not_inferred_from_target_color():
    state = DashboardState(stale_after=0.001)
    assert state.snapshot()['match']['own_team'] is None
    state.update('match', MatchState(own_team=2, revision=1))
    time.sleep(0.005)
    assert state.snapshot()['match'] == {'own_team': 2, 'revision': '1'}
    state.update('match', MatchState(own_team=1, revision=2))
    assert state.snapshot()['match']['own_team'] == 1
    state.update('match', MatchState(own_team=0))
    assert state.snapshot()['match']['own_team'] is None


def test_http_snapshot_stream_disconnect_and_static_map(tmp_path):
    state = DashboardState()
    state.update('timing', PipelineTiming(fps=20.0))
    root = Path(__file__).resolve().parents[1] / 'web'
    map_file = tmp_path / 'map.png'
    map_file.write_bytes(b'Qt same bytes')
    server = DashboardServer(('127.0.0.1', 0), state, root, map_path=map_file)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    url = f'http://127.0.0.1:{server.server_port}'
    try:
        with urlopen(url+'/api/state') as response:
            assert json.load(response)['sources']['timing']['data']['fps'] == 20.0
        with urlopen(url+'/map.png') as response:
            assert response.read() == b'Qt same bytes'
        with urlopen(url+'/') as response:
            assert b'Radar27' in response.read()
        for name in ('radar', 'performance'):
            with urlopen(url+f'/api/{name}-stream') as response:
                assert response.headers['Content-Type'].startswith('text/event-stream')
                assert response.readline() == b'retry: 2000\n'
                response.readline()
                assert response.readline() == f'event: {name}\n'.encode()
                data = response.readline()
                assert data.startswith(b'data: ')
                json.loads(data[6:])
        time.sleep(2.1)
        # Every request slot returns even when clients close mid-stream.
        acquired = [server.client_slots.acquire(blocking=False) for _ in range(16)]
        assert all(acquired)
        for _ in acquired:
            server.client_slots.release()
    finally:
        server.close()
        worker.join(timeout=2)
