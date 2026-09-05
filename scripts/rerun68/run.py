#!/usr/bin/env python3
"""Own-process-only orchestration of production libraries and raw rosbag recording."""
import argparse, hashlib, json, os, pathlib, signal, subprocess, time

ROOT = pathlib.Path(__file__).resolve().parents[2]
BASE = ROOT / 'log/rerun68'

def stop(p):
    if p.poll() is None:
        os.killpg(p.pid, signal.SIGINT)
        try: p.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGTERM)
            try: p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL); p.wait()

def run(key, phase, mapping=None, max_content=None, overrides=None):
    out = BASE / f'{phase}_{key}'
    out.mkdir(exist_ok=False)
    video = pathlib.Path('/home/delphine/rm/car_project/test') / {'g1':'06.mp4','g2':'07.mp4','g3':'08.mp4'}[key]
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID='196', ROS_HOME=str(out/'ros'), ROS_LOG_DIR=str(out/'ros/log'), ROS_LOCALHOST_ONLY='1')
    (out/'ros/log').mkdir(parents=True)
    handles, processes = [], []
    def start(cmd, name):
        f = open(out/(name+'.log'), 'w'); handles.append(f)
        p = subprocess.Popen(cmd, cwd=ROOT, env=env, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(p); return p
    overrides = overrides or {}
    meta = {'phase':phase, 'key':key, 'video':str(video), 'mapping':mapping, 'qt':False, 'overrides':overrides,
            'core_manifest':'../core_before.json', 'started_epoch':time.time(), 'pid':os.getpid()}
    (out/'run.json').write_text(json.dumps(meta,indent=2))
    try:
        recorder = start(['ros2','bag','record','-s','sqlite3','-o',str(out/'bag'),'--topics',
            '/armor_detections','/world_targets','/radar_map','/prior_predictions','/pipeline_timing'], 'record')
        prior = start([str(ROOT/'install/position_prior/lib/position_prior/position_prior_node'), '--ros-args',
            '--params-file',overrides.get('prior_params_file',str(ROOT/'src/position_prior/config/position_prior.yaml')),
            '-p',f'shadow_log_path:={out}/shadow.csv'], 'prior')
        time.sleep(4)
        if recorder.poll() is not None or prior.poll() is not None: raise RuntimeError('recorder/prior startup failed')
        opts = {'video':str(video), 'output':str(out)}
        opts.update({k:v for k,v in overrides.items() if k in ('params_file','pose_library')})
        if phase.startswith('discovery'):
            opts.update(speed='3.0',stride='4')
        else:
            opts.update(slope=str(float(mapping['slope'])),offset=str(float(mapping['offset'])))
            opts['max_content']=str((422.-mapping['offset'])/mapping['slope'])
        if max_content is not None: opts['max_content']=str(float(max_content))
        cmd=[str(BASE/'fixture_build/replay_fixture'),'--ros-args']
        for k,v in opts.items(): cmd += ['-p',f'{k}:={v}']
        fixture = start(cmd,'pipeline')
        print(f'START {phase} {key} fixture_pid={fixture.pid} output={out}',flush=True)
        while fixture.poll() is None:
            if recorder.poll() is not None or prior.poll() is not None:
                raise RuntimeError('recording or prior unexpectedly stopped')
            time.sleep(2)
        if fixture.returncode != 0: raise RuntimeError(f'pipeline exit {fixture.returncode}')
        time.sleep(2)
        meta['status']='recorded'
    except BaseException as e:
        meta['status']='failed';meta['error']=str(e)
        raise
    finally:
        for p in reversed(processes): stop(p)
        for f in handles: f.close()
        meta['finished_epoch']=time.time()
        (out/'run.json').write_text(json.dumps(meta,indent=2))
    export = subprocess.run(['/usr/bin/python3',str(ROOT/'scripts/rerun68/export.py'),str(out)],cwd=ROOT,env=env)
    if export.returncode: raise RuntimeError('direct bag export failed')
    print('COMPLETE',out,flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('phase');p.add_argument('keys',nargs='+');p.add_argument('--max-content',type=float)
    a=p.parse_args()
    mappings=json.loads((BASE/'alignment.json').read_text()) if not a.phase.startswith('discovery') else {}
    for key in a.keys: run(key,a.phase,mappings.get(key),a.max_content)
