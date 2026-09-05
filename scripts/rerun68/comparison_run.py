#!/usr/bin/env python3
"""Sequential GPU runs, two order-reversed blocks; resume only verified recordings."""
import json,time
from run import run,BASE
from comparison_build import CMP,sha

PLAN=[('g1','baseline',1),('g1','armor',1),('g1','armor',2),('g1','baseline',2),
      ('g3','baseline',1),('g3','fixed',1),('g3','nav_off',1),
      ('g3','nav_off',2),('g3','fixed',2),('g3','baseline',2)]

def main():
    manifest=json.loads((CMP/'manifest.json').read_text())
    assert sha(BASE/'alignment.json')==manifest['alignment_sha256']
    mapping=json.loads((BASE/'alignment.json').read_text())
    (CMP/'plan.json').write_text(json.dumps(PLAN,indent=2))
    for i,(key,variant,rep) in enumerate(PLAN,1):
        phase=f'cmp05_{variant}_r{rep}'
        folder=BASE/f'{phase}_{key}'
        if folder.exists():
            meta=json.loads((folder/'run.json').read_text())
            assert meta['status']=='recorded' and (folder/'export_audit.json').exists(),folder
            print('RESUME verified',folder,flush=True);continue
        (CMP/'progress.json').write_text(json.dumps({'index':i,'total':len(PLAN),'key':key,'variant':variant,'repeat':rep,'folder':str(folder),'started':time.time()}))
        run(key,phase,mapping[key],overrides=manifest['variants'][variant])
    (CMP/'progress.json').write_text(json.dumps({'status':'all_recorded','total':len(PLAN),'finished':time.time()}))

if __name__=='__main__':main()
