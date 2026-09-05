#!/usr/bin/env python3
"""Fit video-content -> match time ONCE on discovery data, before final replay.

This is a trajectory-derived alignment, not a referee clock measurement. Even
10-second content blocks calibrate; odd blocks independently check residuals.
"""
import argparse,json,pathlib,sqlite3
import numpy as np

ROOT=pathlib.Path(__file__).resolve().parents[2];BASE=ROOT/'log/rerun68'
GIDS={'g1':1778913533396,'g2':1778914365365,'g3':1778915208150}
ROLES=['英雄','工程','步兵3','步兵4','哨兵'];SLOTS=[0,1,2,3,5]

def load_db(gid):
    c=sqlite3.connect('file:/home/delphine/下载/rmuc_2026_region_dataset.sqlite?mode=ro',uri=True)
    out=[]
    for role in ROLES:
        a=np.array(c.execute("select 时刻秒,y-7.5,x-14.0,当前血量,是否易伤 from timeseries where game_id=? and 阵营='红' and 机器人类型=? order by 时刻秒",(gid,role)).fetchall(),dtype=float)
        assert a.shape==(419,5),a.shape
        out.append(a)
    c.close();return out

def observations(key,phase='discovery'):
    folder=BASE/f'{phase}_{key}';meta=json.loads((folder/'source.json').read_text());out=[]
    with open(folder/'events.jsonl') as f:
        for line in f:
            e=json.loads(line)
            if e['topic']!='/radar_map':continue
            m=e['message'];s=m['header']['stamp'];content=(s['sec']*10**9+s['nanosec']-meta['epoch_ns'])/1e9
            for role,slot in enumerate(SLOTS):
                x,y=m['red_x'][slot],m['red_y'][slot]
                if x==y==0:continue
                out.append((content,role,(x-194)/(388/15),(y-361)/(722/28)))
    return np.array(out)

def errors(obs,db,slope,offset):
    t=slope*obs[:,0]+offset;d=np.full(len(t),np.nan)
    for role,a in enumerate(db):
        mask=(obs[:,1]==role)&(t>=1)&(t<=419)
        ids=np.flatnonzero(mask);tt=t[ids]
        alive=a[np.clip(np.searchsorted(a[:,0],tt,side='right')-1,0,len(a)-1),3]>0
        ids=ids[alive];tt=t[ids]
        d[ids]=np.hypot(obs[ids,2]-np.interp(tt,a[:,0],a[:,1]),obs[ids,3]-np.interp(tt,a[:,0],a[:,2]))
    return d

def fit(obs,db):
    # Use calibration blocks only. Bound point count for a reproducible grid search.
    use=obs[(np.floor(obs[:,0]/10).astype(int)%2)==0]
    use=use[::max(1,len(use)//1600)]
    def score(s,b):
        d=errors(use,db,s,b);v=np.isfinite(d)
        # Penalize exclusion so a fit cannot improve by mapping hard frames away.
        return float(np.where(v,np.minimum(np.nan_to_num(d,nan=4.),4.)**2,8.).mean())
    best=(float('inf'),1.,0.)
    for s in np.arange(1.1,2.101,.025):
        for b in np.arange(-100,11,2.):
            sc=score(s,b)
            if sc<best[0]:best=(sc,s,b)
    for sr,ss,br,bs in [(.04,.004,3.,.3),(.006,.0005,.5,.05)]:
        _,s0,b0=best
        for s in np.arange(s0-sr,s0+sr+ss/2,ss):
            for b in np.arange(b0-br,b0+br+bs/2,bs):
                sc=score(s,b)
                if sc<best[0]:best=(sc,s,b)
    return best

def main(key,phase='discovery'):
    obs=observations(key,phase);candidates={}
    for name,gid in GIDS.items():
        result=fit(obs,load_db(gid));candidates[name]={'score':result[0],'slope':result[1],'offset':result[2]}
    chosen=min(candidates,key=lambda k:candidates[k]['score'])
    if chosen!=key:raise RuntimeError(f'video identity mismatch: {key}: {candidates}')
    result=candidates[key];db=load_db(GIDS[key]);d=errors(obs,db,result['slope'],result['offset'])
    valid=np.isfinite(d);holdout=(np.floor(obs[:,0]/10).astype(int)%2)==1
    result.update(gid=GIDS[key],video_identity_scores=candidates.copy(),observations=len(obs),
        valid_observations=int(valid.sum()),rms_m=float(np.sqrt(np.mean(d[valid]**2))),
        median_m=float(np.median(d[valid])),holdout_rms_m=float(np.sqrt(np.mean(d[valid&holdout]**2))),
        method='frozen content-frame affine mapping; even 10s blocks fit, odd blocks held out',
        source=f'{phase}_{key}/events.jsonl')
    # Avoid self-reference from candidates.copy().
    result['video_identity_scores']={k:{x:v[x] for x in ('score','slope','offset')} for k,v in candidates.items()}
    result['local_shift_checks']=[]
    for start in range(0,420,60):
        t=obs[:,0]*result['slope']+result['offset'];sub=obs[(t>=start)&(t<start+60)&holdout]
        if len(sub)<20:continue
        trials=[]
        for shift in np.arange(-3,3.01,.1):
            dd=errors(sub,db,result['slope'],result['offset']+shift)
            trials.append((float(np.nanmean(np.minimum(dd,4.)**2)),float(shift)))
        result['local_shift_checks'].append({'start_s':start,'best_shift_s':min(trials)[1],'n':len(sub)})
    p=BASE/'alignment.json';existing=json.loads(p.read_text()) if p.exists() else {}
    existing[key]=result;p.write_text(json.dumps(existing,ensure_ascii=False,indent=2))
    print(json.dumps(result,ensure_ascii=False,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('key',choices=GIDS);p.add_argument('--phase',default='discovery')
    a=p.parse_args();main(a.key,a.phase)
