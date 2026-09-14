"""Read fusion snapshots using the fixed 5 Hz offline sender policy."""
import collections,json,math
import numpy as np
from analyze import ROLES,stamp_ns

def fusion_frames(folder):
    meta=json.loads((folder/'source.json').read_text());origin=meta['epoch_ns'];offset=meta['offset']
    frames={'world':[],'fused':[]};counts=collections.Counter();flags=collections.Counter()
    with (folder/'events.jsonl').open() as f:
        for line in f:
            e=json.loads(line);topic=e['topic'];m=e['message'];receipt=e['receipt_ns']
            if topic not in ['/world_targets','/fused_targets']:continue
            key='world' if topic=='/world_targets' else 'fused'
            now=(receipt-origin)/1e9+offset;frame={};counts[key]+=1
            if key!='world':
                assert len(m['targets'])==10
                assert [t['slot_idx'] for t in m['targets']]==list(range(10))
            for slot,t in enumerate(m['targets'][:5]):
                if not t['valid'] or t['team_id']!=1:continue
                if key=='world':
                    if not t['observed'] or t['is_dead'] or t['position_source'] not in [1,2]:continue
                    source='tracking';stamp=stamp_ns(m)
                else:
                    assert t['source'] in [1,3],t['source']
                    source='tracking' if t['source']==1 else 'guess'
                    ts=t['source_stamp'];stamp=ts['sec']*10**9+ts['nanosec']
                    assert stamp<=stamp_ns(m) and stamp<=receipt
                assert math.isfinite(t['world_x']) and math.isfinite(t['world_z'])
                frame[slot]={'t':now,'wx':t['world_x'],'wz':t['world_z'],'source':source,
                             'source_stamp_ns':stamp,'receipt_ns':receipt}
                flags[f'{key}_{source}_entries']+=1
            frames[key].append((now,frame))
    for key,fs in frames.items():
        assert fs and all(a[0]<=b[0] for a,b in zip(fs,fs[1:])),key
    streams={}
    for key,fs in frames.items():
        per=[[] for _ in ROLES];i=-1
        for tick in np.arange(0,420,.2):
            while i+1<len(fs) and fs[i+1][0]<=tick+1e-9:i+=1
            if i<0:continue
            for slot,p in fs[i][1].items():
                if tick-((p['source_stamp_ns']-origin)/1e9+offset)>.5:continue
                cx=int(round((p['wz']+14)*100));cy=int(round((p['wx']+7.5)*100))
                if not (0<=cx<=65535 and 0<=cy<=65535) or cx==cy==0:continue
                per[slot].append(dict(p,t=float(tick),wx=cy/100-7.5,wz=cx/100-14,
                                     protocol_x_cm=cx,protocol_y_cm=cy,send_time_ns=origin+round((tick-offset)*1e9)))
        streams[key]=per
    return streams,dict(counts),dict(flags)

