#!/usr/bin/env python3
"""Audit the actual measurement R published by the tracker, directly from each bag."""
import collections,json,sqlite3
from rclpy.serialization import deserialize_message
from tensorrt_detect_msgs.msg import WorldTargetArray
from comparison_run import PLAN
from comparison_analyze import folder_for

def inspect(key,variant,rep):
    folder=folder_for(key,variant,rep);counts=collections.Counter();examples=set()
    assert json.loads((folder/'run.json').read_text())['status']=='recorded'
    assert (folder/'export_audit.json').exists(),'Never query a bag while its recorder is writing.'
    for path in (folder/'bag').glob('*.db3'):
        con=sqlite3.connect('file:'+str(path)+'?mode=ro',uri=True)
        for (data,) in con.execute("select m.data from messages m join topics t on t.id=m.topic_id where t.name='/world_targets'"):
            msg=deserialize_message(data,WorldTargetArray)
            for p in msg.targets:
                if not p.measurement_covariance_valid:continue
                r=list(p.measurement_covariance);counts['valid_R']+=1
                if all(abs(a-b)<1e-5 for a,b in zip(r,[1,0,0,1])):counts['identity_R']+=1
                elif abs(r[1])<1e-5 and abs(r[2])<1e-5 and abs(r[0]-r[3])<1e-5:counts['other_isotropic_R']+=1
                else:counts['anisotropic_R']+=1
                if len(examples)<12:examples.add(tuple(round(v,6) for v in r))
        con.close()
    assert counts['valid_R']>0
    if variant=='fixed':assert counts['identity_R']>0 and counts['anisotropic_R']==0,counts
    else:assert counts['anisotropic_R']>0,counts
    result={'counts':dict(counts),'examples':sorted(examples),'verified':True}
    (folder/'measurement_audit.json').write_text(json.dumps(result,indent=2))
    print(key,variant,rep,dict(counts),flush=True)

if __name__=='__main__':
    import sys
    if len(sys.argv)==4:inspect(sys.argv[1],sys.argv[2],int(sys.argv[3]))
    else:
        for args in PLAN:inspect(*args)
