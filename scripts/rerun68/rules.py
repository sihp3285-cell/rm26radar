"""RMUC2026 V2.0.1 p117-118 event evaluator, separate from production code.

Scores use integer tenths. A receipt exactly at a timeout deadline is processed
after that timeout (explicit boundary convention). Revival never cancels a timer.
"""
import math

def classify(distance):
    if not math.isfinite(distance): raise ValueError('non-finite reference distance')
    return 'accurate' if distance < .8 else 'semi' if distance < 1.6 else 'wrong'

def simulate(packets, revives=(), end=420.):
    packets=sorted(packets,key=lambda p:p['t'])
    events=[(p['t'],2,'data',p) for p in packets if 0<=p['t']<=end]
    events += [(float(t),0,'revive',None) for t in revives if 0<=t<=end]
    # Only RECEIPTS restart the interruption clock. Generate across every revival.
    last=0.
    for p in packets + [{'t':end,'sentinel':True}]:
        t=min(end,p['t'])
        if t<0:continue
        k=1
        while last+.5*k <= t+1e-9:
            events.append((last+.5*k,1,'interrupt',None));k+=1
        last=t
        if p.get('sentinel'):break
    events.sort(key=lambda e:(e[0],e[1]))
    P=x=0;positive=None
    timeline=[{'t':0.,'P':0.,'x':0.,'state':'initial'}]
    for t,_,kind,p in events:
        if kind=='revive': x=0
        else:
            state=p['state'] if p else 'interrupt'
            new_positive=state in ('accurate','semi')
            delta={'accurate':10,'semi':5,'wrong':-8,'interrupt':-8}[state]
            x=x+delta if positive==new_positive else delta
            P=min(1500,max(0,P+x));positive=new_positive
        row={'t':float(t),'P':P/10.,'x':x/10.,'state':kind if kind!='data' else p['state']}
        if p: row.update({k:v for k,v in p.items() if k not in ('t','state')})
        timeline.append(row)
    return timeline

def intervals(timeline,end=420.,threshold=100.):
    return [(a['t'],b['t']) for a,b in zip(timeline,timeline[1:]+[{'t':end}])
            if a['P']>=threshold and b['t']>a['t']]

def merge(intervals):
    out=[]
    for a,b in sorted(intervals):
        if b<=a:continue
        if out and a<=out[-1][1]+1e-9:out[-1]=(out[-1][0],max(b,out[-1][1]))
        else:out.append((a,b))
    return out

def duration(intervals): return sum(b-a for a,b in merge(intervals))

def intersect(a,b):
    a,b=merge(a),merge(b);i=j=0;out=[]
    while i<len(a) and j<len(b):
        lo=max(a[i][0],b[j][0]);hi=min(a[i][1],b[j][1])
        if hi>lo:out.append((lo,hi))
        if a[i][1]<b[j][1]:i+=1
        else:j+=1
    return out

def unlocks(iv):
    targets=[60.,120.];out=[];elapsed=0.
    for a,b in merge(iv):
        while targets and elapsed+b-a>=targets[0]-1e-9:
            out.append(a+targets.pop(0)-elapsed)
        elapsed+=b-a
    return out
