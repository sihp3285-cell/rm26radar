#!/usr/bin/env python3
"""Check comparison artifacts against raw provenance and independent interval sums."""
import collections,csv,json,math
from html.parser import HTMLParser
import yaml
from comparison_analyze import GROUPS,folder_for
from comparison_build import CMP,sha
from comparison_run import PLAN
from run import ROOT,BASE
from verify import union_s

class Charts(HTMLParser):
    def __init__(self):
        super().__init__();self.bars=[];self.links=[];self.tables=[];self.current=None;self.cells=0;self.paths=0
    def handle_starttag(self,tag,attrs):
        a=dict(attrs)
        if tag=='rect' and 'data-value' in a:
            v,lo,hi=[float(a[x]) for x in ['data-value','data-min','data-max']]
            assert 0<=lo<=v<=hi
            maximum=420 if a['data-metric']=='union_s' else 1500
            assert hi<=maximum and abs(float(a['height'])-v*260/maximum)<1e-6
            assert abs(float(a['y'])+float(a['height'])-310)<1e-6
            self.bars.append(a)
        elif tag=='a':self.links.append(a['href'])
        elif tag=='path':
            self.paths+=1;assert 'nan' not in a['d'].lower() and 'inf' not in a['d'].lower()
        elif tag=='table':self.current=[]
        elif tag=='tr':self.cells=0
        elif tag in ['td','th']:self.cells+=1
    def handle_endtag(self,tag):
        if tag=='tr' and self.current is not None:self.current.append(self.cells)
        elif tag=='table':self.tables.append(self.current);self.current=None

def check_run(key,v,rep):
    folder=folder_for(key,v,rep);s=json.loads((folder/'comparison_summary.json').read_text())
    source=json.loads((folder/'source.json').read_text());run=json.loads((folder/'run.json').read_text())
    assert run['status']=='recorded' and source['speed']==source['stride']==1
    assert run['mapping']==json.loads((BASE/'alignment.json').read_text())[key]
    p=yaml.safe_load((folder/'pose_parameters.yaml').read_text())['/pose_node']['ros__parameters']
    prior=yaml.safe_load((folder/'prior_parameters.yaml').read_text())['/position_prior_node']['ros__parameters']
    assert p['projection_selector_enabled']==(v!='armor')
    assert (prior['navgrid_path']=='')==(v=='nav_off')
    assert prior['stay_anchor_probability_scale']==.8
    measurement=json.loads((folder/'measurement_audit.json').read_text());assert measurement['verified']
    published={int(r['stamp_ns']) for r in csv.DictReader(open(folder/'source_frames.csv')) if r['published']=='1'}
    receipts=set();counts=collections.Counter();prev=0
    with open(folder/'events.jsonl') as f:
        for line in f:
            e=json.loads(line);assert e['receipt_ns']>=prev;prev=e['receipt_ns'];counts[e['topic']]+=1
            stamp=e['message']['header']['stamp'];stamp=stamp['sec']*10**9+stamp['nanosec']
            assert stamp in published and stamp<=e['receipt_ns']
            if e['topic']=='/radar_map':receipts.add(e['receipt_ns'])
    assert dict(counts)==json.loads((folder/'export_audit.json').read_text())['exported_counts']
    per=json.loads((folder/'per_robot.json').read_text())
    for n in ['A','B']:
        grouped=collections.defaultdict(list)
        with open(folder/f'ledger_{n}.jsonl') as f:
            for line in f:
                r=json.loads(line);assert 0<=r['P']<=150 and 0<=r['t']<=420
                grouped[r['role']].append(r)
                if r['state'] in ['accurate','semi','wrong']:
                    assert r['receipt_ns'] in receipts
                    assert r['source_stamp_ns']<=r['receipt_ns']<=r['send_time_ns']+2
                    assert abs(r['t']*5-round(r['t']*5))<1e-7
                    assert 0<=r['protocol_x_cm']<=65535 and 0<=r['protocol_y_cm']<=65535
        alliv=[];total=0
        for p in per[n]:
            tl=grouped[p['role']];assert tl[0]['P']==0
            assert all(b['t']>=a['t'] for a,b in zip(tl,tl[1:]))
            iv=[(a['t'],b['t']) for a,b in zip(tl,tl[1:]+[{'t':420}]) if a['P']>=100]
            val=sum(b-a for a,b in iv);assert abs(val-p['total_s'])<1e-6
            assert abs(val-p['t15_s']-p['t20_s'])<1e-6
            alliv.extend(iv);total+=val
            c=collections.Counter(r['state'] for r in tl if r['state'] in ['accurate','semi','wrong'])
            assert dict(c)==p['counts']
        assert abs(total-s[n]['sum_s'])<1e-6 and abs(union_s(alliv)-s[n]['union_s'])<1e-6
    return {'folder':folder.name,'provenance_and_ledger_checked':True,'actual_parameters_and_R_checked':True,
            'summary_sha256':sha(folder/'comparison_summary.json'),'measurement_counts':measurement['counts']}

def main(ready_only=False):
    before=json.loads((CMP/'production_before.json').read_text())
    changed=[p for p,h in before.items() if not (ROOT/p).is_file() or sha(ROOT/p)!=h];assert not changed,changed
    old=json.loads((BASE/'core_before.json').read_text())
    assert not [p for p,h in old.items() if sha(ROOT/p)!=h]
    manifest=json.loads((CMP/'manifest.json').read_text())
    # The user subsequently requested sum-chart axis changes in all six reports.
    # Compare the original bytes outside that figure, rather than relaxing data checks.
    import re
    primary_unchanged=True
    for p,h in json.loads((CMP/'primary_reports_before.json').read_text()).items():
        if sha(ROOT/p)==h:continue
        primary_unchanged=False
        original=BASE/'chart_axis_20260905'/'before'/p
        assert sha(original)==h
        def without_sum(doc):
            return re.sub(r'<figure class="comparison-chart"><h3>单机易伤合计</h3>.*?</figure>', '', doc, flags=re.S)
        assert without_sum(original.read_text())==without_sum((ROOT/p).read_text())
        def values(doc):
            return re.findall(r'data-metric="sum_s" data-series="([^"]+)" data-value="([^"]+)"',doc)
        assert values(original.read_text())==values((ROOT/p).read_text())
    assert sha(BASE/'alignment.json')==manifest['alignment_sha256']
    assert all(sha(CMP/p)==h for p,h in manifest['artifacts'].items())
    ready=[args for args in PLAN if not ready_only or (folder_for(*args)/'measurement_audit.json').exists()]
    groups=[g for g in GROUPS if all((g[1],v,r) in ready for v in [g[2],'baseline'] for r in [1,2])]
    audit={'status':'complete' if len(ready)==len(PLAN) else 'partial','production_files_checked':len(before),'production_changes':changed,'frozen_alignment_unchanged':True,'primary_reports_unchanged':primary_unchanged,'primary_report_data_unchanged':True,
           'runs':[check_run(*args) for args in ready],'reports':[],
           'browser_visual_check':'not performed; local file navigation previously blocked; HTML/SVG data and structure checked'}
    excluded=CMP/'excluded_runs.json'
    audit['excluded_runs']=json.loads(excluded.read_text()) if excluded.exists() else []
    # Reserve the linked audit path before resolving local report links.
    dest=CMP/'comparison_audit.json'
    if not dest.exists():dest.write_text('{}')
    for name,key,control,title in groups:
        path=ROOT/'log'/name/'report.html';parsed=Charts();parsed.feed(path.read_text())
        assert len(parsed.bars)==10 and parsed.paths==20
        assert [len(t) for t in parsed.tables]==[9,3,6,9,5,12]
        assert all(len(set(t))==1 for t in parsed.tables)
        assert all((path.parent/link).is_file() for link in parsed.links if not link.startswith('https://'))
        expected=[]
        for m in ['union_s','sum_s']:
            sample=json.loads((folder_for(key,'baseline',1)/'comparison_summary.json').read_text())
            expected.append([sample['DB'][m]])
            for v in [control,'baseline']:
                for n in ['A','B']:
                    expected.append([json.loads((folder_for(key,v,r)/'comparison_summary.json').read_text())[n][m] for r in [1,2]])
        for b,values in zip(parsed.bars,expected):
            assert abs(float(b['data-value'])-sum(values)/len(values))<1e-6
            assert float(b['data-min'])==min(values) and float(b['data-max'])==max(values)
        audit['reports'].append({'path':str(path.relative_to(ROOT)),'sha256':sha(path),'bars_match_raw_summaries':True,'tables_links_timelines_checked':True})
    dest.write_text(json.dumps(audit,ensure_ascii=False,indent=2));print(json.dumps(audit,ensure_ascii=False,indent=2))

if __name__=='__main__':
    import sys
    main('--ready' in sys.argv)
