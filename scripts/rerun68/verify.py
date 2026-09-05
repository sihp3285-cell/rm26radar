#!/usr/bin/env python3
"""Completion audit against recorded files, not a substitute for a real run."""
import collections,csv,hashlib,json,pathlib
from html.parser import HTMLParser
from align import ROOT,BASE,GIDS,ROLES

def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()

class ReportStructure(HTMLParser):
    def __init__(self):
        super().__init__();self.tables=[];self.table=None;self.cells=0;self.links=[];self.svg_paths=0;self.headings=0
        self.bar_charts={};self.bars={}
    def handle_starttag(self,tag,attrs):
        attrs=dict(attrs)
        if tag=='svg' and 'data-metric' in attrs:
            metric=attrs['data-metric'];assert metric not in self.bar_charts
            self.bar_charts[metric]=float(attrs['data-maximum'])
        if tag=='rect' and attrs.get('class')=='comparison-bar':
            key=(attrs['data-metric'],attrs['data-series']);assert key not in self.bars
            value=float(attrs['data-value']);self.bars[key]=value
            height=float(attrs['height']);y=float(attrs['y'])
            assert abs(height-value/self.bar_charts[key[0]]*224)<1e-4
            assert abs(y+height-280)<1e-4 and height>=0 and y>=56
        if tag=='table':self.table=[]
        elif tag=='tr':self.cells=0
        elif tag in ('td','th'):self.cells+=1
        elif tag=='a':self.links.append(dict(attrs).get('href',''))
        elif tag=='path':
            self.svg_paths+=1
            data=dict(attrs).get('d','').lower();assert 'nan' not in data and 'inf' not in data
        elif tag=='h2':self.headings+=1
    def handle_endtag(self,tag):
        if tag=='tr' and self.table is not None:self.table.append(self.cells)
        elif tag=='table':self.tables.append(self.table);self.table=None

def union_s(intervals):
    # Independently integrate an interval endpoint sweep.
    events=collections.defaultdict(int)
    for a,b in intervals:
        if b>a:events[a]+=1;events[b]-=1
    active=0;previous=0.;total=0.
    for t,delta in sorted(events.items()):
        if active:total+=t-previous
        active+=delta;previous=t
    assert active==0
    return total

def verify(key):
    folder=BASE/f'realtime_{key}'
    run=json.loads((folder/'run.json').read_text());assert run['status']=='recorded',run
    audit=json.loads((folder/'export_audit.json').read_text());assert audit['verified_all_bag_messages']
    summary=json.loads((folder/'summary.json').read_text());assert summary['gid']==GIDS[key]
    assert summary['sender']['hz']==5 and summary['sender']['protocol']=='0x0305'
    expected_alignment=json.loads((BASE/'alignment.json').read_text())[key]
    assert run['mapping']==expected_alignment
    meta=json.loads((folder/'source.json').read_text())
    assert meta['speed']==1 and meta['stride']==1
    assert meta['slope']==expected_alignment['slope'] and meta['offset']==expected_alignment['offset']
    frames={}
    with open(folder/'source_frames.csv') as f:
        for row in csv.DictReader(f):
            if row['published']=='1':frames[int(row['stamp_ns'])]=row
    previous=0;count=collections.Counter();map_receipts=set()
    with open(folder/'events.jsonl') as f:
        for line in f:
            e=json.loads(line);count[e['topic']]+=1;assert e['receipt_ns']>=previous;previous=e['receipt_ns']
            s=e['message']['header']['stamp'];stamp=s['sec']*10**9+s['nanosec']
            assert stamp in frames,'message has no published source frame'
            assert stamp<=e['receipt_ns'],'negative local output latency'
            if e['topic']=='/radar_map':map_receipts.add(e['receipt_ns'])
    assert dict(count)==audit['exported_counts']
    assert audit['bag_counts']['/radar_map']==audit['bag_counts']['/world_targets']
    all_per=json.loads((folder/'per_robot.json').read_text())
    for name in ['A','B']:
        grouped=collections.defaultdict(list)
        with open(folder/f'ledger_{name}.jsonl') as f:
            for line in f:
                row=json.loads(line);grouped[row['role']].append(row)
                if row['state'] in ('accurate','semi','wrong'):
                    assert row['receipt_ns'] in map_receipts
                    assert row['source_stamp_ns']<=row['receipt_ns']
                    assert row['send_time_ns']+2>=row['receipt_ns']
                    assert abs(row['t']*5-round(row['t']*5))<1e-7
                    assert 0<=row['protocol_x_cm']<=65535 and 0<=row['protocol_y_cm']<=65535
                assert 0<=row['P']<=150 and 0<=row['t']<=420
        ivs=[];sums=0.
        for per in all_per[name]:
            tl=grouped[per['role']]
            assert tl[0]['t']==0 and tl[0]['P']==0
            assert all(b['t']>=a['t'] for a,b in zip(tl,tl[1:]))
            iv=[(a['t'],b['t']) for a,b in zip(tl,tl[1:]+[{'t':420.}]) if a['P']>=100]
            total=sum(b-a for a,b in iv)
            assert abs(total-per['total_s'])<1e-7
            assert abs(total-per['t15_s']-per['t20_s'])<1e-7
            counts=collections.Counter(r['state'] for r in tl if r['state'] in ('accurate','semi','wrong'))
            ts=[r['t'] for r in tl if r['state'] in ('accurate','semi','wrong')]
            assert all(b-a>=.2-1e-8 for a,b in zip(ts,ts[1:]))
            assert dict(counts)==per['counts']
            sums+=total;ivs.extend(iv)
        assert abs(sums-summary[name]['sum_s'])<1e-7
        assert abs(union_s(ivs)-summary[name]['union_s'])<1e-7
    report=ROOT/f'log/report_68{key}/report.html';body=report.read_text()
    assert '本报告已用新录制数据替换旧结果' in body
    assert str(GIDS[key]) in body and f'{summary["B"]["union_s"]:.1f}' in body
    structure=ReportStructure();structure.feed(body)
    assert [len(t) for t in structure.tables]==[4,11,11,19]
    assert all(len(set(t))==1 for t in structure.tables)
    assert structure.svg_paths==5 and structure.headings==6
    assert structure.bar_charts=={'union_s':420,'sum_s':1500}
    assert structure.bars=={(metric,name):summary[name][metric] for metric in ['union_s','sum_s'] for name in ['DB','A','B']}
    assert all((report.parent/link).is_file() for link in structure.links if not link.startswith('https://'))
    assert (BASE/'previous_reports'/f'report_68{key}.html').exists()
    return {'key':key,'raw_export_counts':dict(count),'source_provenance_checked':True,
            'ledger_totals_independently_checked':True,'report_sha256':digest(report),
            'html_tables_links_and_chart_structure_checked':True,
            'comparison_bars_match_summary_and_shared_scales_checked':True,
            'protocol_5hz_and_centimeter_coordinates_checked':True,
            'summary_sha256':digest(folder/'summary.json'),'A_union_s':summary['A']['union_s'],'B_union_s':summary['B']['union_s']}

if __name__=='__main__':
    before=json.loads((BASE/'core_before.json').read_text())
    changed=[p for p,h in before.items() if not (ROOT/p).is_file() or digest(ROOT/p)!=h]
    assert not changed,changed
    manifest=json.loads((BASE/'runtime_manifest.json').read_text())
    assert manifest['alignment_sha256']==digest(BASE/'alignment.json')
    assert all(digest(ROOT/p)==h for p,h in manifest['libraries'].items())
    result={'core_files_checked':len(before),'core_changes':changed,'runtime_binaries_unchanged':True,
            'frozen_alignment_unchanged':True,'games':[verify(key) for key in GIDS]}
    (BASE/'completion_audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print(json.dumps(result,ensure_ascii=False,indent=2))
