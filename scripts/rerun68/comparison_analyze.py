#!/usr/bin/env python3
"""Independent per-run evaluation and order-reversed comparison reports."""
import collections,html,json,math,pathlib,statistics,sys
from analyze import read_packets,rate_limited,evaluate,load_db,GIDS,ROLES,db_intervals,alive_intervals,intersect,duration,step_chart,table,PROTOCOL_URL
from comparison_build import CMP,sha
from comparison_run import PLAN
from run import ROOT,BASE

LABELS={'baseline':'生产配置','armor':'armor优先','fixed':'固定σ=1m','nav_off':'NavGrid＋盲区关'}
GROUPS=[('report_68g1_armor_dyn','g1','armor','动态选点 vs armor优先'),
        ('report_68g3_cov_cmp','g3','fixed','射线协方差 vs 固定协方差'),
        ('report_68g3_navgrid_cmp','g3','nav_off','NavGrid＋盲区先验全开 vs 全关')]

def folder_for(key,variant,rep):return BASE/f'cmp05_{variant}_r{rep}_{key}'
def aggregate(per):
    counts=collections.Counter()
    for p in per:counts.update(p['counts'])
    n=sum(counts.values())
    return {'counts':dict(counts),'submitted':n,'valid_percent':100*(counts['accurate']+counts['semi'])/n if n else 0,
            'guess_packets':sum(p['guess_packets'] for p in per),
            'interruptions':sum(p['interruptions'] for p in per)}

def analyze_run(key,variant,rep):
    folder=folder_for(key,variant,rep)
    run=json.loads((folder/'run.json').read_text());assert run['status']=='recorded'
    manifest=json.loads((CMP/'manifest.json').read_text())
    assert run['overrides']==manifest['variants'][variant]
    assert run['mapping']==json.loads((BASE/'alignment.json').read_text())[key]
    source=json.loads((folder/'source.json').read_text());assert source['speed']==1 and source['stride']==1
    export=json.loads((folder/'export_audit.json').read_text());assert export['verified_all_bag_messages']
    assert export['bag_counts']['/world_targets']==export['bag_counts']['/radar_map']
    raw,stats=read_packets(folder);streams=rate_limited(raw,folder,5);db=load_db(GIDS[key]);result=evaluate(streams,db)
    flags=collections.Counter()
    with open(folder/'events.jsonl') as f:
        for line in f:
            e=json.loads(line)
            if e['topic']=='/prior_predictions':
                for p in e['message']['predictions']:
                    flags['predictions']+=1
                    flags['mesh_used']+=int(p['mesh_used'])
                    flags['blind_zone_biased']+=int(p['blind_zone_biased'])
                    flags['stay_anchor_positive']+=int(p['stay_anchor_probability_mass']>0)
    assert flags['predictions']>0 and flags['stay_anchor_positive']>0
    if variant=='nav_off':assert flags['mesh_used']==flags['blind_zone_biased']==0
    else:assert flags['mesh_used']>0 and flags['blind_zone_biased']>0
    def compact(r):return {k:v for k,v in r.items() if k!='per_robot'}
    summary={'key':key,'variant':variant,'repeat':rep,'gid':GIDS[key],'stats':stats,'flags':dict(flags),
             'sender_hz':5,'run_folder':folder.name,'A':compact(result['A']),'B':compact(result['B']),
             'classification':{n:aggregate(r['per_robot']) for n,r in result.items()},
             'DB':{'union_s':duration([iv for a in db for iv in db_intervals(a)]),'sum_s':sum(duration(db_intervals(a)) for a in db)}}
    scenarios=[]
    for shift in [-2.,-1.,-.5,-.2,0.,.2,.5,1.,2.]:
        rs=result if shift==0 else evaluate(streams,db,shift=shift)
        scenarios.append({'label':f'时间偏移 {shift:+.1f}s','shift_s':shift,**{n:compact(r) for n,r in rs.items()}})
    import numpy as np
    for delay,loss in [(.13,.01),(.2,.03)]:
        rng=np.random.default_rng(68);lost={i for i in range(2100) if rng.random()<loss}
        sampled={n:[[p for p in ps if round(p['t']*5) not in lost] for ps in roles] for n,roles in streams.items()}
        rs=evaluate(sampled,db,extra_delay=delay)
        scenarios.append({'label':f'{delay*1000:.0f}ms＋{loss*100:.0f}%丢包',**{n:compact(r) for n,r in rs.items()}})
    summary['sensitivity']=scenarios
    for name,r in result.items():
        with open(folder/f'ledger_{name}.jsonl','w') as f:
            for p in r['per_robot']:
                for row in p['timeline']:f.write(json.dumps(dict(role=p['role'],**row),ensure_ascii=False)+'\n')
    per={n:[{k:v for k,v in p.items() if k!='timeline'} for p in r['per_robot']] for n,r in result.items()}
    (folder/'per_robot.json').write_text(json.dumps(per,ensure_ascii=False,indent=2))
    (folder/'comparison_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
    (folder/'timeline_B.svg.html').write_text(step_chart(result['B']['per_robot']))
    print(key,variant,rep,'A',summary['A']['sum_s'],'B',summary['B']['sum_s'],flush=True)
    return summary

def mean(values):return statistics.mean(values)
def fmt(x):return f'{x:.1f}'
def bar_chart(metric,title,series,maximum):
    # Each bar is a two-run mean; whiskers are observed min/max, NOT confidence intervals.
    parts=[f'<title>{title}，柱为两次均值，黑线为最小至最大值</title>']
    scale=260/maximum
    for value in [maximum*i/5 for i in range(6)]:
        y=310-value*scale
        parts.append(f'<line x1="64" x2="862" y1="{y}" y2="{y}" stroke="#dce3e8"/><text x="54" y="{y+5}" text-anchor="end">{value:.0f}</text>')
    for i,(label,values,color) in enumerate(series):
        value=mean(values);x=148+i*155;y=310-value*scale;lo=min(values);hi=max(values)
        parts.append(f'<rect data-metric="{metric}" data-label="{html.escape(label)}" data-value="{value}" data-min="{lo}" data-max="{hi}" x="{x-39}" y="{y}" width="78" height="{value*scale}" fill="{color}"/>')
        parts.append(f'<line x1="{x}" x2="{x}" y1="{310-hi*scale}" y2="{310-lo*scale}" stroke="#1d2936" stroke-width="2"/>')
        for v in [lo,hi]:parts.append(f'<line x1="{x-10}" x2="{x+10}" y1="{310-v*scale}" y2="{310-v*scale}" stroke="#1d2936" stroke-width="2"/>')
        parts.append(f'<text x="{x}" y="{310-hi*scale-12}" text-anchor="middle" font-weight="600">{value:.1f}</text><text x="{x}" y="340" text-anchor="middle">{html.escape(label)}</text>')
    return f'<div class="card"><h3>{title}</h3><svg viewBox="0 0 900 365" role="img" aria-label="{title}">{"".join(parts)}</svg></div>'

def write_report(name,key,control,title,data):
    rows={v:[data[(key,v,r)] for r in [1,2]] for v in [control,'baseline']}
    base=rows['baseline'];ctrl=rows[control];db=base[0]['DB']
    display={'baseline':{'armor':'动态选点','fixed':'射线R','nav_off':'全开'}[control],control:LABELS[control]}
    charts=[]
    for metric,label,maximum in [('union_s','易伤阈值并集（秒）',420),('sum_s','单机易伤阈值合计（秒）',1500)]:
        series=[('历史DB',[db[metric]],'#8a949e')]
        for v,colors in [(control,['#e9ab68','#d77823']),('baseline',['#65bba8','#16816d'])]:
            for n,color in zip(['A','B'],colors):series.append((n+' '+display[v],[r[n][metric] for r in rows[v]],color))
        charts.append(bar_chart(metric,label,series,maximum))
    overview=[];runtime=[];classification=[];perrows=[];deltas=[]
    for v in [control,'baseline']:
        for r in rows[v]:
            tag=display[v]+f' · 第{r["repeat"]}次'
            for n in ['A','B']:
                s=r[n];c=r['classification'][n]
                overview.append([tag,n,*[fmt(s[m]) for m in ['union_s','sum_s','alive_union_s','alive_sum_s']], ' / '.join(fmt(t) for t in s['conditional_unlock_s'])])
                classification.append([tag,n,*[c['counts'].get(k,0) for k in ['accurate','semi','wrong']],fmt(c['valid_percent'])+'%',c['guess_packets'],c['interruptions']])
            s=r['stats'];f=r['flags']
            runtime.append([tag,fmt(s['map_hz']),fmt(s['latency_median_ms']),fmt(s['latency_p95_ms']),s['source_frames_dropped_late'],f['mesh_used'],f['blind_zone_biased'],f['stay_anchor_positive']])
    for rep,(a,b) in enumerate(zip(base,ctrl),1):
        deltas.append([rep,*[fmt(a[n][m]-b[n][m]) for n in ['A','B'] for m in ['union_s','sum_s']],fmt((a['B']['sum_s']-a['A']['sum_s'])-(b['B']['sum_s']-b['A']['sum_s']))])
    for i,role in enumerate(ROLES):
        row=[role];role_means={}
        for v in [control,'baseline']:
            for n in ['A','B']:
                vals=[json.loads((BASE/r['run_folder']/'per_robot.json').read_text())[n][i]['total_s'] for r in rows[v]]
                row.append(f'{mean(vals):.1f} [{min(vals):.1f}, {max(vals):.1f}]')
                role_means[(v,n)]=mean(vals)
        row.append(f'{role_means[("baseline","B")]-role_means[(control,"B")]:+.1f}')
        perrows.append(row)
    sens=[]
    for j,scenario in enumerate(base[0]['sensitivity']):
        vals={v:{n:{m:mean([r['sensitivity'][j][n][m] for r in rows[v]]) for m in ['union_s','sum_s']} for n in ['A','B']} for v in rows}
        sens.append([scenario['label'],fmt(vals[control]['B']['union_s']),fmt(vals['baseline']['B']['union_s']),fmt(vals[control]['B']['sum_s']),fmt(vals['baseline']['B']['sum_s']),fmt(vals['baseline']['B']['sum_s']-vals[control]['B']['sum_s'])])
    delta=[a['B']['sum_s']-b['B']['sum_s'] for a,b in zip(base,ctrl)]
    trend='两次均为正向' if min(delta)>0 else '两次均为负向' if max(delta)<0 else '两次方向不一致或有零差值'
    sensitivity_deltas=[float(row[-1]) for row in sens[:9]]
    conclusion=f'生产功能相对对照的 B 单机合计差值：第1次 {delta[0]:+.1f}s，第2次 {delta[1]:+.1f}s，均值 {mean(delta):+.1f}s；{trend}。共同时间偏移扫描中的均值差范围为 {min(sensitivity_deltas):+.1f}～{max(sensitivity_deltas):+.1f}s。每配置仅两次、仅本局录像，不能据此声称统计显著或跨比赛泛化。'
    if min(sensitivity_deltas)<0<max(sensitivity_deltas):
        conclusion+=' 时间偏移会使收益方向反转，本次不能确认对齐误差下的稳定正收益。'
    elif min(delta)>0 and min(sensitivity_deltas)>0:
        conclusion+=' 所测重复和偏移情景均支持本局B合计正向，但不代表A、准确率或各兵种同时改善。'
    method={'armor':'对照使用现有 projection_selector_enabled=false：armor优先，armor缺失或不健康时仍允许回退car。因此本次是安全回退策略对照，不是严格禁止任何car位置的实验。其余配置与生产组一致。',
            'fixed':'对照在独立目录编译固定R版本，正常五射线投影的协方差为diag(1,1)m²，关闭该分支的地形方差膨胀；无效射线回退保持原样。R同时参与投影选择、跟踪及下游猜点，本实验测量整个R策略的总效果，不能只归因于Kalman。补丁按历史fixed实现的含义重建，不修改生产源码或安装库。',
            'nav_off':'对照仅在独立参数文件中将navgrid_path设为空，现有代码因此不加载NavGrid和盲区先验，通用候选使用欧氏距离。stay_anchor_probability_scale仍为0.8。只比较全开/全关，未测两种单开组合，不能分离两个功能的独立贡献。A链路无逻辑变化，其差异反映重放及负载波动；B−A差值仅作为辅助指标，不能完全消除波动。'}[control]
    failed_note='<p>一次fixed第1轮尝试因检查工具读取正在写入的bag导致SQLite锁冲突而中断，已保留、排除并按原配置重录；未计入两次有效样本，也不作为算法失败。之后仅在录制结束后检查bag。<a href="../rerun68/comparison_20260905/excluded_runs.json">排除记录</a>。</p>' if control=='fixed' else ''
    links=''.join(f'<li>{html.escape(display[v])} 第{r["repeat"]}次：<a href="../rerun68/{r["run_folder"]}/comparison_summary.json">汇总</a> · <a href="../rerun68/{r["run_folder"]}/per_robot.json">逐车</a> · <a href="../rerun68/{r["run_folder"]}/export_audit.json">bag核验</a> · <code>{r["run_folder"]}</code></li>' for v in [control,'baseline'] for r in rows[v])
    timelines=''.join(f'<details><summary>B {html.escape(display[v])} 第{r["repeat"]}次 P时间线</summary>{(BASE/r["run_folder"] / "timeline_B.svg.html").read_text()}</details>' for v in [control,'baseline'] for r in rows[v])
    css='body{font:15px/1.75 system-ui,sans-serif;color:#213547;background:#f5f7fa;max-width:1180px;margin:28px auto;padding:0 20px}h1{font-size:27px}h2{font-size:21px;margin-top:32px}h3{font-size:18px}.card,details{background:white;border:1px solid #dce3e8;border-radius:9px;padding:18px;margin:18px 0}.notice{border-left:5px solid #bd7816;background:#fff8e8}.scroll{overflow:auto}table{border-collapse:collapse;width:100%;white-space:nowrap;font-size:14px}th,td{padding:9px 12px;border:1px solid #dce3e8;text-align:right}th{background:#e9f0f4}td:first-child,th:first-child{text-align:left}svg{width:100%;height:auto;font-size:14px}a{color:#126098}code{overflow-wrap:anywhere}.muted{color:#637480}summary{cursor:pointer}'
    doc=f'''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>68{key} {title} · 2026-09-05重测</title><style>{css}</style></head><body>
<h1>68{key}：{title}</h1><p class="muted">2026-09-05重测｜同一录像，固定时间映射，顺序反转的两轮实验｜红方五台地面机器人</p>
<div class="card notice"><b>已替换历史口径数据。</b>本页所有实验值来自本轮完整重录，5Hz厘米坐标提交、因果猜点、逐事件P计算。假设定位在线、无干扰波压制；没有裁判链路实测。死亡后的实际猜点不按DB血量删除，存活覆盖另列。历史DB“是否易伤”是不同来源基线，不能以二者差值证明实战提升。</div>
<div class="card"><b>本局观察：</b>{conclusion}</div>
<h2>1. 对照定义与统计口径</h2><p>{method}</p>{failed_note}<p>A=纯跟踪；B=跟踪缺失时补已到达、原帧年龄≤0.5秒的top-1猜点。各组采用同一冻结映射，计入本地处理与发送等待，主情景额外网络延迟/丢包为零。g1/g3均属当前模型validation，但仍不是新比赛样本。生产配置保持不变，Qt及debug image关闭。</p>
<h2>2. 并集和合计柱状图</h2><p>柱为两次运行均值，黑线为两次的最小/最大值，不是置信区间。DB为单条历史基线。纵轴从零起，并集统一0–420秒、合计统一0–1500秒，每300秒一格。并集同刻只计一次，合计逐车相加。</p>{''.join(charts)}
{table(['配置/重复','方案','并集(s)','合计(s)','存活并集(s)','存活合计(s)','条件机会时刻(s)'],overview)}
<h2>3. 每轮差值：生产功能 − 对照</h2>{table(['轮次','A并集Δ(s)','A合计Δ(s)','B并集Δ(s)','B合计Δ(s)','(B−A)合计Δ(s)'],deltas)}<p>正值表示该指标增加。动态选点、测量R本身会改变A，因此A差异不能一概当作噪声扣除。B并集接近420秒时，应重点看单机合计、逐车结果和错误比例。</p>
<h2>4. 逐机器人阈值时长</h2><p>均值 [最小值, 最大值]，单位秒；末列为生产功能相对对照的B均值差，用于发现总体收益背后的兵种取舍。</p>{table(['机器人','A '+display[control],'B '+display[control],'A '+display['baseline'],'B '+display['baseline'],'B均值Δ(s)'],perrows)}
<h2>5. 提交判定与运行波动</h2>{table(['配置/重复','方案','准确','半准确','错误','有效占比','猜点提交','中断判定'],classification)}<p>有效占比=(准确＋半准确)/实际提交数；不含中断。下面三个功能计数来自原始预测消息，覆盖完整录制窗口，不能等同5Hz提交数。NavGrid关闭组应无mesh/盲区标志，但仍有驻留锚点。</p>{table(['配置/重复','地图Hz','延迟中位(ms)','延迟P95(ms)','源迟到丢帧','mesh标志','盲区标志','驻留锚点'],runtime)}
<h2>6. 共同时间偏移与网络情景</h2><p>同一个情景同时应用于两组，不为每组重新拟合时间。±2秒不是已证明的误差上界。丢包以整包为单位、固定种子68，网络情景不是实测或置信区间。</p>{table(['情景','对照B并集','生产B并集','对照B合计','生产B合计','B合计Δ'],sens)}
<h2>7. P时间线与复核材料</h2>{timelines}<ul>{links}</ul><p>全部原始bag、源帧记录、ledger_A/B.jsonl保存在各运行目录。<a href="../rerun68/comparison_20260905/manifest.json">对照构建与配置清单</a> · <a href="../rerun68/comparison_20260905/fixed_source.patch">固定R最小补丁</a> · <a href="../rerun68/comparison_20260905/comparison_audit.json">完整核验</a>。</p>
<p class="muted">规则V2.0.1第117–118页：先x后P、中断0.5秒、复活保留P而x清零。精确同刻边界采用复活→中断→数据约定。DB为1Hz，位置插值及战亡/复活边界有误差；定位离线特殊判定与干扰压制未还原；双倍仅列条件机会，未模拟实际主动触发。存活阈值覆盖也不等于伤害收益，尚有无敌和其他增益。<a href="{PROTOCOL_URL}">通信协议</a>。生成器：scripts/rerun68/comparison_analyze.py。</p></body></html>'''
    dest=ROOT/'log'/name/'report.html'
    backup=CMP/'previous_reports'/name/'report.html';backup.parent.mkdir(parents=True,exist_ok=True)
    if not backup.exists():backup.write_bytes(dest.read_bytes())
    dest.write_text(doc)
    return {'report':str(dest.relative_to(ROOT)),'sha256':sha(dest),'B_sum_deltas_s':delta,'B_sum_mean_delta_s':mean(delta),'time_shift_mean_delta_range_s':[min(sensitivity_deltas),max(sensitivity_deltas)]}

def main(ready_only=False):
    data={}
    for key,variant,rep in PLAN:
        folder=folder_for(key,variant,rep);cached=folder/'comparison_summary.json'
        if ready_only and not (folder/'measurement_audit.json').exists():continue
        data[(key,variant,rep)]=json.loads(cached.read_text()) if cached.exists() else analyze_run(key,variant,rep)
    reports=[write_report(*g,data) for g in GROUPS if all((g[1],v,r) in data for v in [g[2],'baseline'] for r in [1,2])]
    (CMP/'results.json').write_text(json.dumps(reports,ensure_ascii=False,indent=2))
    print(json.dumps(reports,ensure_ascii=False,indent=2))

if __name__=='__main__':
    if len(sys.argv)==4:analyze_run(sys.argv[1],sys.argv[2],int(sys.argv[3]))
    else:main('--ready' in sys.argv)
