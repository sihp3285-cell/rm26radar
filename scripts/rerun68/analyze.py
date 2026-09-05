#!/usr/bin/env python3
"""Causal A/B replay evaluation and replacement of the three primary reports."""
import argparse,collections,csv,hashlib,html,importlib.util,json,pathlib,sys
import numpy as np
from align import load_db,GIDS,ROLES,SLOTS,BASE,ROOT
from rules import classify,simulate,intervals,duration,merge,intersect,unlocks

PROTOCOL_URL='https://bbs-web-static.robomaster.com/ef4f084944e34393aa70378a4a405c681774586313231/RoboMaster%202026%20%E6%9C%BA%E7%94%B2%E5%A4%A7%E5%B8%88%E9%AB%98%E6%A0%A1%E7%B3%BB%E5%88%97%E8%B5%9B%E9%80%9A%E4%BF%A1%E5%8D%8F%E8%AE%AE%20V1.3.0%EF%BC%8820260327%EF%BC%89.pdf'

def stamp_ns(msg):
    s=msg['header']['stamp'];return s['sec']*10**9+s['nanosec']

def read_packets(folder):
    meta=json.loads((folder/'source.json').read_text());origin=meta['epoch_ns'];offset=meta['offset']
    streams={s:[[] for _ in ROLES] for s in ('A','B')};prior=None;latencies=[];arrivals=[];sources=[]
    stale=0;guesses=0;legacy_omitted=0
    with open(folder/'events.jsonl') as f:
        for line in f:
            e=json.loads(line);m=e['message'];receive=e['receipt_ns'];now=(receive-origin)/1e9+offset
            if e['topic']=='/prior_predictions':
                prior=(receive,m);continue
            if e['topic']!='/radar_map':continue
            age=(receive-stamp_ns(m))/1e9
            if 0<=now<=420: latencies.append(age);arrivals.append(now);sources.append((stamp_ns(m)-origin)/1e9+offset)
            by_slot={}
            if prior and prior[0]<=receive and prior[1]['model_enabled']:
                # Only an already received prior, with a fresh original frame.
                prior_age=(receive-stamp_ns(prior[1]))/1e9
                if 0<=prior_age<=.5:
                    by_slot={p['slot_idx']:p for p in prior[1]['predictions'] if p['valid'] and p['team_id']==1}
                else:stale+=1
            for role,slot in enumerate(SLOTS):
                px,py=m['red_x'][slot],m['red_y'][slot]
                if px!=0 or py!=0:
                    p={'t':now,'wx':(px-194)/(388/15),'wz':(py-361)/(722/28),'source':'tracking','source_stamp_ns':stamp_ns(m),'receipt_ns':receive}
                    streams['A'][role].append(p);streams['B'][role].append(p.copy())
                elif role in by_slot:
                    pred=by_slot[role]
                    p={'t':now,'wx':pred['prior_world_x'],'wz':pred['prior_world_z'],'source':'guess','source_stamp_ns':stamp_ns(prior[1]),'receipt_ns':receive}
                    streams['B'][role].append(p);guesses+=int(0<=now<=420)
                    old_array=prior[1]['predictions']
                    if 0<=now<=420 and (role>=len(old_array) or old_array[role]['slot_idx']!=role):
                        legacy_omitted+=1
    if not arrivals:raise ValueError('no in-match map receipts')
    dt=np.diff(arrivals)
    stats={'map_receipts':len(arrivals),'map_hz':(len(arrivals)-1)/(arrivals[-1]-arrivals[0]),
           'first_receipt_s':arrivals[0],'last_receipt_s':arrivals[-1],
           'latency_median_ms':float(np.median(latencies)*1000),'latency_p95_ms':float(np.percentile(latencies,95)*1000),
           'map_gap_p95_ms':float(np.percentile(dt,95)*1000),'map_gap_max_ms':float(max(dt)*1000),
           'guess_packets':guesses,'stale_prior_frames_rejected':stale,
           'guess_packets_old_array_index_would_omit':legacy_omitted,
           'time_basis':'rosbag local reception; not referee-server reception',
           'causal_prior_max_age_s':.5,'db_hp_used_for_packet_selection':False}
    frames=np.genfromtxt(folder/'source_frames.csv',delimiter=',',names=True)
    stats['source_frames_published']=int(frames['published'].sum())
    stats['source_frames_dropped_late']=int((frames['published']==0).sum())
    stats['negative_output_latency_count']=int(np.sum(np.array(latencies)<0))
    assert stats['negative_output_latency_count']==0,stats
    assert arrivals[0]<2 and arrivals[-1]>418,'incomplete match coverage'
    return streams,stats

def alive_intervals(db):
    # State sampling is left-continuous at samples; edges hold the nearest sample.
    times=np.r_[0.,db[:,0],420.];hp=np.r_[db[0,3],db[:,3]]
    return merge([(a,b) for a,b,h in zip(times[:-1],times[1:],hp) if h>0])

def db_intervals(db):return merge([(t,min(t+1,420.)) for t,*_,v in db if v==1])

def rate_limited(streams,folder,hz):
    """Select the latest *whole fused frame* at a fixed transmission clock.

    A missing slot clears its old coordinate. This never holds the last valid
    per-robot point across missing frames. It is a declared hypothetical sender.
    """
    meta=json.loads((folder/'source.json').read_text());origin=meta['epoch_ns'];offset=meta['offset']
    frames=[]
    with open(folder/'events.jsonl') as f:
        for line in f:
            e=json.loads(line)
            if e['topic']=='/radar_map':
                frames.append(((e['receipt_ns']-origin)/1e9+offset,e['receipt_ns'],stamp_ns(e['message'])))
    lookups={n:[{p['receipt_ns']:p for p in ps} for ps in roles] for n,roles in streams.items()}
    out={n:[[] for _ in ROLES] for n in streams};i=-1
    for tick in np.arange(0,420,1/hz):
        while i+1<len(frames) and frames[i+1][0]<=tick+1e-9:i+=1
        if i<0:continue
        _,receipt,source=frames[i]
        if tick-((source-origin)/1e9+offset)>.5:continue
        for name,roles in lookups.items():
            for role,lookup in enumerate(roles):
                packet=lookup.get(receipt)
                if packet is not None and tick-((packet['source_stamp_ns']-origin)/1e9+offset)<=.5:
                    # Explicit test-sender encoding: nearest cm, uint16 range.
                    # Out-of-range values are rejected rather than wrapped.
                    cx=int(round((packet['wz']+14.)*100));cy=int(round((packet['wx']+7.5)*100))
                    if not (0<=cx<=65535 and 0<=cy<=65535) or cx==cy==0:continue
                    out[name][role].append(dict(packet,t=float(tick),wx=cy/100.-7.5,wz=cx/100.-14.,
                        protocol_x_cm=cx,protocol_y_cm=cy,send_time_ns=origin+round((tick-offset)*1e9)))
    return out

def evaluate(streams,db,shift=0.,extra_delay=0.):
    results={}
    for name,roles in streams.items():
        per=[];alliv=[];aliveiv=[]
        for role,(packets,actual) in enumerate(zip(roles,db)):
            rev=actual[1:,0][(actual[:-1,3]<=0)&(actual[1:,3]>0)].tolist()
            ps=[]
            for packet in packets:
                t=packet['t']+shift+extra_delay
                if not 0<=t<=420:continue
                wx=np.interp(t,actual[:,0],actual[:,1]);wz=np.interp(t,actual[:,0],actual[:,2])
                distance=float(np.hypot(packet['wx']-wx,packet['wz']-wz))
                p=dict(packet,t=t,state=classify(distance),distance_m=distance)
                ps.append(p)
            tl=simulate(ps,rev);iv=intervals(tl);iv20=intervals(tl,threshold=120)
            living=intersect(iv,alive_intervals(actual));counts=collections.Counter(p['state'] for p in ps)
            alliv.extend(iv);aliveiv.extend(living)
            per.append({'role':ROLES[role],'total_s':duration(iv),'t20_s':duration(iv20),
                        't15_s':duration(iv)-duration(iv20),'alive_s':duration(living),
                        'first100_s':next((r['t'] for r in tl if r['P']>=100),None),
                        'first120_s':next((r['t'] for r in tl if r['P']>=120),None),
                        'counts':dict(counts),'packets':len(ps),'guess_packets':sum(p['source']=='guess' for p in ps),
                        'interruptions':sum(r['state']=='interrupt' for r in tl),'revivals':rev,
                        'rms_m':float(np.sqrt(np.mean([p['distance_m']**2 for p in ps]))) if ps else None,
                        'timeline':tl})
        results[name]={'per_robot':per,'union_s':duration(alliv),'sum_s':sum(p['total_s'] for p in per),
                       'alive_union_s':duration(aliveiv),'alive_sum_s':sum(p['alive_s'] for p in per),
                       'conditional_unlock_s':unlocks(alliv),'alive_unlock_s':unlocks(aliveiv)}
    return results

def step_chart(per):
    colors=['#d1495b','#d88b00','#238a63','#347cbb','#7c59a5'];paths=[]
    for p,col in zip(per,colors):
        tl=p['timeline'];coords=[];lastp=0
        for row in tl+[{'t':420,'P':tl[-1]['P']}]:
            x=55+row['t']*800/420;y=220-row['P']*180/150
            if not coords:coords.append(f'M{x:.2f},{y:.2f}')
            else:coords.append(f'H{x:.2f}V{y:.2f}')
        paths.append(f'<path d="{" ".join(coords)}" stroke="{col}" fill="none" stroke-width="1.2"/>')
    axes=''.join(f'<line x1="55" y1="{220-v*180/150}" x2="855" y2="{220-v*180/150}" stroke="#ddd"/><text x="20" y="{224-v*180/150}">{v}</text>' for v in [0,50,100,120,150])
    axes+=''.join(f'<text x="{55+t*800/420}" y="245">{t}s</text>' for t in range(0,421,60))
    legend=''.join(f'<span style="color:{c};margin-right:20px">● {p["role"]}</span>' for p,c in zip(per,colors))
    return f'<div>{legend}</div><svg viewBox="0 0 900 260" role="img" aria-label="B方案各机器人被标记进度阶梯图">{axes}{"".join(paths)}</svg>'

def fmt(v):return '—' if v is None else f'{v:.1f}'

def comparison_bar_chart(metric,title,values,maximum,step,caption):
    """Fixed zero-based scales allow comparisons between all three matches."""
    parts=[f'<title id="{metric}-title">{title}：历史 DB、A 纯跟踪、B 跟踪＋因果猜点</title>']
    for tick in range(0,maximum+1,step):
        y=280-tick/maximum*224
        parts.append(f'<line x1="62" y1="{y:.2f}" x2="490" y2="{y:.2f}" stroke="#dce3e8"/><text x="52" y="{y+5:.2f}" text-anchor="end" fill="#637480">{tick}</text>')
    parts.append('<text x="62" y="25" fill="#637480">秒</text>')
    for x,(name,value),label,color in zip([140,280,420],values,['历史 DB','A 纯跟踪','B 跟踪＋猜点'],['#7f8c8d','#e67e22','#16a085']):
        value=float(value)
        height=value/maximum*224;y=280-height
        parts.append(f'<rect class="comparison-bar" data-metric="{metric}" data-series="{name}" data-value="{value!r}" x="{x-38}" y="{y:.4f}" width="76" height="{height:.4f}" rx="3" fill="{color}"/>')
        parts.append(f'<text x="{x}" y="{y-10:.2f}" text-anchor="middle" font-weight="600">{value:.1f}</text><text x="{x}" y="311" text-anchor="middle">{label}</text>')
    return f'<figure class="comparison-chart"><h3>{title}</h3><svg viewBox="0 0 520 334" role="img" aria-labelledby="{metric}-title" data-metric="{metric}" data-maximum="{maximum}">{"".join(parts)}</svg><figcaption class="muted">{caption}</figcaption></figure>'

def table(headers,rows):
    return '<div class="scroll"><table><thead><tr>'+''.join(f'<th>{html.escape(str(x))}</th>' for x in headers)+'</tr></thead><tbody>'+''.join('<tr>'+''.join(f'<td>{html.escape(str(x))}</td>' for x in row)+'</tr>' for row in rows)+'</tbody></table></div>'

def write_report(key,folder,stats,results,db,sensitivity):
    alignment=json.loads((BASE/'alignment.json').read_text())[key]
    dbiv=[iv for a in db for iv in db_intervals(a)];dbliving=[iv for a in db for iv in intersect(db_intervals(a),alive_intervals(a))]
    dbsum=sum(duration(db_intervals(a)) for a in db)
    dbalive=sum(duration(intersect(db_intervals(a),alive_intervals(a))) for a in db)
    overview=[['历史DB：是否易伤',fmt(duration(dbiv)),fmt(dbsum),fmt(duration(dbliving)),fmt(dbalive),'来源未细分']]
    for name,label in [('A','A 纯跟踪'),('B','B 跟踪＋因果猜点')]:
        r=results[name];overview.append([label,fmt(r['union_s']),fmt(r['sum_s']),fmt(r['alive_union_s']),fmt(r['alive_sum_s']),' / '.join(fmt(t)+'s' for t in r['conditional_unlock_s']) or '无'])
    perrows=[];countrows=[]
    for i,role in enumerate(ROLES):
        for name in ['A','B']:
            r=results[name]['per_robot'][i];c=r['counts'];n=r['packets']
            perrows.append([role,name,fmt(duration(db_intervals(db[i]))),fmt(r['t15_s']),fmt(r['t20_s']),fmt(r['total_s']),fmt(r['alive_s']),fmt(r['first100_s']),fmt(r['first120_s'])])
            countrows.append([role,name,n,c.get('accurate',0),c.get('semi',0),c.get('wrong',0),r['interruptions'],r['guess_packets'],f'{100*(c.get("accurate",0)+c.get("semi",0))/n:.1f}%' if n else '—'])
    sr=[[x['label'],fmt(x['A']['union_s']),fmt(x['B']['union_s']),fmt(x['A']['sum_s']),fmt(x['B']['sum_s'])] for x in sensitivity]
    split=json.loads((ROOT/'position_prior_toolkit/run_v1/02_game_split.json').read_text())['games']
    splitname=next(k for k,v in split.items() if GIDS[key] in v)
    delta=results['B']['union_s']-results['A']['union_s']
    summary={k:{kk:vv for kk,vv in v.items() if kk!='per_robot'} for k,v in results.items()}
    summary.update(key=key,gid=GIDS[key],stats=stats,alignment=alignment,model_split=splitname,
        sender={'hz':5,'protocol':'0x0305','protocol_version':'V1.3.0 (20260327)','source_url':PROTOCOL_URL,
                'coordinate_quantization':'nearest centimeter; unrepresentable uint16 rejected; zero-zero omitted',
                'extra_transport_delay_s':0.,'packet_loss':0.,'clock_phase_match_s':0.,'real_referee_transmission':False},
        DB={'union_s':duration(dbiv),'sum_s':dbsum,'alive_union_s':duration(dbliving),'alive_sum_s':dbalive},
        assumptions={'rf_suppression':'unknown, assumed absent','position_module_offline':'unknown, assumed absent',
                     'referee_transport':'unmeasured; 5Hz virtual send after local receipt; delay/loss scenarios included',
                     'death_progress':'interruptions continue; only x resets on revival',
                     'double':'conditional opportunity times, not actual triggered double vulnerability'},
        sensitivity=sensitivity)
    (folder/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
    for name,r in results.items():
        with open(folder/f'ledger_{name}.jsonl','w') as f:
            for p in r['per_robot']:
                for row in p['timeline']:f.write(json.dumps(dict(role=p['role'],**row),ensure_ascii=False)+'\n')
    def compact(p):return {k:v for k,v in p.items() if k!='timeline'}
    (folder/'per_robot.json').write_text(json.dumps({n:[compact(p) for p in r['per_robot']] for n,r in results.items()},ensure_ascii=False,indent=2))
    css='body{font:15px/1.7 system-ui,sans-serif;max-width:1180px;margin:28px auto;padding:0 20px;color:#213547;background:#f6f8fa}h1{font-size:26px}h2{font-size:20px;margin-top:30px}.box{padding:18px 22px;background:white;border:1px solid #dce3e8;border-radius:9px;margin:18px 0}.notice{border-left:5px solid #bd7816;background:#fff8e8}table{border-collapse:collapse;width:100%;white-space:nowrap;font-size:14px}th,td{padding:9px 12px;border:1px solid #dce3e8;text-align:right}th{background:#e9f0f4}td:first-child,th:first-child{text-align:left}.scroll{overflow:auto}svg{width:100%;height:auto}a{color:#126098}code{background:#edf1f5;padding:2px 5px}.muted{color:#637480}'
    css+='.comparison-charts{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:18px;margin:20px 0}.comparison-chart{margin:0;padding:18px;background:white;border:1px solid #dce3e8;border-radius:9px;min-width:0}.comparison-chart h3{margin:0 0 8px;font-size:18px}.comparison-chart svg{display:block;font-size:14px}.comparison-chart figcaption{font-size:13px}@media(max-width:780px){.comparison-charts{grid-template-columns:1fr}}'
    charts=comparison_bar_chart('union_s','易伤并集', [('DB',duration(dbiv))]+[(n,results[n]['union_s']) for n in ['A','B']],420,60,'任意一台机器人处于易伤的时间，同一时刻只计一次。三局统一纵轴：0–420秒。')
    charts+=comparison_bar_chart('sum_s','单机易伤合计', [('DB',dbsum)]+[(n,results[n]['sum_s']) for n in ['A','B']],1500,300,'五台机器人各自易伤时长相加，同时易伤分别计入。合计图统一纵轴：0–1500秒，每300秒一格。')
    doc=f'''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>68{key} 易伤重测报告 · 修正规则与时间口径</title><style>{css}</style></head><body>
<h1>第 68 场 · 第 {key[-1]} 局雷达易伤重测报告</h1>
<p class="muted">华中科技大学（红）vs 电子科技大学中山学院（蓝）｜评估目标：红方五台地面机器人｜GID {GIDS[key]}</p>
<div class="box notice"><b>本报告已用新录制数据替换旧结果。</b>主表、逐车统计和P时间线统一采用 <b>5Hz坐标提交</b>，符合0x0305协议频率上限；采用固定视频内容时间映射和比赛节奏重放，但不是裁判系统实测。核心检测、跟踪、定位和猜点代码及配置保持不变；Qt 关闭。射频压制、定位模块离线及实际裁判通信延迟未实测，因此不再宣称“实战超过历史 DB 某百分比”。</div>
<div class="box"><b>同一次录制：</b>B 相对 A 的标记阈值并集变化为 {delta:+.1f}s。并集接近饱和时，应同时看逐车时长和猜点错误。<br>
<b>数据时钟：</b>地图实测输出 {stats['map_hz']:.2f}Hz，但主评估按5Hz发送时钟选取当时已收到的最新融合整帧，计分时刻包括本地处理与等待发送的延迟。原帧到地图接收延迟中位 {stats['latency_median_ms']:.1f}ms / P95 {stats['latency_p95_ms']:.1f}ms；实际链路延迟未测，另列情景。<br>
<b>录制完整性：</b>比赛窗口地图消息 {stats['map_receipts']} 条；完整 bag 逐条直接读取并核对数量；源视频迟到丢帧 {stats['source_frames_dropped_late']} 帧。<br>
<b>时间映射：</b>比赛秒 = {alignment['slope']:.6f} × 文件内容秒 {alignment['offset']:+.3f}。映射在独立预录上拟合，正式录制前冻结；预录留出块 RMS {alignment['holdout_rms_m']:.3f}m。RMS 是空间残差，不等于时钟精度。</div>
<h2>1. 整体时长与条件机会</h2><p>“标记阈值时长”按模拟 P≥100 统计；DB 按原始“是否易伤”列统计，二者来源口径并非完全相同。存活时长另按 DB 血量统计，血量不参与在线选点。条件机会按阈值并集累计 60/120 秒，未模拟主动触发及持续 30 秒的翻倍效果，也不能确认死亡阶段官方是否采用相同累计口径。</p>
<div class="comparison-charts">{charts}</div>
{table(['方案','并集(s)','单机合计(s)','存活并集(s)','存活合计(s)','条件机会解锁时刻'],overview)}
<h2>2. 逐机器人易伤阈值时长</h2>{table(['机器人','方案','DB标记(s)','15%档(s)','20%档(s)','合计(s)','存活合计(s)','首次P≥100(s)','首次P≥120(s)'],perrows)}
<h2>3. 每次提交的判定与数据中断</h2><p>有效占比 =（准确＋半准确）/ 5Hz发送器实际提交的坐标数；中断单列。先按地图到达事件生成因果融合整帧，再由5Hz发送器读取最新整帧。猜点只用 top-1，源帧超过0.5s拒绝；无未来消息及 DB 死亡过滤。坐标按厘米四舍五入，不能用uint16表示的坐标拒发；两轴均为0视为未提交。</p><p><b>修正槽位查找：</b>预测数组不是固定槽位数组，本次按每条预测的 slot_idx 查找。原始地图节拍中有 {stats['guess_packets']} 次猜点候选，其中 {stats['guess_packets_old_array_index_would_omit']} 次会被旧“数组下标=槽位”查找方式漏掉；这不是下表5Hz提交数，B 的变化也不能仅归因于计时修复。</p>{table(['机器人','方案','提交数','准确','半准确','错误','中断判定','猜点提交','有效占比'],countrows)}
<h2>4. B 方案 P 时间线（事件阶梯，未做线性平滑）</h2><div class="box">{step_chart(results['B']['per_robot'])}</div>
<h2>5. 对时间轴、通信延迟及发送节拍的敏感性</h2><p>默认均以5Hz主情景为基础，不重新拟合轨迹或重跑模型。±2秒不是已证明的误差上界。网络丢包情景对同一整包中的所有机器人一致丢弃，随机种子固定为68；单次随机情景不是统计置信区间。4Hz为额外可行节拍；10Hz和未限频结果超过协议上限，仅作算法诊断，不应解释为比赛可实现值。</p>{table(['情景','A并集(s)','B并集(s)','A合计(s)','B合计(s)'],sr)}
<h2>6. 规则、数据边界与可复核材料</h2><div class="box"><ul>
<li>规则：V2.0.1 第117–118页；先更新 x 再更新 P；整数十分之一分计算；准确/半准确共享正向连续性；错误/中断共享负向连续性；每0.5秒未收数据扣分。复活仅重置x，不停止中断计时。</li>
<li><a href="{PROTOCOL_URL}">官方通信协议V1.3.0</a>第7页规定0x0305上限5Hz，第32–33页规定厘米整数坐标、两轴均为0视为未发送。第4页给出常规约130ms延迟/丢包率小于1%、较差环境约200ms/约3%的参考。本报告网络情景仅按这些量级做压力扫描，未认定为本机实测链路。</li>
<li>同刻事件约定：复活 → 中断 → 数据。0.5秒边界收包按中断先发生；应以裁判实测确认精确边界。</li>
<li>DB为每车1Hz、1–419秒；坐标线性插值，0–1秒及419–420秒保留最近端点；死亡状态使用已发生样本。急转/复活附近参考坐标有不确定性。</li>
<li>规则第120页干扰波压制及定位模块离线状态未记录，报告仅在无压制、定位模块在线的假设下解释。P阈值持续时间不能直接等同于实际伤害收益。</li>
<li>当前先验模型中本局划分为 <b>{splitname}</b>；训练集局不得用于独立泛化证明。未修改或重新训练该模型。</li>
<li>原始bag、逐帧来源、逐事件P/x账本及汇总保留于 <code>log/rerun68/{folder.name}</code>。<a href="../rerun68/{folder.name}/summary.json">汇总JSON</a> · <a href="../rerun68/{folder.name}/per_robot.json">逐车JSON</a> · <a href="../rerun68/{folder.name}/export_audit.json">消息数核验</a>。</li>
<li>生成器：<code>scripts/rerun68/analyze.py</code>；规则测试：<code>scripts/rerun68/test_rules.py</code>。旧报告备份保留于 <code>log/rerun68/previous_reports</code>。</li>
</ul></div></body></html>'''
    dest=ROOT/f'log/report_68{key}/report.html';dest.parent.mkdir(exist_ok=True)
    backup=BASE/'previous_reports'/f'report_68{key}.html';backup.parent.mkdir(exist_ok=True)
    if dest.exists() and not backup.exists():backup.write_bytes(dest.read_bytes())
    dest.write_text(doc)
    print(json.dumps(summary,ensure_ascii=False,indent=2))

def main(key,phase):
    folder=BASE/f'{phase}_{key}';raw_streams,stats=read_packets(folder);db=load_db(GIDS[key])
    streams=rate_limited(raw_streams,folder,5);results=evaluate(streams,db)
    sensitivity=[]
    for shift in [-2.,-1.,-.5,-.2,0.,.2,.5,1.,2.]:
        rs=results if shift==0 else evaluate(streams,db,shift=shift)
        sensitivity.append({'label':f'时间映射偏移 {shift:+.1f}s',**{n:{k:r[k] for k in ('union_s','sum_s')} for n,r in rs.items()}})
    for delay in [.1,.13,.2]:
        rs=evaluate(streams,db,extra_delay=delay)
        sensitivity.append({'label':f'额外通信延迟 {delay*1000:.0f}ms',**{n:{k:r[k] for k in ('union_s','sum_s')} for n,r in rs.items()}})
    for hz in [4,5,10]:
        rs=evaluate(rate_limited(raw_streams,folder,hz),db)
        sensitivity.append({'label':f'固定发送节拍 {hz}Hz'+('（超协议上限，仅诊断）' if hz>5 else ''),**{n:{k:r[k] for k in ('union_s','sum_s')} for n,r in rs.items()}})
    for delay,loss in [(.13,.01),(.2,.03)]:
        rng=np.random.default_rng(68);lost={i for i in range(2100) if rng.random()<loss}
        sampled={n:[[p for p in ps if round(p['t']*5) not in lost] for ps in roles] for n,roles in streams.items()}
        rs=evaluate(sampled,db,extra_delay=delay)
        sensitivity.append({'label':f'5Hz＋{delay*1000:.0f}ms＋{loss*100:.0f}%丢包（种子68）',**{n:{k:r[k] for k in ('union_s','sum_s')} for n,r in rs.items()}})
    rs=evaluate(raw_streams,db)
    sensitivity.append({'label':'未限频地图节拍（超协议上限，仅诊断）',**{n:{k:r[k] for k in ('union_s','sum_s')} for n,r in rs.items()}})
    write_report(key,folder,stats,results,db,sensitivity)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('key',choices=GIDS);p.add_argument('--phase',default='realtime');a=p.parse_args();main(a.key,a.phase)
