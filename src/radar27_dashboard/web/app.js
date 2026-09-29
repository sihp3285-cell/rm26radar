'use strict';
const $ = id => document.getElementById(id);
const colors = {measurement:'#5c718d',tracking:'#5c718d',prior:'#b68b39',fusion:'#5c718d'};
const teamColors = {1:'#f05260',2:'#4296ff',0:'#a9b8ca'};
const roles = {0:['?','车辆'],1:['?','装甲板'],2:['1','英雄'],3:['2','工程'],4:['3','步兵'],5:['4','步兵'],6:['S','哨兵'],7:['O','前哨站']};
const metricNames = {camera_fps:'采集帧率',fps:'检测帧率',dropped_fps:'丢帧速率',dropped_count:'累计丢帧',car_ms:'车辆检测 / ms',armor_ms:'装甲检测 / ms',cls_ms:'兵种分类 / ms',total_ms:'推理总耗时 / ms',end_to_end_ms:'端到端耗时 / ms'};
const moduleNames = {Detection:'检测',Localization:'定位',Tracking:'跟踪',Prior:'猜点',Fusion:'融合',Decision:'决策',Camera:'输入',LiDAR:'激光雷达',Radio:'无线电',Referee:'裁判'};
const statusNames = {LIVE:'活跃',STALE:'已过期',WAITING:'等待数据',DISABLED:'已禁用','NOT IMPLEMENTED':'未实现'};
const trackingNames = {ACTIVE:'跟踪中',PREDICTED:'预测中',LOST:'已丢失',DEAD:'已死亡',INVALID:'无效'};
const sourceNames = {TRACKED:'真实跟踪',PREDICTED:'预测',PRIOR:'位置猜点',INVALID:'无有效位置'};
const rejectionNames = {0:'通过门控',1:'尚未到预测窗口',2:'超出预测窗口',3:'无可靠观测',4:'不支持的兵种',5:'超出场地',6:'无猜点分布',7:'候选不可达',8:'置信度不足',9:'模型未启用'};
const layerNames = {measurement:'测量',tracking:'跟踪',prior:'猜点',fusion:'融合',last:'最后测量',anchor:'观测锚点',candidate:'候选'};
const metrics = Object.keys(metricNames);
const enabled = {measurement:true,tracking:true,prior:true,fusion:true};
let state = null, selected = null, flipped = false, streams = [], rows = [], linked = {}, lastRadar = 0;
let mapSignature = '', targetPage = 0, candidatePage = 0, targetPageSize = 8, rowHeight = 0;
let labels = [];
const esc = value => String(value ?? '—').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const fmt = (v,n=2) => typeof v==='number' && Number.isFinite(v) ? v.toFixed(n) : '—';
const yes = v => v == null ? '—' : v ? '是' : '否';
const valid = p => Array.isArray(p) && p.length===2 && p.every(Number.isFinite);
const pos = p => valid(p) ? `(${fmt(p[0])}, ${fmt(p[1])})` : '—';
const code = t => `${({1:'R',2:'B'})[t.team_id] || 'U'}${roles[t.class_id]?.[0] || '?'}`;
const role = t => roles[t.class_id]?.[1] || '未知兵种';
const robot = t => `${code(t)} ${role(t)}`;
const teamClass = t => ({1:'team-red',2:'team-blue'})[t.team_id] || 'team-unknown';
const relation = t => ![1,2].includes(t.team_id) ? '阵营未知' : !state?.match?.own_team ? '敌我未定' : t.team_id===state.match.own_team ? '己方' : '敌方';
const dl = pairs => pairs.map(([cn,en,v])=>`<dt title="${esc(cn+' / '+en)}">${esc(cn)} <small>${esc(en)}</small></dt><dd title="${esc(v)}">${esc(v)}</dd>`).join('');
$('metrics').innerHTML = metrics.map(k=>`<div class="metric"><div class="metric-label">${metricNames[k]}<small>${k}</small></div><div class="metric-value" id="metric-${k}">—</div></div>`).join('');

function performance(data) {
  const s=data.performance;
  $('performance-status').textContent=`${statusNames[s.status]} ${s.status} · ${fmt(s.age_s,1)} s`;
  metrics.forEach(k=>$('metric-'+k).textContent=s.status==='LIVE'?fmt(s.data?.[k],k==='dropped_count'?0:1):'—');
}
function active(name) {
  const s=state.sources[name],t=state.sources.tracking;
  if (!s || s.status!=='LIVE') return [];
  if (['prior','measurement'].includes(name) && t.calibration_version!==null && s.calibration_version!==t.calibration_version) return [];
  // Fused has no calibration epoch; only overlay an exactly matching live frame.
  if (name==='fusion' && (t.status!=='LIVE' || s.stamp_ns!==t.stamp_ns)) return [];
  return s.data || [];
}
function assemble() {
  linked={};
  ['tracking','prior','fusion'].forEach(source=>active(source).forEach(t=>{(linked[t.key] ||= {identity:t})[source]=t;}));
  rows=Object.values(linked).sort((a,b)=>a.identity.slot_idx-b.identity.slot_idx);
  if (selected && !linked[selected]) { selected=null; candidatePage=0; }
}
function paginate(items,page,size,prefix) {
  const count=Math.max(1,Math.ceil(items.length/size)),current=Math.max(0,Math.min(page,count-1));
  $(prefix+'-page').textContent=`${current+1} / ${count}`;
  $(prefix+'-prev').disabled=current===0;$(prefix+'-next').disabled=current===count-1;
  return {current,items:items.slice(current*size,(current+1)*size)};
}
function render() {
  if (!state) return;
  assemble();
  const own=state.match?.own_team;
  $('team-context').textContent=own?`己方：${own===1?'红方 R':'蓝方 B'}　敌方：${own===1?'蓝方 B':'红方 R'}`:'阵营未配置 · 等待 /match_state';
  $('team-context').className=own===1?'team-red':own===2?'team-blue':'';
  $('modules').innerHTML=Object.entries(state.modules).map(([name,m])=>`<div class="module" title="${esc(m.note || m.topic || '预留 Adapter')} · ${m.status} · ${fmt(m.age_s,1)}s"><div class="module-name">${moduleNames[name]} <small>${name}</small></div><div class="module-status ${m.status.toLowerCase()}">${statusNames[m.status]} ${m.status==='NOT IMPLEMENTED'?'NOT IMPLEMENTED':m.status}</div></div>`).join('');
  renderTargets();
  const sourceLabels={detection:'检测',timing:'性能',measurement:'定位',tracking:'跟踪',prior:'猜点',fusion:'融合',decision:'决策'};
  $('source-status').innerHTML=Object.entries(state.sources).map(([k,s])=>`<span class="source ${s.status.toLowerCase()}" title="${esc(s.topic)} · ${statusNames[s.status]} ${s.status} · ${fmt(s.age_s,1)}s · 时间戳(ns) ${esc(s.stamp_ns)} · 标定版本 ${esc(s.calibration_version)}">● ${sourceLabels[k] || k}</span>`).join('');
  const d=state.sources.decision;
  const tactics={engineer_on_island:'工程上岛',opponent_attack:'敌方进攻',our_attack:'己方进攻',opponent_near_fortress:'敌近堡垒'};
  $('decision').textContent=d.status==='LIVE'?Object.entries(d.data).map(([k,v])=>`${tactics[k]}：${yes(v)}`).join(' · '):'决策：等待新鲜战术数据';
  $('decision').title=d.status==='LIVE'?Object.entries(d.data).map(([k,v])=>`${tactics[k]} (${k})=${v}`).join('\n'):'';
  drawMap();inspector();
}
function renderTargets() {
  $('target-count').textContent=`${rows.length} 个目标`;
  const page=paginate(rows,targetPage,targetPageSize,'targets');targetPage=page.current;
  $('targets').innerHTML=page.items.map(r=>{
    const t=r.identity,ts=r.tracking?.tracking_state,fs=r.fusion?.source;
    return `<tr data-key="${esc(t.key)}" class="${teamClass(t)} ${t.key===selected?'selected':''}"><td><button data-select="${esc(t.key)}" aria-label="选择 ${esc(robot(t))}"><span class="team-tag">${code(t)}<span class="relationship">${relation(t)}</span></span><div class="target-name">${role(t)}</div></button></td><td title="track_id">${t.track_id}</td><td title="${esc(ts || '无当前轨迹')}">${trackingNames[ts] || '无轨迹'}</td><td title="lost_duration">${fmt(r.tracking?.lost_duration ?? r.prior?.lost_duration,1)}</td><td title="${esc(fs || '无同帧融合输出')}">${sourceNames[fs] || '待同帧'}</td></tr>`;
  }).join('') || '<tr><td class="empty" colspan="5">暂无新鲜目标数据</td></tr>';
  // First real row: measure it so the page capacity matches the panel.
  if(page.items.length && !rowHeight) fitTargetPage();
}
function project(p) {
  if (!valid(p) || !state.field.map) return null;
  const f=state.field,w=f.map.height,h=f.map.width;
  let x=p[0]*w/f.width+w/2,y=p[1]*h/f.length+h/2;
  if (flipped) {x=w-1-x;y=h-1-y;}
  return x>=0&&y>=0&&x<w&&y<h?[x,y]:null;
}
function marker(p,layer,key,title,kind=layer,small=false,target=null) {
  const point=project(p);if(!point)return '';
  const [x,y]=point,c=teamColors[target?.team_id] || teamColors[0],r=small?3:6;
  let shape;
  if(kind==='tracking')shape=`<rect x="-6" y="-6" width="12" height="12" fill="none" stroke="${c}" stroke-width="2.5"/>`;
  else if(kind==='prior')shape=`<path d="M0,-8 L8,0 0,8 -8,0Z" fill="${c}" stroke="white" stroke-width="1.5"/>`;
  else if(kind==='last')shape=`<path d="M-7,0 H7 M0,-7 V7" stroke="${c}" stroke-width="2.5"/>`;
  else if(kind==='anchor')shape=`<path d="M-6,-6 L6,6 M-6,6 L6,-6" stroke="${c}" stroke-width="2"/>`;
  else shape=`<circle r="${kind==='fusion'?10:r}" fill="${kind==='fusion'?'none':c}" stroke="${kind==='fusion'?c:'white'}" stroke-width="${kind==='fusion'?2.5:1}"/>`;
  if(target)labels.push({x,y,p,c,key,target,kind,layer,title});
  return `<g class="marker" data-layer="${layer}" data-team="${target?.team_id || 0}" transform="translate(${x},${y})" ${key?`data-select="${esc(key)}" tabindex="0" role="button" aria-label="${esc(title)}"`:''}><title>${esc(title)} · ${pos(p)}</title><circle class="hit" r="13" fill="transparent"/>${shape}${key&&key===selected&&!small?'<circle r="15" fill="none" stroke="white" stroke-dasharray="3 3"/>':''}</g>`;
}
function drawLabels(w,h) {
  // Merge only exactly coincident labels of the same identity. Do not invent
  // measurement-to-track associations; raw measurements keep their own labels.
  const grouped=new Map();
  for(const l of labels){
    const id=`${l.key || 'measurement:'+code(l.target)}:${l.x}:${l.y}`;
    if(grouped.has(id))grouped.get(id).kinds.push(l.kind);
    else grouped.set(id,{...l,kinds:[l.kind]});
  }
  let result='';
  for(const left of [true,false]){
    const side=[...grouped.values()].filter(l=>(l.x<w/2)===left).sort((a,b)=>a.y-b.y);
    const gap=Math.min(38,(h-12)/Math.max(1,side.length));
    let next=4;
    side.forEach((l,i)=>{
      const y=Math.max(next,Math.min(l.y-15,h-34-(side.length-1-i)*gap));next=y+gap;
      const x=left?-165:w+10,lineX=left?-9:w+9;
      const label=`${robot(l.target)} · ${[...new Set(l.kinds)].map(k=>layerNames[k]).join('/')}`;
      const textColor=l.target.team_id===1?'#be2d3d':l.target.team_id===2?'#1d61b4':'#54637a';
      result+=`<path class="leader" stroke="${l.c}" d="M${l.x},${l.y} L${lineX},${y+16}"/><g class="map-label" data-team="${l.target.team_id}" ${l.key?`data-select="${esc(l.key)}"`:''} transform="translate(${x},${y})"><title>${esc(l.title)} · ${pos(l.p)} · ${relation(l.target)}</title><rect width="155" height="34" rx="4" fill="${l.target.team_id===1?'#fff0f1':l.target.team_id===2?'#eaf3ff':'#f8f9fb'}" stroke="${l.c}" stroke-width=".7"/><text x="5" y="14" fill="${textColor}" textLength="${Math.max(80,Math.min(144,label.length*9))}" lengthAdjust="spacingAndGlyphs">${esc(label)}</text><text class="coord" x="5" y="29" fill="${textColor}">${pos(l.p)}</text></g>`;
    });
  }
  return result;
}
function drawMap() {
  const map=state.field.map;
  if(!map){$('map').innerHTML='';mapSignature='';$('map-empty').textContent='未配置 Qt 底图（map_config）';return;}
  const w=map.height,h=map.width;
  $('map').setAttribute('viewBox',`-170 0 ${w+340} ${h}`);
  const rotate=map.clockwise?`translate(${w},0) rotate(90)`:`translate(0,${h}) rotate(-90)`;
  const signature=JSON.stringify(map);
  if(signature!==mapSignature){
    $('map').innerHTML=`<g id="map-base"><image href="/map.png" width="${h}" height="${w}" transform="${rotate}"/></g><g id="map-overlay"></g><g id="map-labels"></g>`;mapSignature=signature;
  }
  $('map-base').setAttribute('transform',flipped?`translate(${w},${h}) rotate(180)`:'');
  labels=[];let svg='';
  if(enabled.measurement)svg+=active('measurement').filter(m=>!m.negative).map(m=>marker(m.position,'measurement',null,`${robot(m)} 原始测量（未关联轨迹）`,'measurement',false,m)).join('');
  for(const layer of ['tracking','prior','fusion'])if(enabled[layer])svg+=active(layer).map(t=>marker(t.position,layer,t.key,`${robot(t)} ${layerNames[layer]} #${t.track_id}`,layer,false,t)).join('');
  const item=linked[selected];
  if(item){
    if(enabled.measurement&&item.tracking)svg+=marker(item.tracking.measurement_position,'measurement',selected,'最后真实测量','last',false,item.identity);
    if(enabled.prior&&item.prior){
      const p=item.prior;
      svg+=marker(p.last_position,'prior',selected,'最后可靠观测锚点（跟踪输出）','anchor',false,item.identity);
      if(enabled.tracking&&!valid(item.tracking?.position))svg+=marker(p.tracker_position,'tracking',selected,'猜点缓存的 Kalman 预测','tracking',false,item.identity);
      svg+=p.candidates.map((c,i)=>marker(c.position,'prior',selected,`${robot(item.identity)} 候选${i+1} · 概率 ${fmt(c.probability)} · 可达 ${yes(c.reachable)} · 阻挡 ${yes(c.blocked)}`,'candidate',true,item.identity)).join('');
    }
  }
  $('map-overlay').innerHTML=svg;$('map-labels').innerHTML=drawLabels(w,h);
  $('map-empty').textContent=labels.length?'':'暂无新鲜目标 · 等待话题';
  $('map-note').textContent=`${state.field.length} × ${state.field.width} m · world X 向右 / Z 向下${flipped?' · 视角已旋转':''} · 同色为同一阵营，形状区分来源`;
}
function inspector() {
  const r=linked[selected],t=r?.tracking,p=r?.prior,f=r?.fusion;
  $('selection-name').textContent=r?`${robot(r.identity)} · ${relation(r.identity)}`:'未选择目标';
  $('selection-name').className=r?teamClass(r.identity):'';
  $('selection-hint').textContent=r?`槽位 slot ${r.identity.slot_idx} · 悬停可查看完整字段与时间戳`:'点击地图点或列表，查看位置来源。';
  $('identity').innerHTML=dl([
    ['轨迹编号','track_id',r?.identity.track_id],['兵种','class',r?robot(r.identity):'—'],
    ['跟踪状态','tracking_state',t?`${trackingNames[t.tracking_state]} ${t.tracking_state}`:'—'],
    ['本帧观测','observed',yes(t?.observed)],['失联时长','lost_duration / s',fmt(t?.lost_duration??p?.lost_duration)],
    ['速度','velocity / m/s',pos(t?.velocity)],['检测置信度','detection',fmt(t?.detection_confidence)],
    ['跟踪置信度','tracking',fmt(t?.tracking_confidence)],['猜点置信度','prior',fmt(p?.confidence)]]);
  const rejection=p?rejectionNames[p.rejection_code] || '未知拒绝原因':'无对应猜点';
  const steps=[
    ['measurement','真实测量 Measurement',t?.measurement_position,'最近一次原始测量',t?.measurement_stamp_ns],
    ['tracking','卡尔曼跟踪 Kalman',t?.position,`${trackingNames[t?.tracking_state] || '无当前轨迹'}`,t?state.sources.tracking.stamp_ns:null],
    ['prior','位置猜点 Prior',p?.position,rejection,p?state.sources.prior.stamp_ns:null],
    ['fusion','融合结果 Fusion',f?.position,f?`来源 ${sourceNames[f.source]} (${f.source}) · 置信度 ${fmt(f.confidence)}`:'无同帧、同身份的融合输出',f?.source_stamp_ns]];
  $('pipeline').innerHTML=steps.map(([c,title,position,note,stamp])=>`<div class="step" style="--color:${r?(teamColors[r.identity.team_id]||'#7b899b'):colors[c]}" title="${esc(note)} · 时间戳 stamp(ns): ${esc(stamp)}"><strong>${title}</strong><div class="position">${pos(position)}</div><p class="${c==='prior'&&p&&!p.valid?'rejected':''}">${esc(note)}</p></div>`).join('');
  const model=state.sources.prior.model_status||'未收到模型状态';
  $('prior-model').textContent=state.sources.prior.model_enabled?'模型已加载':'模型未就绪';$('prior-model').title=model;
  $('prior-details').innerHTML=dl([
    ['分布熵','entropy',fmt(p?.entropy)],['可达质量','mass',fmt(p?.reachable_mass)],
    ['样本数量','samples',p?.sample_count],['预测时域','horizon / s',p?.horizon_seconds],
    ['运动门控','motion gate',yes(p?.motion_gate)],['导航网格','mesh',yes(p?.mesh_used)],
    ['盲区偏置','blind bias',yes(p?.blind_zone_bias)],['盲区质量','blind mass',fmt(p?.blind_zone_mass)],
    ['驻留质量','stay mass',fmt(p?.stay_anchor_mass)],['猜点置信','confidence',fmt(p?.confidence)]]);
  $('rejection').textContent=`门控结果：${rejection}${p?.rejection_reason?' · '+p.rejection_reason:''}`;
  $('rejection').title=`拒绝原因 rejection_reason：${p?.rejection_reason || '—'}\n最后可靠观测锚点 last_position：${pos(p?.last_position)}\n卡尔曼缓存 tracker_position：${pos(p?.tracker_position)}`;
  const size=window.innerHeight<760?2:3;
  const page=paginate(p?.candidates || [],candidatePage,size,'candidates');candidatePage=page.current;
  $('candidates').innerHTML=page.items.map((c,i)=>`<div class="candidate" title="网格 grid_index=${c.grid_index} · 原始概率 prior_probability=${fmt(c.prior_probability)} · 门控概率 fused_probability=${fmt(c.probability)} · 距离 distance=${fmt(c.distance)}m"><div class="candidate-line"><span>候选 ${candidatePage*size+i+1} · ${pos(c.position)}</span><strong>${fmt(c.probability)}</strong></div><div class="flags ${c.blocked?'blocked':''}">可达 ${yes(c.reachable)} · 阻挡 ${yes(c.blocked)} · 盲区 ${yes(c.blind_zone)} · 驻留 ${yes(c.stay_anchor)}</div></div>`).join('') || '<p class="hint">暂无候选数据</p>';
}
function select(event) {
  const target=event.target.closest('[data-select],tr[data-key]');if(!target)return;
  selected=target.dataset.select||target.dataset.key;candidatePage=0;
  const index=rows.findIndex(r=>r.identity.key===selected);if(index>=0)targetPage=Math.floor(index/targetPageSize);
  render();
}
$('map').addEventListener('click',select);$('targets').addEventListener('click',select);
$('map').addEventListener('keydown',e=>{if(e.key==='Enter'||e.key===' '){e.preventDefault();select(e);}});
$('layers').addEventListener('change',e=>{enabled[e.target.dataset.layer]=e.target.checked;if(state)drawMap();});
$('flip').addEventListener('click',()=>{flipped=!flipped;$('flip').setAttribute('aria-pressed',String(flipped));if(state)drawMap();});
for(const prefix of ['targets','candidates'])for(const direction of ['prev','next'])$(prefix+'-'+direction).addEventListener('click',()=>{
  const delta=direction==='next'?1:-1;
  if(prefix==='targets'){targetPage+=delta;renderTargets();}else{candidatePage+=delta;inspector();}
});
function fitTargetPage(){
  const view=document.querySelector('.table-scroll');
  if(!view)return;
  const sample=$('targets').querySelector('tr[data-key]');
  if(sample)rowHeight=sample.getBoundingClientRect().height;
  const available=view.clientHeight;
  const size=rowHeight>0&&available>0
    ? Math.max(2,Math.min(16,Math.floor(available/rowHeight)))
    : 8;
  if(size!==targetPageSize){targetPageSize=size;if(state)renderTargets();}
}
new ResizeObserver(fitTargetPage).observe(document.querySelector('.table-scroll'));
function connection(live,label){$('connection').textContent=label;$('connection-dot').classList.toggle('live',live);}
function stop(){streams.forEach(s=>s.close());streams=[];}
function connect(){
  stop();const radar=new EventSource('/api/radar-stream'),perf=new EventSource('/api/performance-stream');streams=[radar,perf];
  radar.addEventListener('radar',e=>{state=JSON.parse(e.data);lastRadar=Date.now();connection(true,'SSE 已连接');render();});
  perf.addEventListener('performance',e=>performance(JSON.parse(e.data)));
  radar.onerror=()=>{connection(false,'连接中断 · 自动重连');expire();};
  perf.onerror=()=>{$('performance-status').textContent='性能流已断开';metrics.forEach(k=>$('metric-'+k).textContent='—');};
}
function expire(){
  if(!state)return;
  Object.values(state.sources).forEach(s=>{if(s.status==='LIVE')s.status='STALE';});
  Object.values(state.modules).forEach(m=>{if(['LIVE','DISABLED'].includes(m.status))m.status='STALE';});render();
}
document.addEventListener('visibilitychange',()=>{if(document.hidden){stop();connection(false,'页面隐藏 · 已暂停连接');expire();}else connect();});
window.addEventListener('pagehide',stop);
setInterval(()=>{if(lastRadar&&Date.now()-lastRadar>4000){connection(false,document.hidden?'页面隐藏 · 已暂停连接':'未收到更新 · 等待重连');expire();}},2000);
connect();
