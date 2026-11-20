'use strict';
const $ = id => document.getElementById(id);
const icon = (name, small = false) => `<svg class="icon${small ? ' small' : ''}" aria-hidden="true"><use href="#i-${name}"/></svg>`;
const colors = {measurement:'#95aaa1', tracking:'#95aaa1', prior:'#e5bd7e', fusion:'#d3ef9b'};
const teamColors = {1:'#ed8992', 2:'#8ab5ed', 0:'#a0aca8'};
const roles = {0:['?','车辆'],1:['?','装甲板'],2:['1','英雄'],3:['2','工程'],4:['3','步兵'],5:['4','步兵'],6:['S','哨兵'],7:['O','前哨站']};
const metricNames = {camera_fps:'采集帧率',fps:'检测帧率',dropped_fps:'丢帧速率',dropped_count:'累计丢帧',car_ms:'车辆检测 / ms',armor_ms:'装甲检测 / ms',cls_ms:'兵种分类 / ms',total_ms:'推理总耗时 / ms',end_to_end_ms:'端到端耗时 / ms'};
const moduleNames = {Camera:'输入',Detection:'检测',Localization:'定位',Tracking:'跟踪',Prior:'猜点',Fusion:'融合',Decision:'决策',LiDAR:'激光雷达',Radio:'无线电',Referee:'裁判'};
const moduleIcons = {Camera:'camera',Detection:'target',Localization:'pin',Tracking:'tracking',Prior:'prior',Fusion:'fusion',Decision:'branch',LiDAR:'lidar',Radio:'radio',Referee:'flag'};
const statusNames = {LIVE:'活跃',STALE:'已过期',WAITING:'等待数据',DISABLED:'已禁用','NOT IMPLEMENTED':'未实现'};
const trackingNames = {ACTIVE:'跟踪中',PREDICTED:'预测中',LOST:'已丢失',DEAD:'已死亡',INVALID:'无效'};
const sourceNames = {TRACKED:'真实跟踪',PREDICTED:'预测',PRIOR:'位置猜点',INVALID:'无有效位置'};
const rejectionNames = {0:'通过门控',1:'尚未到预测窗口',2:'超出预测窗口',3:'无可靠观测',4:'不支持的兵种',5:'超出场地',6:'无猜点分布',7:'候选不可达',8:'置信度不足',9:'模型未启用'};
const layerNames = {measurement:'测量',tracking:'跟踪',prior:'猜点',fusion:'融合',last:'最后测量',anchor:'观测锚点',candidate:'候选'};
const metrics = Object.keys(metricNames);
const enabled = {measurement:true,tracking:true,prior:true,fusion:true};
const performanceHistory = [];
const renderedHTML = new WeakMap();
let state = null, selected = null, flipped = false, streams = [], rows = [], linked = {};
let lastRadar = 0, lastPerformance = 0, performanceStamp = null;
let targetPage = 0, candidatePage = 0, targetPageSize = 6, candidateSelected = null;
let filter = 'all', zoom = 1, markerCount = 0, paused = false, toastTimer, mapSignature = '';

const esc = value => String(value ?? '—').replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const finite = value => typeof value === 'number' && Number.isFinite(value);
const fmt = (value, n = 2) => finite(value) ? value.toFixed(n) : '—';
const yes = value => value == null ? '—' : value ? '是' : '否';
const valid = point => Array.isArray(point) && point.length === 2 && point.every(finite);
const pos = point => valid(point) ? `(${fmt(point[0])}, ${fmt(point[1])})` : '—';
const code = target => `${({1:'R',2:'B'})[target.team_id] || 'U'}${roles[target.class_id]?.[0] || '?'}`;
const role = target => roles[target.class_id]?.[1] || '未知兵种';
const robot = target => `${code(target)} ${role(target)}`;
const teamClass = target => ({1:'team-red',2:'team-blue'})[target.team_id] || 'team-unknown';
const relation = target => ![1,2].includes(target.team_id) ? '阵营未知' : !state?.match?.own_team ? '敌我未定' : target.team_id === state.match.own_team ? '己方' : '敌方';
const dl = pairs => pairs.map(([cn,en,value]) => `<div class="detail-pair"><dt title="${esc(cn+' / '+en)}">${esc(cn)}</dt><dd title="${esc(value)}">${esc(value)}</dd></div>`).join('');
const statusClass = status => status === 'LIVE' ? 'green' : ['STALE','DISABLED'].includes(status) ? 'amber' : 'dim';

function setHTML(element, html) {
  if (renderedHTML.get(element) !== html) {
    element.innerHTML = html;
    renderedHTML.set(element,html);
  }
}
// Preserve controls during SSE updates so a refresh cannot interrupt a click or focus.
function syncChildren(parent, entries, tag, update) {
  const existing = new Map(Array.from(parent.children).map(element => [element.dataset.renderKey,element]));
  const retained = new Set();
  entries.forEach((entry,index) => {
    let element = existing.get(entry.key);
    if (!element) {
      element = document.createElement(tag);
      element.dataset.renderKey = entry.key;
    }
    retained.add(element);
    update(element,entry);
    if (parent.children[index] !== element) parent.insertBefore(element,parent.children[index] || null);
  });
  Array.from(parent.children).forEach(element => { if (!retained.has(element)) element.remove(); });
}

function setTheme(dark) {
  document.documentElement.dataset.theme = dark ? 'dark' : 'light';
  const label = dark ? '切换浅色模式' : '切换深色模式';
  $('theme-toggle').setAttribute('aria-pressed', String(dark));
  $('theme-toggle').setAttribute('aria-label', label);
  $('theme-toggle').title = label;
  $('theme-toggle').innerHTML = icon(dark ? 'sun' : 'moon');
}
try { setTheme(localStorage.getItem('radar27-theme') !== 'light'); } catch { setTheme(true); }
$('theme-toggle').addEventListener('click', () => {
  const dark = document.documentElement.dataset.theme !== 'dark';
  setTheme(dark);
  try { localStorage.setItem('radar27-theme', dark ? 'dark' : 'light'); } catch {}
});

function drawTrend(id, key, label) {
  const values = performanceHistory.map(sample => sample[key]);
  const points = values.filter(finite);
  const chart = $(id);
  chart.setAttribute('aria-label', points.length ? `${label}，最近 ${values.length} 次性能快照` : `等待${label}数据`);
  if (points.length < 2) { chart.innerHTML = ''; return; }
  const low = Math.min(...points), high = Math.max(...points), range = Math.max(high-low, 1);
  let path = '', connected = false;
  values.forEach((value,index) => {
    if (!finite(value)) { connected = false; return; }
    const x = index*94/Math.max(1,values.length-1), y = 23-(value-low)/range*18;
    path += `${connected?'L':'M'}${x.toFixed(2)},${y.toFixed(2)} `;
    connected = true;
  });
  chart.innerHTML = `<line x1="0" y1="27" x2="94" y2="27"/><path d="${path}"/>`;
}
function performance(data) {
  const source = data.performance;
  if (!source) return;
  lastPerformance = Date.now();
  const live = source.status === 'LIVE', values = source.data;
  metrics.forEach(key => { $('metric-'+key).textContent = live ? fmt(values?.[key],key === 'dropped_count' ? 0 : 1) : '—'; });
  $('performance-status').textContent = `${statusNames[source.status] || source.status} · PipelineTiming · ${fmt(source.age_s,1)} s`;
  $('performance-status').className = `metric-caption ${statusClass(source.status)}`;
  if (live && values && source.stamp_ns !== performanceStamp) {
    performanceHistory.push({fps:values.fps,end_to_end_ms:values.end_to_end_ms});
    if (performanceHistory.length > 30) performanceHistory.shift();
    performanceStamp = source.stamp_ns;
  } else if (!live) { performanceHistory.length = 0; performanceStamp = null; }
  $('history-count').textContent = String(performanceHistory.length);
  drawTrend('fps-history','fps','检测帧率');
  drawTrend('latency-history','end_to_end_ms','端到端延迟');
  const stages = [['car_ms','车辆','#b9d68e'],['armor_ms','装甲','#8cabb8'],['cls_ms','分类','#777f8f']];
  const known = live ? stages.filter(([key]) => finite(values?.[key]) && values[key] >= 0) : [];
  const sum = known.reduce((total,[key]) => total+values[key],0);
  const total = Math.max(sum, live && finite(values?.total_ms) ? values.total_ms : 0);
  $('stage-bar').innerHTML = total > 0 ? known.map(([key,,color]) => `<span style="width:${values[key]/total*100}%;background:${color}"></span>`).join('') : '';
  $('stage-bar').setAttribute('aria-label', known.length ? stages.map(([key,name]) => `${name} ${fmt(values[key],1)} ms`).join('，') : '等待推理耗时数据');
}
function clearPerformance(label) {
  performance({performance:{status:'STALE',age_s:null,data:null}});
  $('performance-status').textContent = label;
  lastPerformance = 0;
}
function active(name) {
  const source = state?.sources[name], tracking = state?.sources.tracking;
  if (!source || source.status !== 'LIVE') return [];
  if (name === 'prior' && source.model_enabled === false) return [];
  if (['prior','measurement'].includes(name) && tracking?.calibration_version != null && source.calibration_version !== tracking.calibration_version) return [];
  // Fusion has no calibration epoch; preserve exact live Tracking-frame matching.
  if (name === 'fusion' && (tracking?.status !== 'LIVE' || tracking.stamp_ns == null || source.stamp_ns !== tracking.stamp_ns)) return [];
  return Array.isArray(source.data) ? source.data : [];
}
function assemble() {
  linked = {};
  ['tracking','prior','fusion'].forEach(source => active(source).forEach(target => {
    (linked[target.key] ||= {identity:target})[source] = target;
  }));
  rows = Object.values(linked).sort((a,b) => a.identity.slot_idx-b.identity.slot_idx);
  if (selected && !linked[selected]) { selected = null; candidatePage = 0; candidateSelected = null; }
}
function paginate(items, page, size, prefix) {
  const count = Math.max(1,Math.ceil(items.length/size)), current = Math.max(0,Math.min(page,count-1));
  $(prefix+'-page').textContent = `${current+1} / ${count}`;
  $(prefix+'-prev').disabled = current === 0;
  $(prefix+'-next').disabled = current === count-1;
  return {current,items:items.slice(current*size,(current+1)*size)};
}
function renderModules() {
  const modules = Object.entries(moduleNames).filter(([name]) => state.modules[name]);
  $('modules').innerHTML = modules.map(([name,cn]) => {
    const module = state.modules[name], status = module.status;
    const title = `${module.note || module.topic || '预留 Adapter'} · ${status} · ${fmt(module.age_s,1)} s`;
    return `<div class="system-module ${status === 'NOT IMPLEMENTED' ? 'reserved' : ''}" title="${esc(title)}"><span class="module-icon">${icon(moduleIcons[name],true)}</span><div><div class="module-name">${cn}</div><div class="module-status ${statusClass(status)}"><i class="dot"></i>${esc(status === 'NOT IMPLEMENTED' ? 'RESERVED' : status)}</div></div></div>`;
  }).join('');
  $('health-list').innerHTML = modules.map(([name,cn]) => {
    const module = state.modules[name];
    return `<div class="health-row" title="${esc(module.note || module.topic || '预留 Adapter')} · ${fmt(module.age_s,1)} s"><span>${cn}</span><span class="mono">${name}</span><span class="state-label ${statusClass(module.status)}"><i class="dot"></i>${esc(statusNames[module.status] || module.status)}</span></div>`;
  }).join('');
}
function render() {
  if (!state) return;
  assemble();
  const own = state.match?.own_team;
  $('team-context').textContent = own ? `己方：${own === 1 ? '红方 R' : '蓝方 B'} / 敌方：${own === 1 ? '蓝方 B' : '红方 R'}` : '阵营未配置 · 等待 /match_state';
  $('team-context').className = `team-context ${own === 1 ? 'team-red' : own === 2 ? 'team-blue' : 'dim'}`;
  renderModules();
  renderTargets();
  const sourceLabels = {detection:'检测',timing:'性能',measurement:'定位',tracking:'跟踪',prior:'猜点',fusion:'融合',decision:'决策'};
  $('source-status').innerHTML = Object.entries(state.sources).map(([key,source]) => `<span class="source-item ${statusClass(source.status)}" title="${esc(source.topic)} · ${esc(statusNames[source.status] || source.status)} ${esc(source.status)} · ${fmt(source.age_s,1)} s · 时间戳(ns) ${esc(source.stamp_ns)} · 标定版本 ${esc(source.calibration_version)}"><i class="dot"></i>${esc(sourceLabels[key] || key)}</span>`).join('');
  $('calibration-status').textContent = `CALIBRATION ${state.sources.tracking?.calibration_version ?? '—'}`;
  $('field-size').textContent = `${fmt(state.field.length,0)} × ${fmt(state.field.width,0)} m`;
  renderDecision(state.sources.decision);
  drawMap();
  inspector();
}
function renderDecision(source) {
  const panel = $('decision');
  panel.title = '';
  if (source?.status !== 'LIVE' || !source.data || source.data.input_fresh === false) {
    panel.textContent = '等待新鲜战术数据';
    return;
  }
  const data = source.data;
  if (Array.isArray(data.regions)) {
    const team = ({1:'红方',2:'蓝方'})[data.own_team] || '未知';
    const names = new Map(data.regions.map(region => [region.id,region.name || region.id]));
    const busiest = (data.busiest_region_ids || []).map(id => names.get(id) || id).join('、') || '无';
    panel.textContent = [`决策己方：${team}`,`己方工程上岛：${yes(data.our_engineer_on_island)}`,`敌方工程上岛：${yes(data.enemy_engineer_on_island)}`,
      ...data.regions.map(region => `${region.name || region.id}：敌方 ${region.enemy_count}`),`敌方最多区域：${busiest}`].join(' · ');
    panel.title = panel.textContent+`\n边界容差：${fmt(data.boundary_margin_m)} m\n`+data.regions.map(region => `${region.name || region.id}：track_id ${(region.enemy_track_ids || []).join(', ') || '无'}`).join('\n');
  } else {
    const tactics = {engineer_on_island:'工程上岛',opponent_attack:'敌方进攻',our_attack:'己方进攻',opponent_near_fortress:'敌近堡垒'};
    panel.textContent = Object.entries(tactics).map(([key,label]) => `${label}：${yes(data[key])}`).join(' · ');
  }
}
function filteredRows() {
  const query = $('search').value.trim().toLowerCase();
  return rows.filter(row => (filter === 'all' || String(row.identity.team_id) === filter) && (!query || `${robot(row.identity)} ${row.identity.track_id}`.toLowerCase().includes(query)));
}
function renderTargets() {
  const visible = filteredRows(), page = paginate(visible,targetPage,targetPageSize,'targets');
  targetPage = page.current;
  $('target-count').textContent = `${rows.length} 个目标`;
  $('predicted-count').textContent = `${rows.filter(row => row.tracking?.tracking_state === 'PREDICTED').length} 个预测中`;
  $('table-count').textContent = visible.length ? `显示 ${targetPage*targetPageSize+1}–${targetPage*targetPageSize+page.items.length} / ${visible.length} 个目标` : '0 个目标';
  const entries = page.items.map(row => {
    const target = row.identity, tracking = row.tracking?.tracking_state, source = row.fusion?.source;
    return {key:target.key,target,tracking,cells:[
      `<button class="robot-cell" type="button" data-select="${esc(target.key)}" aria-label="选择 ${esc(robot(target))}"><span class="robot-badge ${teamClass(target)}">${code(target)}</span><span><span class="robot-name">${role(target)}</span><span class="robot-team">${relation(target)}</span></span></button>`,
      esc(target.track_id),
      `<span class="state-label ${tracking === 'ACTIVE' ? 'green' : tracking === 'PREDICTED' ? 'amber' : 'dim'}"><i class="dot"></i>${esc(trackingNames[tracking] || '无轨迹')}</span>`,
      `${fmt(row.tracking?.lost_duration ?? row.prior?.lost_duration,1)} <span class="detail-unit">s</span>`,
      pos(row.fusion?.position ?? row.tracking?.position ?? row.prior?.position),
      `<span class="source-tag ${source === 'PRIOR' ? 'prior' : ''}">${esc(sourceNames[source] || '待同帧')}</span>`]};
  });
  if (!entries.length) setHTML($('targets'),`<tr><td class="empty-row" colspan="6">${rows.length ? '没有匹配的目标，请调整筛选条件' : '暂无新鲜目标数据'}</td></tr>`);
  else {
    renderedHTML.delete($('targets'));
    syncChildren($('targets'),entries,'tr',(element,entry) => {
      element.dataset.key = entry.key;
      element.className = `target-row ${teamClass(entry.target)} ${entry.key === selected ? 'selected' : ''}`;
      element.setAttribute('aria-selected',String(entry.key === selected));
      entry.cells.forEach((html,index) => {
        let cell = element.children[index];
        if (!cell) { cell = document.createElement('td'); element.append(cell); }
        cell.className = ['', 'mono muted', '', `mono ${entry.tracking === 'PREDICTED' ? 'amber' : 'dim'}`, 'mono muted', ''][index];
        setHTML(cell,html);
      });
    });
  }
  fitTargetPage();
}
// Compute the same Qt world pixels first, then turn that view 90° counterclockwise.
// This changes presentation only: horizontal screen X = Qt v, Y = Qt W - Qt u.
function project(point) {
  if (!valid(point) || !state?.field.map) return null;
  const field = state.field, width = field.map.height, height = field.map.width;
  if (!(width > 0 && height > 0 && field.width > 0 && field.length > 0)) return null;
  let u = point[0]*width/field.width+width/2, v = point[1]*height/field.length+height/2;
  if (flipped) { u = width-1-u; v = height-1-v; }
  return u >= 0 && v >= 0 && u < width && v < height ? [v,width-u] : null;
}
function marker(point, layer, key, title, kind = layer, small = false, target = null, highlighted = false, labeled = true) {
  const projected = project(point);
  if (!projected) return '';
  markerCount++;
  const [x,y] = projected, color = teamColors[target?.team_id] || teamColors[0];
  const main = ['measurement','tracking','prior','fusion'].includes(kind);
  const isSelected = key != null && key === selected;
  let shape;
  if (kind === 'last') shape = `<path d="M-6,0 H6 M0,-6 V6" stroke="${color}" stroke-width="1.8"/>`;
  else if (kind === 'anchor') shape = `<path d="M-5,-5 L5,5 M-5,5 L5,-5" stroke="${color}" stroke-width="1.8"/>`;
  else if (small) shape = `<circle r="3" fill="${color}" stroke="#152022" stroke-width="1"/>${highlighted ? '<circle r="8" class="selection-ring"/>' : ''}`;
  else if (kind === 'measurement') shape = `<circle r="3.5" fill="${color}" stroke="#172126" stroke-width="1.2"/>`;
  else {
    const outline = kind === 'tracking' ? '<rect x="-15" y="-15" width="30" height="30" rx="3"/>' : kind === 'prior' ? '<path d="M0,-22 L22,0 0,22 -22,0Z"/>' : kind === 'fusion' ? '<circle r="17"/>' : '';
    shape = `<g fill="none" stroke="${color}" stroke-width="1.1">${outline}</g>`;
    shape += labeled ? `<circle class="marker-core" r="12" fill="${color}" stroke="#172126" stroke-width="1.5"/><text class="marker-role" text-anchor="middle" dy=".35em">${esc(target ? code(target) : '?')}</text>` : `<circle r="2.5" fill="${color}"/>`;
  }
  const description = `${title} · ${pos(point)}${target ? ' · '+relation(target) : ''}`;
  return `<g class="marker ${isSelected ? 'is-selected' : ''}" data-layer="${layer}" data-team="${target?.team_id || 0}" transform="translate(${x},${y})" ${key && main ? `data-select="${esc(key)}" tabindex="0" role="button" aria-label="${esc(description)}"` : ''}><title>${esc(description)}</title><circle class="hit" r="${main ? 23 : 8}" fill="transparent"/>${isSelected && main && labeled ? '<circle class="selection-ring" r="25"/>' : ''}${shape}</g>`;
}
function drawMap() {
  if (!state) return;
  const map = state.field.map;
  if (!map) {
    $('map').innerHTML = '';
    mapSignature = '';
    $('map-empty').textContent = '未配置 Qt 底图（map_config）';
    $('map-note').textContent = '请配置 Qt 原始底图以显示世界坐标。';
    $('map-status').textContent = '无底图';
    return;
  }
  const width = map.width, height = map.height;
  const viewWidth = width/zoom, viewHeight = height/zoom;
  $('map').setAttribute('viewBox', `${(width-viewWidth)/2-24} ${(height-viewHeight)/2-24} ${viewWidth+48} ${viewHeight+48}`);
  $('map').style.setProperty('--map-aspect',`${width+48} / ${height+48}`);
  // Undo Qt's 90° presentation rotation. A counterclockwise Qt base is 180° here.
  const baseFlipped = flipped !== !map.clockwise;
  const signature = JSON.stringify(map);
  if (signature !== mapSignature) {
    $('map').innerHTML = `<g id="map-base"><image class="field-image" href="/map.png" width="${width}" height="${height}"/></g><g id="map-overlay"></g>`;
    mapSignature = signature;
  }
  $('map-base').setAttribute('transform',baseFlipped ? `translate(${width},${height}) rotate(180)` : '');
  markerCount = 0;
  let svg = '', foreground = '';
  // Keep every layer at its exact coordinate, but label each identity once.
  const labelLayers = new Map();
  for (const layer of ['prior','tracking','fusion']) if (enabled[layer]) {
    active(layer).forEach(target => { if (project(target.position)) labelLayers.set(target.key,layer); });
  }
  if (enabled.measurement) svg += active('measurement').filter(item => !item.negative).map(item => marker(item.position,'measurement',null,`${robot(item)} 原始测量（未关联轨迹）`,'measurement',false,item)).join('');
  for (const layer of ['tracking','prior','fusion']) if (enabled[layer]) {
    active(layer).forEach(target => {
      const labeled = labelLayers.get(target.key) === layer;
      const glyph = marker(target.position,layer,target.key,`${robot(target)} ${layerNames[layer]} #${target.track_id}`,layer,false,target,false,labeled);
      if (labeled) foreground += glyph;
      else svg += glyph;
    });
  }
  const item = linked[selected];
  if (item) {
    if (enabled.measurement && item.tracking) svg += marker(item.tracking.measurement_position,'measurement',null,'最后真实测量','last',false,item.identity);
    if (enabled.prior && item.prior) {
      const prior = item.prior;
      svg += marker(prior.last_position,'prior',null,'最后可靠观测锚点（跟踪输出）','anchor',false,item.identity);
      if (enabled.tracking && !valid(item.tracking?.position)) svg += marker(prior.tracker_position,'tracking',selected,'猜点缓存的 Kalman 预测','tracking',false,item.identity,false,!labelLayers.has(selected));
      svg += prior.candidates.map((candidate,index) => marker(candidate.position,'prior',null,`${robot(item.identity)} 候选${index+1} · 概率 ${fmt(candidate.probability)} · 可达 ${yes(candidate.reachable)} · 阻挡 ${yes(candidate.blocked)}`,'candidate',true,item.identity,index === candidateSelected)).join('');
    }
  }
  $('map-overlay').innerHTML = svg+foreground;
  $('map-empty').textContent = markerCount ? '' : '暂无新鲜目标 · 等待话题';
  $('map-status').textContent = `${rows.length} 个目标`;
  $('map-note').textContent = `${fmt(state.field.length,0)} × ${fmt(state.field.width,0)} m · world Z 向右 / X 向上${flipped ? ' · 视角已旋转 180°' : ''} · 颜色区分阵营，形状区分来源`;
}
function inspector() {
  const row = linked[selected], tracking = row?.tracking, prior = row?.prior, fusion = row?.fusion;
  $('selection-name').textContent = row ? robot(row.identity) : '未选择目标';
  $('selected-badge').textContent = row ? code(row.identity) : '—';
  $('selected-badge').className = `robot-badge ${row ? teamClass(row.identity) : 'team-unknown'}`;
  $('identity-tag').textContent = row ? relation(row.identity) : '—';
  $('identity-subtitle').textContent = row ? `TRACK #${row.identity.track_id} · SLOT ${row.identity.slot_idx}` : '点击地图点或目标列表';
  $('selection-hint').textContent = row ? `${trackingNames[tracking?.tracking_state] || '无当前轨迹'} · 本帧观测：${yes(tracking?.observed)}` : '点击地图点或列表，查看位置来源。';
  $('identity').innerHTML = dl([
    ['失联时长 / s','lost_duration',fmt(tracking?.lost_duration ?? prior?.lost_duration)],
    ['速度 (x,z) / m/s','velocity',pos(tracking?.velocity)],
    ['跟踪置信度','tracking_confidence',fmt(tracking?.tracking_confidence)],
    ['猜点置信度','prior_confidence',fmt(prior?.confidence)]]);
  $('identity-extra').innerHTML = dl([
    ['轨迹编号','track_id',row?.identity.track_id],['兵种','class',row ? robot(row.identity) : '—'],
    ['跟踪状态','tracking_state',tracking ? `${trackingNames[tracking.tracking_state]} ${tracking.tracking_state}` : '—'],
    ['本帧观测','observed',yes(tracking?.observed)],['检测置信度','detection_confidence',fmt(tracking?.detection_confidence)]]);
  const rejection = prior ? rejectionNames[prior.rejection_code] || '未知拒绝原因' : '无对应猜点';
  const steps = [
    ['measurement','真实测量',tracking?.measurement_position,tracking ? `${tracking.observed ? '最近一次原始测量' : '历史原始测量 · 本帧未观测'}` : '无对应原始测量',tracking?.measurement_stamp_ns],
    ['tracking','卡尔曼跟踪',tracking?.position,trackingNames[tracking?.tracking_state] || '无当前轨迹',tracking ? state.sources.tracking.stamp_ns : null],
    ['prior','位置猜点',prior?.position,rejection,prior ? state.sources.prior.stamp_ns : null],
    ['fusion','融合结果',fusion?.position,fusion ? `来源 ${sourceNames[fusion.source]} (${fusion.source}) · 置信度 ${fmt(fusion.confidence)}` : '无同帧、同身份的融合输出',fusion?.source_stamp_ns]];
  $('pipeline').innerHTML = steps.map(([layer,title,position,note,stamp],index) => `<div class="pipeline-step ${layer === 'fusion' && valid(position) ? 'accepted' : ''}" title="${esc(note)} · 时间戳(ns) ${esc(stamp)}"><span class="step-node">${index === 3 ? icon('check',true) : ['+','□','◇'][index]}</span><div class="step-heading"><span>${title}</span><span class="mono">${layer === 'fusion' ? '最终输出' : pos(position)}</span></div><p class="step-caption ${layer === 'prior' && prior && !prior.valid ? 'rejected' : ''}">${esc(note)}</p>${layer === 'fusion' ? `<div class="fusion-result"><span class="mono">${pos(position)}</span><small>${esc(fusion?.source || '等待同帧')}</small></div>` : ''}</div>`).join('');
  const source = state.sources.prior, model = source.model_status || '未收到模型状态';
  $('prior-model').textContent = source.status !== 'LIVE' ? (statusNames[source.status] || source.status) : source.model_enabled ? '模型已加载' : '模型未就绪';
  $('prior-model').className = `model-tag ${source.status === 'LIVE' && source.model_enabled ? 'green' : 'dim'}`;
  $('prior-model').title = model;
  $('prior-details').innerHTML = dl([
    ['分布熵','entropy',fmt(prior?.entropy)],['可达质量','mass',fmt(prior?.reachable_mass)],
    ['样本数量','samples',prior?.sample_count],['预测时域 / s','horizon',fmt(prior?.horizon_seconds)]]);
  $('prior-extra').innerHTML = dl([
    ['运动门控','motion_gate',yes(prior?.motion_gate)],['导航网格','mesh',yes(prior?.mesh_used)],
    ['盲区偏置','blind_bias',yes(prior?.blind_zone_bias)],['盲区质量','blind_mass',fmt(prior?.blind_zone_mass)],
    ['驻留质量','stay_mass',fmt(prior?.stay_anchor_mass)],['猜点置信','confidence',fmt(prior?.confidence)]]);
  $('rejection').textContent = `门控结果：${rejection}${prior?.rejection_reason ? ' · '+prior.rejection_reason : ''}`;
  $('rejection').classList.toggle('rejected',Boolean(prior && !prior.valid));
  $('rejection').title = `拒绝原因：${prior?.rejection_reason || '—'}\n最后可靠观测锚点：${pos(prior?.last_position)}\nKalman 缓存：${pos(prior?.tracker_position)}`;
  const page = paginate(prior?.candidates || [],candidatePage,3,'candidates');
  candidatePage = page.current;
  if (candidateSelected != null && candidateSelected >= (prior?.candidates.length || 0)) candidateSelected = null;
  const candidates = page.items.map((candidate,index) => {
    const absoluteIndex = candidatePage*3+index, probability = candidate.probability;
    return {key:String(absoluteIndex),candidate,index:absoluteIndex,html:`<span class="candidate-index">${String(absoluteIndex+1).padStart(2,'0')}</span><span><span class="candidate-position">${pos(candidate.position)}</span><span class="candidate-flags ${candidate.blocked ? 'blocked' : ''}">可达 ${yes(candidate.reachable)} · 阻挡 ${yes(candidate.blocked)}<br>盲区 ${yes(candidate.blind_zone)} · 驻留 ${yes(candidate.stay_anchor)}</span></span><span class="candidate-probability">${fmt(probability)}<span class="probability-bar"><i style="width:${finite(probability) ? Math.max(0,Math.min(1,probability))*100 : 0}%"></i></span></span>`};
  });
  if (!candidates.length) setHTML($('candidates'),'<p class="hint candidate-empty">暂无对应候选数据</p>');
  else {
    renderedHTML.delete($('candidates'));
    syncChildren($('candidates'),candidates,'button',(element,entry) => {
      element.className = 'candidate-row';
      element.type = 'button';
      element.dataset.candidate = String(entry.index);
      element.setAttribute('aria-label',`在地图中查看候选 ${entry.index+1}`);
      element.setAttribute('aria-pressed',String(entry.index === candidateSelected));
      element.title = `grid_index=${entry.candidate.grid_index} · 原始概率 ${fmt(entry.candidate.prior_probability)} · 门控概率 ${fmt(entry.candidate.probability)} · 距离 ${fmt(entry.candidate.distance)} m`;
      setHTML(element,entry.html);
    });
  }
}
function selectKey(key) {
  if (!linked[key]) return;
  selected = key;
  candidatePage = 0;
  candidateSelected = null;
  const index = filteredRows().findIndex(row => row.identity.key === selected);
  if (index >= 0) targetPage = Math.floor(index/targetPageSize);
  renderTargets();
  drawMap();
  inspector();
}
function select(event) {
  const target = event.target.closest('[data-select],tr[data-key]');
  if (target) selectKey(target.dataset.select || target.dataset.key);
}
$('map').addEventListener('click',select);
$('targets').addEventListener('click',select);
$('map').addEventListener('keydown',event => {
  if (event.key === 'Enter' || event.key === ' ') { event.preventDefault(); select(event); }
});
$('layers').addEventListener('change',event => {
  const layer = event.target.dataset.layer;
  if (Object.hasOwn(enabled,layer)) { enabled[layer] = event.target.checked; drawMap(); }
});
$('flip').addEventListener('click',() => {
  flipped = !flipped;
  $('flip').setAttribute('aria-pressed',String(flipped));
  drawMap();
});
function changeZoom(delta) {
  zoom = Math.max(1,Math.min(1.8,Number((zoom+delta).toFixed(1))));
  $('zoom-in').disabled = zoom >= 1.8;
  $('zoom-out').disabled = zoom <= 1;
  drawMap();
}
$('zoom-in').addEventListener('click',() => changeZoom(.2));
$('zoom-out').addEventListener('click',() => changeZoom(-.2));
function expandMap(force) {
  const expanded = force ?? !$('field-panel').classList.contains('map-expanded');
  $('field-panel').classList.toggle('map-expanded',expanded);
  $('expand').setAttribute('aria-pressed',String(expanded));
  $('expand').setAttribute('aria-label',expanded ? '收起地图' : '展开地图');
  document.body.style.overflow = expanded ? 'hidden' : '';
}
$('expand').addEventListener('click',() => expandMap());
function notify(message) {
  $('toast').textContent = message;
  $('toast').classList.add('visible');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => $('toast').classList.remove('visible'),2400);
}
$('candidates').addEventListener('click',event => {
  const button = event.target.closest('[data-candidate]');
  if (!button) return;
  candidateSelected = Number(button.dataset.candidate);
  inspector();
  drawMap();
  notify(enabled.prior ? '候选已在地图中高亮' : '候选已选择，请开启猜点图层查看');
});
for (const prefix of ['targets','candidates']) for (const direction of ['prev','next']) {
  $(prefix+'-'+direction).addEventListener('click',() => {
    const delta = direction === 'next' ? 1 : -1;
    if (prefix === 'targets') { targetPage += delta; renderTargets(); }
    else { candidatePage += delta; inspector(); }
  });
}
function fitTargetPage() {
  const view = document.querySelector('.table-scroll'), sample = $('targets').querySelector('tr[data-key]');
  if (!sample) return;
  const rowHeight = sample.getBoundingClientRect().height;
  const available = view.clientHeight-view.querySelector('thead').getBoundingClientRect().height;
  const size = rowHeight > 0 && available > 0 ? Math.max(2,Math.min(16,Math.floor(available/rowHeight))) : 6;
  if (size !== targetPageSize) { targetPageSize = size; renderTargets(); }
}
new ResizeObserver(fitTargetPage).observe(document.querySelector('.table-scroll'));
$('search').addEventListener('input',() => { targetPage = 0; renderTargets(); });
document.querySelectorAll('[data-filter]').forEach(button => button.addEventListener('click',() => {
  filter = button.dataset.filter;
  targetPage = 0;
  document.querySelectorAll('[data-filter]').forEach(item => item.setAttribute('aria-pressed',String(item === button)));
  renderTargets();
}));
document.addEventListener('keydown',event => {
  if (event.key === 'Escape') expandMap(false);
  if (['INPUT','TEXTAREA','BUTTON','A','SUMMARY'].includes(event.target.tagName) || document.querySelector('dialog[open]')) return;
  if (event.key !== 'ArrowDown' && event.key !== 'ArrowUp') return;
  const visible = filteredRows();
  if (!visible.length) return;
  event.preventDefault();
  const index = visible.findIndex(row => row.identity.key === selected);
  const next = index < 0 ? (event.key === 'ArrowDown' ? 0 : visible.length-1) : (index+(event.key === 'ArrowDown' ? 1 : -1)+visible.length)%visible.length;
  selectKey(visible[next].identity.key);
});
document.querySelectorAll('[data-nav]').forEach(link => link.addEventListener('click',() => {
  document.querySelectorAll('[data-nav]').forEach(item => {
    item.classList.toggle('active',item === link);
    if (item === link) item.setAttribute('aria-current','location');
    else item.removeAttribute('aria-current');
  });
}));
$('health-nav').addEventListener('click',() => $('health-dialog').showModal());
$('about').addEventListener('click',() => $('about-dialog').showModal());
document.querySelectorAll('[data-close-dialog]').forEach(button => button.addEventListener('click',() => $(button.dataset.closeDialog).close()));
document.querySelectorAll('dialog').forEach(dialog => dialog.addEventListener('click',event => {
  const rect = dialog.getBoundingClientRect();
  if (event.clientX < rect.left || event.clientX > rect.right || event.clientY < rect.top || event.clientY > rect.bottom) dialog.close();
}));
function updateClock() { $('clock').textContent = new Date().toLocaleTimeString('zh-CN',{hour12:false}); }
updateClock();
setInterval(updateClock,1000);

function connection(live,label) {
  $('connection').textContent = label;
  $('connection-dot').classList.toggle('live',live);
  $('connection').parentElement.classList.toggle('connected',live);
}
function stop() { streams.forEach(stream => stream.close()); streams = []; }
function connect() {
  stop();
  if (paused || document.hidden) return;
  connection(false,'连接中');
  const radar = new EventSource('/api/radar-stream'), perf = new EventSource('/api/performance-stream');
  streams = [radar,perf];
  radar.addEventListener('radar',event => {
    state = JSON.parse(event.data);
    lastRadar = Date.now();
    connection(true,'SSE 已连接');
    render();
  });
  perf.addEventListener('performance',event => performance(JSON.parse(event.data)));
  radar.onerror = () => { connection(false,'连接中断 · 自动重连'); expire(); };
  perf.onerror = () => clearPerformance('性能流已断开 · 等待重连');
}
function expire() {
  if (!state) return;
  Object.values(state.sources).forEach(source => { if (source.status === 'LIVE') source.status = 'STALE'; });
  Object.values(state.modules).forEach(module => { if (module.status === 'LIVE') module.status = 'STALE'; });
  render();
}
function suspend(label) {
  stop();
  connection(false,label);
  expire();
  clearPerformance(label);
  lastRadar = 0;
}
$('pause').addEventListener('click',() => {
  paused = !paused;
  $('pause').setAttribute('aria-pressed',String(paused));
  $('pause').innerHTML = icon(paused ? 'play' : 'pause',true)+`<span>${paused ? '继续接收' : '暂停接收'}</span>`;
  if (paused) suspend('已暂停接收');
  else connect();
});
document.addEventListener('visibilitychange',() => {
  if (document.hidden) suspend(paused ? '已暂停接收' : '页面隐藏 · 已暂停连接');
  else if (!paused) connect();
});
window.addEventListener('pagehide',stop);
setInterval(() => {
  if (lastRadar && Date.now()-lastRadar > 4000) { connection(false,'未收到更新 · 等待重连'); expire(); }
  if (lastPerformance && Date.now()-lastPerformance > 4000) clearPerformance('未收到性能更新 · 等待重连');
},2000);
connect();
