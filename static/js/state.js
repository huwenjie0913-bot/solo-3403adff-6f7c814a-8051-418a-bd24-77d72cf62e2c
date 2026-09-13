/* 全局状态、默认示例、ID 生成、localStorage 自动保存。 */
'use strict';
let uidSeq = 1;
function uid(prefix) {
  const exists = id =>
    State.pulleys.some(p => p.id === id) ||
    State.obstacles.some(o => o.id === id) ||
    State.shafts.some(s => s.id === id) ||
    State.loops.some(l => l.id === id);
  let id;
  do { id = prefix + (uidSeq++); } while (exists(id));
  return id;
}

function syncUidSeq() {
  const ids = [...State.pulleys, ...State.obstacles, ...State.shafts]
    .map(o => parseInt(String(o.id).replace(/\D/g, ''), 10))
    .filter(n => Number.isFinite(n));
  uidSeq = Math.max(uidSeq, 100, ...ids) + 1;
}

const DEFAULT_GLOBALS = {
  inputRpm: 1440, powerKw: 5.0, friction: 0.3, allowTension: 900,
  minWrapDeg: 120, safetyGap: 15, tensionerTravel: 60,
  stretchRate: 0.01, installAllowance: 0, efficiency: 0.96
};

// 与后端 belt/analyzer.py LOOP_PALETTE 保持一致
const LOOP_COLORS = ['#d8b15a', '#5b9bd5', '#58c08a', '#e0845a',
  '#b48cd4', '#5ac0c0', '#d96c8b', '#9bb05c'];
function loopColor(idOrIndex) {
  const i = State.loops.findIndex(l => l.id === idOrIndex);
  const idx = i >= 0 ? i : (Number.isInteger(idOrIndex) ? idOrIndex : 0);
  return LOOP_COLORS[idx % LOOP_COLORS.length];
}

function demoScheme() {
  uidSeq = 100;
  return {
    globals: { ...DEFAULT_GLOBALS },
    shafts: [
      { id: 's1', name: '电机轴', x: 0, y: 0, locked: false,
        allowTorque: 60, allowRadial: 4000 },
      { id: 's2', name: '中间轴', x: 680, y: 0, locked: false,
        allowTorque: 130, allowRadial: 4000 },
      { id: 's3', name: '工作轴', x: 1300, y: 300, locked: false,
        allowTorque: 400, allowRadial: 4000 }
    ],
    pulleys: [
      { id: 'p1', name: '电机轮', kind: 'driver', shaftId: 's1',
        diameter: 160, locked: false, dir: 1, targetRpm: null, targetDir: null },
      { id: 'p2', name: '中间大带轮', kind: 'driven', shaftId: 's2',
        diameter: 320, locked: false, dir: 1, targetRpm: null, targetDir: null },
      { id: 'p3', name: '中间小带轮', kind: 'driver', shaftId: 's2',
        diameter: 140, locked: false, dir: 1, targetRpm: null, targetDir: null },
      { id: 'p4', name: '工作轴轮', kind: 'driven', shaftId: 's3',
        diameter: 280, locked: false, dir: 1, targetRpm: 360, targetDir: 1 }
    ],
    obstacles: [],
    loops: [
      { id: 'L1', name: '回路 1', order: ['p1', 'p2'], crossedEdges: {},
        isInput: true, powerKw: 5.0, efficiency: 0.96, allowTension: 1000 },
      { id: 'L2', name: '回路 2', order: ['p3', 'p4'], crossedEdges: {},
        isInput: false, powerKw: 4.8, efficiency: 0.96, allowTension: 2000 }
    ],
    route: { order: ['p1', 'p2'], crossedEdges: {} }
  };
}

const State = {
  ...demoScheme(),
  tool: 'select',
  selection: { type: null, id: null },
  activeLoopId: 'L1',     // 当前正在编辑绕行顺序的回路
  result: null,           // /api/analyze 返回（多级模式）
  quick: null,            // 离线快速路径（拖拽时）
  overlayScheme: null,    // 叠加显示的另一份方案 + 其分析结果
  compare: { a: null, b: null, result: null },
  view: { scale: 1, ox: 0, oy: 0 },
  highlight: { pulleys: [], edges: [], obstacles: [], shafts: [],
    loops: [], lrefs: [], activeWarn: null },
  dirty: false
};

const AUTOSAVE_KEY = 'belt-layout-autosave-v2';
function persist() {
  try {
    localStorage.setItem(AUTOSAVE_KEY, JSON.stringify({
      version: 2, globals: State.globals, pulleys: State.pulleys,
      obstacles: State.obstacles, shafts: State.shafts, loops: State.loops,
      route: State.route, activeLoopId: State.activeLoopId
    }));
  } catch (e) { /* 隐私模式等场景忽略 */ }
}

function migrateV1(s) {
  // 旧版单回路方案：每个轮建一根独立轴，全部轮归入一条回路
  const shafts = [];
  const loops = [{
    id: 'L1', name: '回路 1', order: (s.route && s.route.order) ||
      s.pulleys.map(p => p.id),
    crossedEdges: (s.route && s.route.crossedEdges) || {},
    isInput: true,
    powerKw: s.globals?.powerKw ?? DEFAULT_GLOBALS.powerKw,
    efficiency: s.globals?.efficiency ?? DEFAULT_GLOBALS.efficiency
  }];
  s.pulleys.forEach((p, i) => {
    const sid = 's' + (i + 1);
    shafts.push({ id: sid, name: p.name + ' 轴', x: p.x || 0, y: p.y || 0,
      locked: !!p.locked, allowTorque: 0, allowRadial: 0 });
    p.shaftId = sid;
  });
  return { shafts, loops };
}

function restoreAutosave() {
  try {
    let raw = localStorage.getItem(AUTOSAVE_KEY);
    if (!raw) {
      // 旧版单回路自动保存：迁移后一次性接管
      raw = localStorage.getItem('belt-layout-autosave-v1');
      if (raw) {
        const old = JSON.parse(raw);
        const m = migrateV1(old);
        raw = JSON.stringify({
          version: 2, globals: old.globals, pulleys: old.pulleys,
          obstacles: old.obstacles, shafts: m.shafts, loops: m.loops,
          route: { order: m.loops[0].order.slice(), crossedEdges: {} }
        });
      }
    }
    if (!raw) return false;
    const s = JSON.parse(raw);
    if (!s.pulleys) return false;
    return applyData(s, false);
  } catch (e) { return false; }
}

function applyData(s, replace) {
  State.globals = { ...DEFAULT_GLOBALS, ...(s.globals || {}) };
  State.obstacles = s.obstacles || [];
  if (Array.isArray(s.loops) && s.loops.length) {
    State.pulleys = s.pulleys;
    State.shafts = s.shafts || [];
    State.loops = s.loops;
    State.route = s.route || { order: s.loops[0].order.slice(), crossedEdges: {} };
  } else {
    // 旧格式：迁移
    State.pulleys = s.pulleys;
    const m = migrateV1(s);
    State.shafts = m.shafts;
    State.loops = m.loops;
    State.route = { order: m.loops[0].order.slice(), crossedEdges: {} };
  }
  // 兜底：缺少轴定义的轮补轴
  const smap = Object.fromEntries(State.shafts.map(x => [x.id, x]));
  let n = State.shafts.length;
  State.pulleys.forEach(p => {
    if (!smap[p.shaftId]) {
      n += 1;
      const sid = 's' + n;
      smap[sid] = { id: sid, name: p.name + ' 轴', x: p.x || 0, y: p.y || 0,
        locked: !!p.locked, allowTorque: 0, allowRadial: 0 };
      State.shafts.push(smap[sid]);
      p.shaftId = sid;
    }
    if (smap[p.shaftId]) {
      p.x = smap[p.shaftId].x; p.y = smap[p.shaftId].y;
      p.locked = smap[p.shaftId].locked;
    }
  });
  State.activeLoopId = (replace === false && s.activeLoopId &&
    State.loops.some(l => l.id === s.activeLoopId)) ? s.activeLoopId
    : (State.loops[0]?.id ?? null);
  State.selection = { type: null, id: null };
  State.overlayScheme = null;
  clearHighlight();
  syncUidSeq();
  persist();
  return true;
}

// ---------- 查询辅助 ----------
function findPulley(id) { return State.pulleys.find(p => p.id === id); }
function findObstacle(id) { return State.obstacles.find(o => o.id === id); }
function findShaft(id) { return State.shafts.find(s => s.id === id); }
function activeLoop() {
  return State.loops.find(l => l.id === State.activeLoopId) || State.loops[0] || null;
}
function pulleyLoop(id) {
  return State.loops.find(l => l.order.includes(id)) || null;
}
function shaftPulleys(sid) { return State.pulleys.filter(p => p.shaftId === sid); }
function shaftLoops(sid) {
  const ids = new Set(shaftPulleys(sid).map(p => pulleyLoop(p.id)?.id).filter(Boolean));
  return [...ids];
}

function selectObj(type, id) {
  State.selection = { type, id };
  if (type === 'pulley') {
    const lp = pulleyLoop(id);
    if (lp) State.activeLoopId = lp.id;
    const i = lp ? lp.order.indexOf(id) : -1;
    if (lp) lp._sel = i >= 0 ? i : null;
  }
}
function clearHighlight() {
  State.highlight = { pulleys: [], edges: [], obstacles: [], shafts: [],
    loops: [], lrefs: [], activeWarn: null };
}

// 结果侧查询
function resultPulley(id) {
  return (State.result?.pulleys || []).find(p => p.id === id);
}
function resultShaft(id) {
  return (State.result?.shafts || []).find(s => s.id === id);
}
function resultLoop(id) {
  return (State.result?.loops || []).find(l => l.id === id);
}
