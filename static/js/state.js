/* 全局状态、默认示例、ID 生成、localStorage 自动保存。 */
'use strict';
let uidSeq = 1;
function uid(prefix) {
  const exists = id =>
    State.pulleys.some(p => p.id === id) || State.obstacles.some(o => o.id === id);
  let id;
  do { id = prefix + (uidSeq++); } while (exists(id));
  return id;
}

function syncUidSeq() {
  const ids = [...State.pulleys, ...State.obstacles]
    .map(o => parseInt(String(o.id).replace(/\D/g, ''), 10))
    .filter(n => Number.isFinite(n));
  uidSeq = Math.max(uidSeq, 100, ...ids) + 1;
}

const DEFAULT_GLOBALS = {
  inputRpm: 1440, powerKw: 5.0, friction: 0.3, allowTension: 900,
  minWrapDeg: 120, safetyGap: 15, tensionerTravel: 60,
  stretchRate: 0.01, installAllowance: 0
};

function demoScheme() {
  uidSeq = 100;
  return {
    globals: { ...DEFAULT_GLOBALS },
    pulleys: [
      { id: 'p1', name: '电机轮', kind: 'driver', x: 0, y: 0, diameter: 160,
        locked: false, dir: 1, targetRpm: null, targetDir: null },
      { id: 'p2', name: '从动轴', kind: 'driven', x: 760, y: 60, diameter: 320,
        locked: false, dir: 1, targetRpm: 720, targetDir: 1 },
      { id: 'p3', name: '张紧轮', kind: 'tensioner', x: 240, y: -470, diameter: 120,
        locked: false, dir: 1, targetRpm: null, targetDir: null }
    ],
    obstacles: [
      { id: 'o1', name: '机架护罩', x: 380, y: 230, w: 280, h: 90, rot: 0 }
    ],
    route: { order: ['p1', 'p2', 'p3'], crossedEdges: {} }
  };
}

const State = {
  ...demoScheme(),
  tool: 'select',
  selection: { type: null, id: null },
  result: null,            // /api/analyze 返回
  quick: null,             // 离线快速路径（拖拽时）
  overlayScheme: null,     // 叠加显示的另一份方案 + 其分析结果
  compare: { a: null, b: null, result: null },
  view: { scale: 1, ox: 0, oy: 0 },
  highlight: { pulleys: [], edges: [], obstacles: [], activeWarn: null },
  dirty: false
};

const AUTOSAVE_KEY = 'belt-layout-autosave-v1';
function persist() {
  try {
    localStorage.setItem(AUTOSAVE_KEY, JSON.stringify({
      globals: State.globals, pulleys: State.pulleys,
      obstacles: State.obstacles, route: State.route
    }));
  } catch (e) { /* 隐私模式等场景忽略 */ }
}
function restoreAutosave() {
  try {
    const raw = localStorage.getItem(AUTOSAVE_KEY);
    if (!raw) return false;
    const s = JSON.parse(raw);
    if (!s.pulleys || s.pulleys.length < 2) return false;
    State.globals = { ...DEFAULT_GLOBALS, ...s.globals };
    State.pulleys = s.pulleys;
    State.obstacles = s.obstacles || [];
    State.route = s.route || { order: s.pulleys.map(p => p.id), crossedEdges: {} };
    uidSeq = 100 + Math.max(0, ...[...State.pulleys, ...State.obstacles]
      .map(o => parseInt(String(o.id).replace(/\D/g, ''), 10) || 0));
    return true;
  } catch (e) { return false; }
}

function findPulley(id) { return State.pulleys.find(p => p.id === id); }
function findObstacle(id) { return State.obstacles.find(o => o.id === id); }
function selectObj(type, id) {
  State.selection = { type, id };
  if (type === 'pulley') {
    const i = State.route.order.indexOf(id);
    State.route._sel = i >= 0 ? i : null;
  }
}
function clearHighlight() {
  State.highlight = { pulleys: [], edges: [], obstacles: [], activeWarn: null };
}
