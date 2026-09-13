/* 多级回路与共享轴的增删、归并操作。 */
'use strict';
const Loops = (() => {
  function addLoop() {
    const id = uid('L');
    const seq = State.loops.length + 1;
    State.loops.push({
      id, name: '回路 ' + seq, order: [], crossedEdges: {},
      isInput: false,
      powerKw: State.globals.powerKw, efficiency: State.globals.efficiency,
      inputRpm: State.globals.inputRpm,
      friction: State.globals.friction, allowTension: State.globals.allowTension,
      minWrapDeg: State.globals.minWrapDeg,
      tensionerTravel: State.globals.tensionerTravel,
      stretchRate: State.globals.stretchRate
    });
    State.activeLoopId = id;
    persist();
    App.refreshAll();
    App.flash('已新建回路 ' + seq + '：放置带轮或用绕行面板加入带轮');
  }

  function removeLoop(lid) {
    if (State.loops.length <= 1) return;
    State.loops = State.loops.filter(l => l.id !== lid);
    // 轮保留但不再属于任何回路（后端给 PULLEY_UNROUTED 提示）
    if (State.activeLoopId === lid)
      State.activeLoopId = State.loops[0].id;
    persist();
    App.refreshAll();
  }

  function addToLoop(lid, pid) {
    const lp = State.loops.find(l => l.id === lid);
    const p = findPulley(pid);
    if (!lp || !p || lp.order.includes(pid)) return;
    if (pulleyLoop(pid)) return;
    lp.order.push(pid);
    persist();
    App.refreshAll();
  }

  function removeFromLoop(lid, pid) {
    const lp = State.loops.find(l => l.id === lid);
    if (!lp) return;
    lp.order = lp.order.filter(x => x !== pid);
    Object.keys(lp.crossedEdges).forEach(k => {
      if (k.startsWith(pid + '->') || k.endsWith('->' + pid)) delete lp.crossedEdges[k];
    });
    persist();
    App.refreshAll();
  }

  function newShaftFor(p, name) {
    const s = { id: uid('s'), name: name || (p.name + ' 轴'),
      x: p.x || 0, y: p.y || 0, locked: false,
      allowTorque: 0, allowRadial: 0 };
    State.shafts.push(s);
    return s;
  }

  // 属性面板里切换所属轴下拉 / 归并到另一根轴
  function movePulleyToShaft(pid, newSid) {
    const p = findPulley(pid);
    const s = findShaft(newSid);
    if (!p || !s || p.shaftId === newSid) return;
    const oldSid = p.shaftId;
    p.shaftId = newSid;
    // 移动后坐标跟随新轴
    p.x = s.x; p.y = s.y; p.locked = s.locked;
    // 原轴上已无带轮则删除空轴
    if (!State.pulleys.some(q => q.shaftId === oldSid))
      State.shafts = State.shafts.filter(x => x.id !== oldSid);
    persist();
  }

  function splitPulleyShaft(pid) {
    const p = findPulley(pid);
    const old = findShaft(p.shaftId);
    // 同组轮不止一个：拆一根新轴，位置取当前轴位
    const s = newShaftFor(p);
    if (old) { s.x = old.x; s.y = old.y; }
    p.shaftId = s.id;
    persist();
    App.flash('已把 ' + p.name + ' 拆到「' + s.name + '」');
  }

  function mergeShaft(fromSid, toSid) {
    const s = findShaft(toSid);
    if (!s) return;
    State.pulleys.forEach(p => {
      if (p.shaftId === fromSid) {
        p.shaftId = toSid;
        p.x = s.x; p.y = s.y; p.locked = s.locked;
      }
    });
    // 删除空轴
    if (!State.shafts.some(x => x.id === fromSid && shaftPulleys(fromSid).length)) {
      State.shafts = State.shafts.filter(x => x.id !== fromSid);
    }
    persist();
    App.flash('已归并到共享轴「' + s.name + '」，带轮将整体移动');
  }

  function changeKind(pid, kind) {
    const p = findPulley(pid);
    if (!p) return;
    if (kind === 'driver') {
      const lp = pulleyLoop(pid);
      // 同回路内只允许一个主动轮
      State.pulleys.forEach(q => {
        if (q.kind === 'driver' && pulleyLoop(q.id)?.id === lp?.id) q.kind = 'driven';
      });
    }
    p.kind = kind;
    if (kind === 'driver' && !('dir' in p)) p.dir = 1;
    persist();
    App.refreshAll();
  }

  // 拖动一整根共享轴
  function moveShaft(sid, x, y) {
    const s = findShaft(sid);
    if (!s || s.locked) return;
    s.x = x; s.y = y;
    shaftPulleys(sid).forEach(p => { p.x = x; p.y = y; });
  }

  return { addLoop, removeLoop, addToLoop, removeFromLoop, newShaftFor,
    movePulleyToShaft, splitPulleyShaft, mergeShaft, changeKind, moveShaft };
})();
