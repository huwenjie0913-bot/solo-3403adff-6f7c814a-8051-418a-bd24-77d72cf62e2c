/* 鼠标交互：工具放置、拖拽移动、视图平移缩放、键盘删除。 */
'use strict';
const Drag = {
  mode: null,        // 'move' | 'pan' | 'place'
  id: null, type: null,
  start: null, grabOffset: null,
  ghost: null, placeKind: null
};

const Interact = (() => {
  const canvas = Render.canvas;
  let moved = false;

  function eventPos(e) {
    const r = canvas.getBoundingClientRect();
    return [e.clientX - r.left, e.clientY - r.top];
  }

  function down(e) {
    const [sx, sy] = eventPos(e);
    moved = false;
    if (e.button === 1 || e.button === 2 || (e.button === 0 && e.altKey)) {
      Drag.mode = 'pan'; Drag.start = [sx, sy, State.view.ox, State.view.oy];
      e.preventDefault();
      return;
    }
    if (e.button !== 0) return;
    const tool = State.tool;
    if (tool !== 'select') {
      const w = Render.s2w(sx, sy);
      if (tool === 'obstacle') {
        // 按下拖拽定义矩形宽高（最小 40mm），单击放置默认 200×120
        Drag.mode = 'place'; Drag.placeKind = 'obstacle';
        Drag.start = [w[0], w[1]];
        Drag.ghost = { tool, x: w[0], y: w[1], w: 0, h: 0, ax: w[0], ay: w[1] };
      } else {
        Drag.mode = 'place'; Drag.placeKind = tool;
        Drag.start = [w[0], w[1]];
        Drag.ghost = { tool, x: w[0], y: w[1],
          d: tool === 'driven' ? 240 : 140, ax: w[0], ay: w[1] };
      }
      return;
    }
    const hit = Render.pickAt(sx, sy);
    if (hit && hit.type === 'pulley') {
      const p = findPulley(hit.id);
      selectObj('pulley', hit.id);
      if (p.locked) { Drag.mode = null; App.refreshAll(); return; }
      Drag.mode = 'move'; Drag.id = hit.id; Drag.type = 'pulley';
      Drag.start = [p.x, p.y];
      Drag.grabOffset = [p.x - Render.s2w(sx, sy)[0], p.y - Render.s2w(sx, sy)[1]];
    } else if (hit && hit.type === 'obstacle') {
      selectObj('obstacle', hit.id);
      Drag.mode = 'move'; Drag.id = hit.id; Drag.type = 'obstacle';
      const w = Render.s2w(sx, sy);
      const o = findObstacle(hit.id);
      Drag.grabOffset = [o.x - w[0], o.y - w[1]];
    } else if (hit && hit.type === 'edge') {
      State.selection = { type: 'edge', index: hit.index };
      // 点击带段：优先显示与之相关的告警依据
      const w = App.warnsForEdge(hit.index)[0];
      if (w) App.activateWarn(w, true);
      else { clearHighlight(); State.highlight.edges = [hit.index]; }
    } else {
      selectObj(null, null);
      State.selection = { type: null, id: null };
      clearHighlight();
    }
    App.refreshAll();
  }

  function move(e) {
    const [sx, sy] = eventPos(e);
    const w = Render.s2w(sx, sy);
    if (Drag.mode === 'pan') {
      State.view.ox = Drag.start[2] + (sx - Drag.start[0]);
      State.view.oy = Drag.start[3] + (sy - Drag.start[1]);
      Render.render();
      return;
    }
    if (Drag.mode === 'place' && Drag.ghost) {
      moved = true;
      if (Drag.placeKind === 'obstacle') {
        const ax = Drag.ghost.ax, ay = Drag.ghost.ay;
        Drag.ghost.x = (ax + w[0]) / 2;
        Drag.ghost.y = (ay + w[1]) / 2;
        Drag.ghost.w = Math.max(40, Math.abs(w[0] - ax));
        Drag.ghost.h = Math.max(40, Math.abs(w[1] - ay));
      } else {
        Drag.ghost.x = Drag.ghost.ax; Drag.ghost.y = Drag.ghost.ay;
        Drag.ghost.d = Math.max(60, 2 * Math.hypot(w[0] - Drag.ghost.ax,
          w[1] - Drag.ghost.ay));
      }
      Render.render();
      return;
    }
    if (Drag.mode === 'move') {
      moved = true;
      if (Drag.type === 'pulley') {
        const p = findPulley(Drag.id);
        p.x = w[0] + Drag.grabOffset[0];
        p.y = w[1] + Drag.grabOffset[1];
      } else {
        const o = findObstacle(Drag.id);
        o.x = w[0] + Drag.grabOffset[0];
        o.y = w[1] + Drag.grabOffset[1];
      }
      State.quick = GEO.quickPath(State);
      // 拖拽中用后端节流重算 + 本地即时预览
      App.invalidate(true);
      Render.render();
      Props.renderProps();
    } else if (State.tool === 'select') {
      canvas.style.cursor = Render.pickAt(sx, sy) ? 'pointer' : 'default';
    }
  }

  function up(e) {
    if (Drag.mode === 'place' && Drag.ghost) {
      commitPlace(Drag.ghost);
    }
    if (Drag.mode === 'move' && moved) {
      State.dirty = true; persist();
      App.invalidate();
    }
    State.quick = null;
    Drag.mode = null; Drag.ghost = null;
    App.refreshAll();
  }

  function commitPlace(g) {
    if (g.tool === 'obstacle') {
      const o = { id: uid('o'), name: '护罩' + (State.obstacles.length + 1),
        x: Math.round(g.x), y: Math.round(g.y),
        w: Math.round(g.w) || 200, h: Math.round(g.h) || 120, rot: 0 };
      State.obstacles.push(o);
      selectObj('obstacle', o.id);
    } else {
      const kind = g.tool;
      // 新主动轮自动把旧的降级
      if (kind === 'driver')
        State.pulleys.forEach(p => { if (p.kind === 'driver') p.kind = 'driven'; });
      const seq = State.pulleys.filter(p => p.kind === kind).length + 1;
      const p = {
        id: uid('p'),
        name: { driver: '主动轮', driven: '从动轮', idler: '惰轮', tensioner: '张紧轮' }[kind] + seq,
        kind, x: Math.round(g.x), y: Math.round(g.y), diameter: g.d,
        locked: false, dir: 1, targetRpm: null, targetDir: null
      };
      if (kind === 'driven') { p.targetRpm = null; p.targetDir = 1; }
      State.pulleys.push(p);
      State.route.order.push(p.id);
      selectObj('pulley', p.id);
    }
    setTool('select');
    State.dirty = true; persist();
    App.refreshAll();
  }

  function wheel(e) {
    e.preventDefault();
    const [sx, sy] = eventPos(e);
    const factor = e.deltaY < 0 ? 1.12 : 1 / 1.12;
    const w0 = Render.s2w(sx, sy);
    State.view.scale = Math.max(0.02, Math.min(6, State.view.scale * factor));
    const w1 = Render.s2w(sx, sy);
    State.view.ox += (w1[0] - w0[0]) * State.view.scale;
    State.view.oy += (w1[1] - w0[1]) * State.view.scale;
    Render.render();
  }

  function key(e) {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
    if (e.key === 'Delete' || e.key === 'Backspace') {
      if (State.selection.type === 'pulley' || State.selection.type === 'obstacle')
        App.deleteSelection();
    }
    if (e.key === 'f' || e.key === 'F') { Render.fit(); Render.render(); }
  }

  function setTool(t) {
    State.tool = t;
    document.querySelectorAll('.tool-btn[data-tool]').forEach(b =>
      b.classList.toggle('active', b.dataset.tool === t));
    canvas.style.cursor = t === 'select' ? 'default' : 'crosshair';
  }

  function init() {
    canvas.addEventListener('mousedown', down);
    window.addEventListener('mousemove', move);
    window.addEventListener('mouseup', up);
    canvas.addEventListener('wheel', wheel, { passive: false });
    canvas.addEventListener('contextmenu', e => e.preventDefault());
    window.addEventListener('keydown', key);
    document.querySelectorAll('.tool-btn[data-tool]').forEach(b =>
      b.addEventListener('click', () => setTool(b.dataset.tool)));
  }

  return { init, setTool };
})();
