/* Canvas 绘制：世界坐标(mm, y向下) -> 屏幕；命中检测；高亮联动。 */
'use strict';
const Render = (() => {
  const canvas = document.getElementById('canvas');
  const ctx = canvas.getContext('2d');
  const $ = id => document.getElementById(id);
  let DPR = 1;

  function resize() {
    const r = canvas.parentElement.getBoundingClientRect();
    DPR = window.devicePixelRatio || 1;
    canvas.width = r.width * DPR;
    canvas.height = r.height * DPR;
    canvas.style.width = r.width + 'px';
    canvas.style.height = r.height + 'px';
  }
  function viewW() { return canvas.width / DPR; }
  function viewH() { return canvas.height / DPR; }

  const w2sX = x => x * State.view.scale + State.view.ox;
  const w2sY = y => y * State.view.scale + State.view.oy;
  const w2s = p => [w2sX(p[0]), w2sY(p[1])];
  const s2wX = x => (x - State.view.ox) / State.view.scale;
  const s2wY = y => (y - State.view.oy) / State.view.scale;

  function fit(padding) {
    padding = padding ?? 120;
    const pts = [];
    State.pulleys.forEach(p => {
      const r = p.diameter / 2;
      pts.push([p.x - r, p.y - r], [p.x + r, p.y + r]);
    });
    State.obstacles.forEach(o => GEO.obbCorners(o).forEach(c => pts.push(c)));
    if (!pts.length) { pts.push([-500, -500], [500, 500]); }
    const x0 = Math.min(...pts.map(p => p[0])) - padding;
    const x1 = Math.max(...pts.map(p => p[0])) + padding;
    const y0 = Math.min(...pts.map(p => p[1])) - padding;
    const y1 = Math.max(...pts.map(p => p[1])) + padding;
    const sc = Math.min(viewW() / (x1 - x0), viewH() / (y1 - y0));
    State.view.scale = Math.max(0.05, sc);
    State.view.ox = (viewW() - (x0 + x1) * State.view.scale) / 2;
    State.view.oy = (viewH() - (y0 + y1) * State.view.scale) / 2;
  }

  // ---------- 命中检测（世界坐标） ----------
  function hitPulley(w) {
    const p = State.pulleys.find(q => GEO.dist(w, [q.x, q.y]) <= q.diameter / 2 + 6);
    return p ? p.id : null;
  }
  function hitObstacle(w) {
    return State.obstacles.some(o => {
      if (GEO.pointInObb(w, o)) return true;
      const cs = GEO.obbCorners(o);
      for (let i = 0; i < 4; i++)
        if (GEO.pointSegDist(w, cs[i], cs[(i + 1) % 4]) < 6) return true;
      return false;
    });
  }
  function hitEdge(w) {
    const res = State.result || null;
    const edges = res?.edges;
    if (!edges) return null;
    let best = null, bd = 8;
    edges.forEach((e, i) => {
      if (!e.feasible) return;
      const d = GEO.pointSegDist(w, e.p1, e.p2);
      if (d < bd) { bd = d; best = i; }
    });
    return best;
  }
  function pickAt(sx, sy) {
    const w = [s2wX(sx), s2wY(sy)];
    const ei = hitEdge(w);
    if (ei !== null) return { type: 'edge', index: ei };
    for (const p of [...State.pulleys].reverse())
      if (hitPulley.call(null, w) && GEO.dist(w, [p.x, p.y]) <= p.diameter / 2 + 6)
        return { type: 'pulley', id: p.id };
    for (const o of [...State.obstacles].reverse()) {
      const cs = GEO.obbCorners(o);
      let near = GEO.pointInObb(w, o);
      if (!near) for (let i = 0; i < 4; i++)
        if (GEO.pointSegDist(w, cs[i], cs[(i + 1) % 4]) < 6) { near = true; break; }
      if (near) return { type: 'obstacle', id: o.id };
    }
    return null;
  }

  // ---------- 绘制 ----------
  const COLORS = {
    driver: '#5b9bd5', driven: '#58c08a', idler: '#b48cd4',
    tensioner: '#e0b65a', obstacle: '#d98c8c', belt: '#d8b15a',
    beltDim: '#7c6a3c', grid: '#262c33', axis: '#333c46'
  };

  function drawGrid() {
    const sc = State.view.scale;
    const stepMm = [20, 50, 100, 200, 500, 1000, 2000].find(s => s * sc > 28) || 2000;
    const gx0 = Math.floor(s2wX(0) / stepMm) * stepMm;
    const gx1 = Math.ceil(s2wX(viewW()) / stepMm) * stepMm;
    const gy0 = Math.floor(s2wY(0) / stepMm) * stepMm;
    const gy1 = Math.ceil(s2wY(viewH()) / stepMm) * stepMm;
    ctx.strokeStyle = COLORS.grid; ctx.lineWidth = 1;
    ctx.beginPath();
    for (let x = gx0; x <= gx1; x += stepMm) {
      ctx.moveTo(w2sX(x), 0); ctx.lineTo(w2sX(x), viewH());
    }
    for (let y = gy0; y <= gy1; y += stepMm) {
      ctx.moveTo(0, w2sY(y)); ctx.lineTo(viewW(), w2sY(y));
    }
    ctx.stroke();
    // 坐标轴
    ctx.strokeStyle = COLORS.axis; ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(0, w2sY(0)); ctx.lineTo(viewW(), w2sY(0));
    ctx.moveTo(w2sX(0), 0); ctx.lineTo(w2sX(0), viewH());
    ctx.stroke();
  }

  function drawOverlayScheme() {
    const ov = State.overlayScheme;
    if (!ov) return;
    const res = ov.result;
    ctx.save();
    ctx.globalAlpha = 0.55;
    // 叠加方案的轮
    for (const p of ov.scheme.pulleys || []) {
      const c = w2s([p.x, p.y]), r = p.diameter / 2 * State.view.scale;
      ctx.strokeStyle = '#7f93a8'; ctx.setLineDash([5, 4]); ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.arc(c[0], c[1], r, 0, Math.PI * 2); ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = '#7f93a8'; ctx.font = '10px sans-serif';
      ctx.fillText('(对比)' + p.name, c[0] - r, c[1] - r - 2);
    }
    drawBelt(res, '#7f93a8', true);
    ctx.restore();
  }

  function drawObstacles() {
    const hl = State.highlight.obstacles || [];
    for (const o of State.obstacles) {
      const cs = GEO.obbCorners(o).map(w2s);
      const active = State.selection.type === 'obstacle' && State.selection.id === o.id;
      const lit = hl.includes(o.id);
      ctx.save();
      ctx.beginPath();
      cs.forEach((c, i) => i ? ctx.lineTo(c[0], c[1]) : ctx.moveTo(c[0], c[1]));
      ctx.closePath();
      ctx.fillStyle = lit ? 'rgba(239,95,95,.28)' : 'rgba(217,140,140,.10)';
      ctx.fill();
      ctx.strokeStyle = lit ? '#ef5f5f' : (active ? '#fff' : COLORS.obstacle);
      ctx.lineWidth = active || lit ? 2.2 : 1.4;
      if (active) ctx.setLineDash([6, 3]);
      ctx.stroke();
      ctx.setLineDash([]);
      const c = w2s([o.x, o.y]);
      ctx.fillStyle = '#e8b0b0'; ctx.font = '11px sans-serif'; ctx.textAlign = 'center';
      ctx.fillText(o.name || o.id, c[0], c[1]);
      // 间隙标注
      const gap = State.result?.obstacleGaps?.[o.id];
      if (gap != null) {
        ctx.fillStyle = gap < State.globals.safetyGap ? '#ef5f5f' : '#8fa5b8';
        ctx.font = '10px Consolas, monospace';
        ctx.fillText('间隙 ' + gap.toFixed(1), c[0], c[1] + 13);
      }
      ctx.textAlign = 'left';
      ctx.restore();
    }
  }

  function arcPath(wp) {
    const P = findPulley(wp.id);
    // 结果可能比当前状态旧（轮刚删除）：跳过已不存在的轮弧
    if (!P) return false;
    const c = w2s([P.x, P.y]);
    const r = wp.radius * State.view.scale;
    const a0 = wp.arcA0, sweep = wp.arcDir * wp.arcSweep;
    const n = Math.max(10, Math.round(Math.abs(wp.arcSweep) * 180 / Math.PI / 3));
    ctx.beginPath();
    for (let i = 0; i <= n; i++) {
      const a = a0 + sweep * i / n;
      const x = c[0] + r * Math.cos(a), y = c[1] + r * Math.sin(a);
      i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
    }
    return true;
  }

  function drawBelt(res, color, dashed) {
    if (!res?.edges) return;
    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = 5;
    ctx.lineJoin = 'round'; ctx.lineCap = 'round';
    if (dashed) ctx.setLineDash([7, 5]);
    res.edges.forEach((e, i) => {
      const hl = State.highlight.edges || [];
      if (!e.feasible) {
        ctx.save(); ctx.globalAlpha = 0.35; ctx.setLineDash([3, 6]);
        ctx.beginPath();
        const a0 = w2s(e.p1), b0 = w2s(e.p2);
        ctx.moveTo(a0[0], a0[1]); ctx.lineTo(b0[0], b0[1]); ctx.stroke();
        ctx.restore();
        return;
      }
      ctx.beginPath();
      const a = w2s(e.p1), b = w2s(e.p2);
      ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]);
      ctx.stroke();
      if (hl.includes(i)) {
        ctx.save(); ctx.strokeStyle = '#ff6b6b'; ctx.lineWidth = 9;
        ctx.globalAlpha = 0.5; ctx.stroke(); ctx.restore();
      }
      // 交叉带的交叉点标记
      if (e.crossed) {
        const m = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
        ctx.strokeStyle = color; ctx.lineWidth = 2;
        ctx.beginPath(); ctx.arc(m[0], m[1], 6, 0, Math.PI * 2); ctx.stroke();
      }
    });
    // 弧段（结果可能比状态旧，缺失轮返回 false 时跳过）
    (res.pulleys || []).forEach(wp => {
      if (!arcPath(wp)) return;
      const lit = (State.highlight.pulleys || []).includes(wp.id);
      ctx.save();
      ctx.strokeStyle = lit ? '#ff6b6b' : color;
      ctx.lineWidth = lit ? 8 : 5;
      if (lit) ctx.globalAlpha = 0.85;
      ctx.stroke();
      ctx.restore();
    });
    ctx.restore();
  }

  function drawPulleys() {
    const res = State.result;
    const wpMap = Object.fromEntries((res?.pulleys || []).map(p => [p.id, p]));
    const hl = State.highlight.pulleys || [];
    const minWrap = State.globals.minWrapDeg;
    for (const p of State.pulleys) {
      const c = w2s([p.x, p.y]);
      const r = p.diameter / 2 * State.view.scale;
      const active = State.selection.type === 'pulley' && State.selection.id === p.id;
      const lit = hl.includes(p.id);
      const col = COLORS[p.kind] || '#999';
      // 轮体
      ctx.beginPath(); ctx.arc(c[0], c[1], r, 0, Math.PI * 2);
      ctx.fillStyle = 'rgba(255,255,255,.03)'; ctx.fill();
      ctx.strokeStyle = lit ? '#ff6b6b' : col;
      ctx.lineWidth = active ? 2.6 : 2;
      ctx.stroke();
      // 轮毂与键槽
      ctx.beginPath(); ctx.arc(c[0], c[1], Math.max(2.5, r * 0.12), 0, Math.PI * 2);
      ctx.fillStyle = col; ctx.fill();
      // 转向箭头
      const wp = wpMap[p.id];
      const dir = p.kind === 'driver' ? (p.dir || 1) : (wp ? wp.dir : 1);
      drawRotationArrow(c, r, dir, col);
      // 锁定标记
      if (p.locked) {
        ctx.fillStyle = '#cfd8e3'; ctx.font = '12px sans-serif';
        ctx.fillText('🔒', c[0] + r - 4, c[1] - r + 12);
      }
      // 标签
      ctx.font = '12px sans-serif'; ctx.textAlign = 'center';
      ctx.fillStyle = '#e8eef5';
      ctx.fillText(p.name, c[0], c[1] - r - 16);
      if (wp) {
        ctx.font = '10px Consolas, monospace';
        ctx.fillStyle = wp.wrapDeg < minWrap ? '#ff8a8a' : '#9fb4c7';
        ctx.fillText(`⌀${p.diameter}  ${Math.abs(Math.round(wp.rpm))} rpm  θ${wp.wrapDeg.toFixed(0)}°`,
          c[0], c[1] + r + 14);
        ctx.fillStyle = '#7d8fa1';
        ctx.fillText(`${(wp.dir > 0 ? 'CW' : 'CCW')}`, c[0] + r + 10, c[1] + 4);
      }
      ctx.textAlign = 'left';
    }
  }

  function drawRotationArrow(c, r, dir, color) {
    if (r < 12) return;
    const rr = r * 0.55;
    const a = -Math.PI / 2;
    ctx.save();
    ctx.strokeStyle = color; ctx.lineWidth = 1.6;
    ctx.beginPath(); ctx.arc(c[0], c[1], rr, a - 0.9, a + 0.9); ctx.stroke();
    const tipA = a + dir * 0.9;
    const t = [c[0] + rr * Math.cos(tipA), c[1] + rr * Math.sin(tipA)];
    const va = dir > 0 ? tipA + Math.PI / 2 : tipA - Math.PI / 2;
    ctx.beginPath();
    ctx.moveTo(t[0], t[1]);
    ctx.lineTo(t[0] - 7 * Math.cos(va - 0.4), t[1] - 7 * Math.sin(va - 0.4));
    ctx.lineTo(t[0] - 7 * Math.cos(va + 0.4), t[1] - 7 * Math.sin(va + 0.4));
    ctx.closePath(); ctx.fillStyle = color; ctx.fill();
    ctx.restore();
  }

  function drawQuickPath() {
    const q = State.quick;
    if (!q || !Drag.mode) return;
    ctx.save();
    ctx.strokeStyle = '#8a7540'; ctx.lineWidth = 4;
    ctx.setLineDash([8, 6]); ctx.globalAlpha = 0.9;
    ctx.beginPath();
    let started = false;
    q.order.forEach((id, i) => {
      const e = q.edges[i];
      const a = w2s(e.p1), b = w2s(e.p2);
      if (!started) { ctx.moveTo(a[0], a[1]); started = true; }
      ctx.lineTo(b[0], b[1]);
      const arc = q.arcs[e.to];
      const n = Math.max(8, Math.round(Math.abs(arc.sweep) * 180 / Math.PI / 4));
      for (let k = 1; k <= n; k++) {
        const ang = arc.a0 + arc.dir * arc.sweep * k / n;
        const P = findPulley(e.to);
        const x = P.x + arc.r * Math.cos(ang), y = P.y + arc.r * Math.sin(ang);
        const s2 = w2s([x, y]);
        ctx.lineTo(s2[0], s2[1]);
      }
    });
    ctx.stroke();
    ctx.restore();
  }

  function drawDimensions() {
    const res = State.result;
    if (!res) return;
    ctx.save();
    ctx.fillStyle = '#6fa8dc'; ctx.strokeStyle = '#6fa8dc';
    ctx.font = '10px Consolas, monospace';
    // 两轮：中心距
    if (res.centerDistance != null && res.pulleys.length === 2) {
      const [a, b] = res.pulleys;
      const A = w2s([a.x, a.y]), B = w2s([b.x, b.y]);
      const u = GEO.norm([B[0] - A[0], B[1] - A[1]]);
      const off = 42;
      const n = [-u[1], u[0]];
      const p1 = [A[0] + n[0] * off, A[1] + n[1] * off];
      const p2 = [B[0] + n[0] * off, B[1] + n[1] * off];
      ctx.globalAlpha = 0.8;
      ctx.beginPath(); ctx.moveTo(p1[0], p1[1]); ctx.lineTo(p2[0], p2[1]); ctx.stroke();
      [[A, p1], [B, p2]].forEach(([x, y]) => {
        ctx.beginPath(); ctx.moveTo(x[0], x[1]); ctx.lineTo(y[0], y[1]); ctx.stroke();
      });
      ctx.globalAlpha = 1;
      ctx.fillStyle = '#9cc4ef';
      ctx.fillText('中心距 ' + res.centerDistance.toFixed(0), (p1[0] + p2[0]) / 2 - 30,
        (p1[1] + p2[1]) / 2 - 4);
    }
    // 带段长度（贴近段中点）
    res.edges.forEach((e, i) => {
      if (!e.feasible || e.length <= 0) return;
      const a = w2s(e.p1), b = w2s(e.p2);
      const m = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
      const lit = (State.highlight.edges || []).includes(i);
      ctx.fillStyle = lit ? '#ff9a9a' : 'rgba(150,180,210,.85)';
      ctx.fillText(e.length.toFixed(0), m[0] + 4, m[1] - 4);
    });
    // 包角弧线（结果可能比状态旧，跳过缺失轮）
    res.pulleys.forEach(wp => {
      const P = findPulley(wp.id);
      if (!P) return;
      const c = w2s([P.x, P.y]);
      const rr = wp.radius * State.view.scale + 14;
      if (rr < 18) return;
      ctx.globalAlpha = 0.55;
      ctx.beginPath();
      // canvas 角增量方向在 y 向下的屏幕上即顺时针；dir<0 才需要 anticlockwise
      ctx.arc(c[0], c[1], rr, wp.arcA0, wp.arcA0 + wp.arcDir * wp.arcSweep, wp.arcDir < 0);
      ctx.stroke();
      ctx.globalAlpha = 1;
    });
    ctx.restore();
  }

  function drawPlaceGhost() {
    if (!Drag.ghost) return;
    const { tool, x, y, w, h } = Drag.ghost;
    ctx.save();
    ctx.globalAlpha = 0.5;
    if (tool === 'obstacle') {
      ctx.strokeStyle = COLORS.obstacle; ctx.setLineDash([5, 4]); ctx.lineWidth = 1.5;
      ctx.strokeRect(w2sX(x - w / 2), w2sY(y - h / 2), w * State.view.scale, h * State.view.scale);
    } else {
      const d = Drag.ghost.d || 160;
      ctx.strokeStyle = COLORS[tool] || '#fff'; ctx.setLineDash([5, 4]);
      ctx.beginPath();
      ctx.arc(w2sX(x), w2sY(y), d / 2 * State.view.scale, 0, Math.PI * 2);
      ctx.stroke();
    }
    ctx.restore();
  }

  function render() {
    ctx.setTransform(DPR, 0, 0, DPR, 0, 0);
    ctx.clearRect(0, 0, viewW(), viewH());
    drawGrid();
    drawOverlayScheme();
    drawObstacles();
    drawQuickPath();
    drawBelt(State.result, COLORS.belt, false);
    drawDimensions();
    drawPulleys();
    drawPlaceGhost();
  }

  return { canvas, ctx, resize, fit, render, w2s, s2w: (x, y) => [s2wX(x), s2wY(y)],
    pickAt, viewW, viewH };
})();
