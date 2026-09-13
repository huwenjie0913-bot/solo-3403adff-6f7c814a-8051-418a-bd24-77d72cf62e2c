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
    State.shafts.forEach(s => pts.push([s.x - 40, s.y - 40], [s.x + 40, s.y + 40]));
    State.pulleys.forEach(p => {
      const s = findShaft(p.shaftId);
      const x = s ? s.x : p.x, y = s ? s.y : p.y;
      const r = p.diameter / 2;
      pts.push([x - r, y - r], [x + r, y + r]);
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
  function pulleyPos(p) {
    const s = findShaft(p.shaftId);
    return s ? [s.x, s.y] : [p.x, p.y];
  }
  function hitPulleyAt(w) {
    let best = null, bd = Infinity;
    for (const p of State.pulleys) {
      const c = pulleyPos(p);
      const d = GEO.dist(w, c) - p.diameter / 2;
      if (d <= 6 && d < bd) { bd = d; best = p; }
    }
    return best;
  }
  function hitShaftCenter(w) {
    // 共享轴轴芯小方块（同位置多轮时点击轮毂）
    let best = null, bd = 14;
    for (const s of State.shafts) {
      const d = GEO.dist(w, [s.x, s.y]);
      if (d < bd) { bd = d; best = s; }
    }
    return best;
  }
  function hitObstacleAt(w) {
    for (const o of State.obstacles) {
      if (GEO.pointInObb(w, o)) return o;
      const cs = GEO.obbCorners(o);
      for (let i = 0; i < 4; i++)
        if (GEO.pointSegDist(w, cs[i], cs[(i + 1) % 4]) < 6) return o;
    }
    return null;
  }
  function edgeHit(w) {
    let best = null, bd = 8;
    (State.result?.loops || []).forEach(lr => {
      lr.edges.forEach((e, i) => {
        if (!e.feasible) return;
        const d = GEO.pointSegDist(w, e.p1, e.p2);
        if (d < bd) { bd = d; best = { loopId: lr.id, index: i }; }
      });
    });
    return best;
  }
  function pickAt(sx, sy) {
    const w = [s2wX(sx), s2wY(sy)];
    // 带段优先（便于看告警）
    const eh = edgeHit(w);
    if (eh) return { type: 'edge', loopId: eh.loopId, index: eh.index };
    const p = hitPulleyAt(w);
    if (p) return { type: 'pulley', id: p.id };
    // 多个轮同位置：轮毂区域点中轴
    const s = hitShaftCenter(w);
    if (s) return { type: 'shaft', id: s.id };
    const o = hitObstacleAt(w);
    if (o) return { type: 'obstacle', id: o.id };
    return null;
  }

  const KIND_COLORS = {
    driver: '#5b9bd5', driven: '#58c08a', idler: '#b48cd4',
    tensioner: '#e0b65a', obstacle: '#d98c8c'
  };

  function drawGrid() {
    const sc = State.view.scale;
    const stepMm = [20, 50, 100, 200, 500, 1000, 2000].find(s => s * sc > 28) || 2000;
    const gx0 = Math.floor(s2wX(0) / stepMm) * stepMm;
    const gx1 = Math.ceil(s2wX(viewW()) / stepMm) * stepMm;
    const gy0 = Math.floor(s2wY(0) / stepMm) * stepMm;
    const gy1 = Math.ceil(s2wY(viewH()) / stepMm) * stepMm;
    ctx.strokeStyle = '#262c33'; ctx.lineWidth = 1;
    ctx.beginPath();
    for (let x = gx0; x <= gx1; x += stepMm) {
      ctx.moveTo(w2sX(x), 0); ctx.lineTo(w2sX(x), viewH());
    }
    for (let y = gy0; y <= gy1; y += stepMm) {
      ctx.moveTo(0, w2sY(y)); ctx.lineTo(w2sY(y), viewH());
    }
    ctx.stroke();
    ctx.strokeStyle = '#333c46'; ctx.lineWidth = 1.5;
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
    ctx.globalAlpha = 0.5;
    for (const p of ov.scheme.pulleys || []) {
      const c = w2s([p.x, p.y]), r = p.diameter / 2 * State.view.scale;
      ctx.strokeStyle = '#7f93a8'; ctx.setLineDash([5, 4]); ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.arc(c[0], c[1], r, 0, Math.PI * 2); ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = '#7f93a8'; ctx.font = '10px sans-serif';
      ctx.fillText('(对比)' + p.name, c[0] - r, c[1] - r - 2);
    }
    if (res?.mode === 'multi') {
      (res.loops || []).forEach(lr => drawBeltLoop(lr, '#7f93a8', true));
    } else {
      drawBeltLegacy(res, '#7f93a8', true);
    }
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
      ctx.strokeStyle = lit ? '#ef5f5f' : (active ? '#fff' : KIND_COLORS.obstacle);
      ctx.lineWidth = active || lit ? 2.2 : 1.4;
      if (active) ctx.setLineDash([6, 3]);
      ctx.stroke();
      ctx.setLineDash([]);
      const c = w2s([o.x, o.y]);
      ctx.fillStyle = '#e8b0b0'; ctx.font = '11px sans-serif'; ctx.textAlign = 'center';
      ctx.fillText(o.name || o.id, c[0], c[1]);
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

  function arcPathOn(cx, cy, r, wp) {
    const a0 = wp.arcA0, sweep = (wp.arcDir || 1) * wp.arcSweep;
    const n = Math.max(10, Math.round(Math.abs(wp.arcSweep) * 180 / Math.PI / 3));
    ctx.beginPath();
    for (let i = 0; i <= n; i++) {
      const a = a0 + sweep * i / n;
      const x = cx + r * Math.cos(a), y = cy + r * Math.sin(a);
      i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
    }
  }

  function lrefActive(lid, i) {
    return (State.highlight.lrefs || []).some(p => p[0] === lid && p[1] === i);
  }
  function loopDim(lid) {
    const hl = State.highlight.loops || [];
    return hl.length && !hl.includes(lid);
  }

  function drawBeltLoop(lr, forceColor, dashed) {
    if (!lr?.edges?.length) return;
    const color = forceColor || loopColor(State.loops.findIndex(l => l.id === lr.id));
    const dim = !forceColor && loopDim(lr.id);
    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = 5;
    ctx.lineJoin = 'round'; ctx.lineCap = 'round';
    if (dashed) ctx.setLineDash([7, 5]);
    if (dim) ctx.globalAlpha = 0.18;
    lr.edges.forEach((e, i) => {
      if (!e.feasible) {
        ctx.save(); ctx.globalAlpha = dim ? 0.1 : 0.35; ctx.setLineDash([3, 6]);
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
      if (lrefActive(lr.id, i)) {
        ctx.save(); ctx.strokeStyle = '#ff6b6b'; ctx.lineWidth = 9;
        ctx.globalAlpha = 0.55; ctx.stroke(); ctx.restore();
      }
      if (e.crossed) {
        const m = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
        ctx.strokeStyle = color; ctx.lineWidth = 2;
        ctx.beginPath(); ctx.arc(m[0], m[1], 6, 0, Math.PI * 2); ctx.stroke();
      }
    });
    // 弧段
    const wpMap = Object.fromEntries((State.result?.pulleys || [])
      .filter(p => p.loopId === lr.id).map(p => [p.id, p]));
    Object.values(wpMap).forEach(wp => {
      const P = findPulley(wp.id);
      if (!P) return;
      const s = findShaft(P.shaftId);
      const c = w2s(s ? [s.x, s.y] : [P.x, P.y]);
      const r = wp.radius * State.view.scale;
      const lit = (State.highlight.pulleys || []).includes(wp.id);
      arcPathOn(c[0], c[1], r, wp);
      ctx.save();
      ctx.strokeStyle = lit ? '#ff6b6b' : color;
      ctx.lineWidth = lit ? 8 : 5;
      if (lit) ctx.globalAlpha = 0.9;
      if (dim) ctx.globalAlpha = 0.15;
      ctx.stroke();
      ctx.restore();
    });
    ctx.restore();
  }

  function drawBeltLegacy(res, color, dashed) {
    if (!res?.edges) return;
    ctx.save();
    ctx.strokeStyle = color; ctx.lineWidth = 5;
    ctx.lineJoin = 'round'; ctx.lineCap = 'round';
    if (dashed) ctx.setLineDash([7, 5]);
    res.edges.forEach((e, i) => {
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
    });
    (res.pulleys || []).forEach(wp => {
      const P = findPulley(wp.id);
      if (!P) return;
      const c = w2s([P.x, P.y]);
      arcPathOn(c[0], c[1], wp.radius * State.view.scale, wp);
      ctx.strokeStyle = color; ctx.lineWidth = 5; ctx.stroke();
    });
    ctx.restore();
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

  function drawPulleys() {
    const res = State.result;
    const wpMap = Object.fromEntries((res?.pulleys || []).map(p => [p.id, p]));
    const hl = State.highlight.pulleys || [];
    for (const p of State.pulleys) {
      const s = findShaft(p.shaftId);
      const c = w2s(s ? [s.x, s.y] : [p.x, p.y]);
      const r = p.diameter / 2 * State.view.scale;
      const active = State.selection.type === 'pulley' && State.selection.id === p.id;
      const lit = hl.includes(p.id);
      const lp = pulleyLoop(p.id);
      const baseCol = KIND_COLORS[p.kind] || '#999';
      const loopCol = lp ? loopColor(State.loops.indexOf(lp)) : '#888';
      const dim = lp && loopDim(lp.id);
      ctx.save();
      if (dim) ctx.globalAlpha = 0.3;
      ctx.beginPath(); ctx.arc(c[0], c[1], r, 0, Math.PI * 2);
      ctx.fillStyle = 'rgba(255,255,255,.03)'; ctx.fill();
      ctx.strokeStyle = lit ? '#ff6b6b' : baseCol;
      ctx.lineWidth = active ? 2.8 : 2;
      ctx.stroke();
      // 回路颜色色环（外圈虚线）
      ctx.beginPath(); ctx.arc(c[0], c[1], r + 3.5, 0, Math.PI * 2);
      ctx.strokeStyle = loopCol; ctx.lineWidth = 1.4;
      ctx.setLineDash([6, 5]); ctx.globalAlpha = (dim ? 0.25 : 0.8); ctx.stroke();
      ctx.setLineDash([]); ctx.globalAlpha = dim ? 0.3 : 1;
      // 轮毂
      ctx.beginPath(); ctx.arc(c[0], c[1], Math.max(2.5, r * 0.12), 0, Math.PI * 2);
      ctx.fillStyle = baseCol; ctx.fill();
      const wp = wpMap[p.id];
      const dir = wp ? wp.dir : (p.dir || 1);
      drawRotationArrow(c, r, dir, baseCol);
      // 标签
      ctx.font = '12px sans-serif'; ctx.textAlign = 'center';
      ctx.fillStyle = '#e8eef5';
      ctx.fillText(p.name, c[0], c[1] - r - 16);
      if (wp) {
        const lpWrapMin = lp && (lp.minWrapDeg ?? State.globals.minWrapDeg);
        ctx.font = '10px Consolas, monospace';
        ctx.fillStyle = wp.wrapDeg < lpWrapMin ? '#ff8a8a' : '#9fb4c7';
        ctx.fillText(`⌀${p.diameter}  ${Math.abs(Math.round(wp.rpm))} rpm  θ${wp.wrapDeg.toFixed(0)}°`,
          c[0], c[1] + r + 14);
        ctx.fillStyle = loopCol;
        ctx.fillText(`${wp.dir > 0 ? 'CW' : 'CCW'}`, c[0] + r + 12, c[1] + 4);
      }
      ctx.restore();
    }

    // 共享轴轴芯与标签
    for (const s of State.shafts) {
      const c = w2s([s.x, s.y]);
      const rs = resultShaft(s.id);
      const active = State.selection.type === 'shaft' && State.selection.id === s.id;
      const lit = (State.highlight.shafts || []).includes(s.id);
      const multi = shaftPulleys(s.id).length > 1;
      ctx.save();
      ctx.fillStyle = lit ? '#ff6b6b' : (active ? '#fff' : '#cfd8e3');
      ctx.strokeStyle = '#1b2026'; ctx.lineWidth = 1.5;
      const k = 7;
      ctx.beginPath();
      ctx.moveTo(c[0], c[1] - k); ctx.lineTo(c[0] + k, c[1]);
      ctx.lineTo(c[0], c[1] + k); ctx.lineTo(c[0] - k, c[1]);
      ctx.closePath(); ctx.fill(); ctx.stroke();
      if (s.locked) {
        ctx.font = '12px sans-serif';
        ctx.fillText('🔒', c[0] + k + 2, c[1] - k - 2);
      }
      ctx.font = '11px sans-serif'; ctx.textAlign = 'center';
      ctx.fillStyle = lit ? '#ff9a9a' : (multi ? '#e8eef5' : '#7d8fa1');
      const label = (multi ? '⌖ ' : '') + s.name +
        (rs && rs.rpm != null ? `  ${Math.round(rs.rpm)} rpm` : '');
      ctx.fillText(label, c[0], c[1] + (multi ? 26 : 16));
      if (multi && rs) {
        const tBad = s.allowTorque > 0 && rs.torque > s.allowTorque;
        const fBad = s.allowRadial > 0 && rs.radial > s.allowRadial;
        ctx.font = '10px Consolas, monospace';
        ctx.fillStyle = tBad ? '#ff8a8a' : '#9fb4c7';
        ctx.fillText(`T ${rs.torque.toFixed(0)} N·m`, c[0], c[1] + 38);
        ctx.fillStyle = fBad ? '#ff8a8a' : '#9fb4c7';
        ctx.fillText(`Fr ${rs.radial.toFixed(0)} N`, c[0], c[1] + 50);
      }
      ctx.restore();
    }
  }

  function drawQuickPath() {
    if (!Drag.mode) return;
    const qq = State.quick;
    if (!qq) return;
    const loops = qq.loops || [{ id: State.activeLoopId, ...qq }];
    ctx.save();
    loops.forEach(q => {
      if (!q.edges || !q.edges.length) return;
      const col = loopColor(State.loops.findIndex(l => l.id === q.id));
      ctx.strokeStyle = col; ctx.lineWidth = 4;
      ctx.setLineDash([8, 6]); ctx.globalAlpha = 0.85;
      ctx.beginPath();
      let started = false;
      q.order.forEach((id, i) => {
        const e = q.edges[i];
        const a = w2s(e.p1), b = w2s(e.p2);
        if (!started) { ctx.moveTo(a[0], a[1]); started = true; }
        ctx.lineTo(b[0], b[1]);
        const arc = q.arcs[e.to];
        const n = Math.max(8, Math.round(Math.abs(arc.sweep) * 180 / Math.PI / 4));
        for (let kk = 1; kk <= n; kk++) {
          const ang = arc.a0 + arc.dir * arc.sweep * kk / n;
          const P = findPulley(e.to);
          const s = findShaft(P.shaftId);
          const cx = s ? s.x : P.x, cy = s ? s.y : P.y;
          ctx.lineTo(w2sX(cx + arc.r * Math.cos(ang)),
                     w2sY(cy + arc.r * Math.sin(ang)));
        }
      });
      ctx.stroke();
    });
    ctx.restore();
  }

  function drawDimensions() {
    const res = State.result;
    if (!res || res.mode !== 'multi') return;
    ctx.save();
    ctx.font = '10px Consolas, monospace';
    (res.loops || []).forEach(lr => {
      const col = loopColor(State.loops.findIndex(l => l.id === lr.id));
      ctx.fillStyle = col; ctx.strokeStyle = col;
      // 两轮中心距
      if (lr.centerDistance != null && lr.edges.length === 2) {
        const wp0 = res.pulleys.find(p => p.id === lr.order[0]);
        const wp1 = res.pulleys.find(p => p.id === lr.order[1]);
        if (wp0 && wp1) {
          const A = w2s([wp0.x, wp0.y]), B = w2s([wp1.x, wp1.y]);
          const u = GEO.norm([B[0] - A[0], B[1] - A[1]]);
          const off = 34, n = [-u[1], u[0]];
          const p1 = [A[0] + n[0] * off, A[1] + n[1] * off];
          const p2 = [B[0] + n[0] * off, B[1] + n[1] * off];
          ctx.globalAlpha = 0.7;
          ctx.beginPath(); ctx.moveTo(p1[0], p1[1]); ctx.lineTo(p2[0], p2[1]); ctx.stroke();
          [[A, p1], [B, p2]].forEach(([x, y]) => {
            ctx.beginPath(); ctx.moveTo(x[0], x[1]); ctx.lineTo(y[0], y[1]); ctx.stroke();
          });
          ctx.globalAlpha = 1;
          ctx.fillStyle = col;
          ctx.fillText('中心距 ' + lr.centerDistance.toFixed(0),
            (p1[0] + p2[0]) / 2 - 28, (p1[1] + p2[1]) / 2 - 4);
        }
      }
      // 段长
      lr.edges.forEach(e => {
        if (!e.feasible || e.length <= 0) return;
        const a = w2s(e.p1), b = w2s(e.p2);
        const m = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
        ctx.globalAlpha = lrefActive(lr.id, lr.edges.indexOf(e)) ? 1 : 0.7;
        ctx.fillStyle = lrefActive(lr.id, lr.edges.indexOf(e)) ? '#ff9a9a' : col;
        ctx.fillText(e.length.toFixed(0), m[0] + 4, m[1] - 4);
        ctx.globalAlpha = 1;
      });
    });
    // 包角弧线
    res.pulleys.forEach(wp => {
      const P = findPulley(wp.id);
      if (!P) return;
      const s = findShaft(P.shaftId);
      const c = w2s(s ? [s.x, s.y] : [P.x, P.y]);
      const rr = wp.radius * State.view.scale + 14;
      if (rr < 18) return;
      ctx.globalAlpha = 0.5;
      ctx.strokeStyle = loopColor(State.loops.findIndex(l => l.id === wp.loopId));
      ctx.beginPath();
      ctx.arc(c[0], c[1], rr, wp.arcA0, wp.arcA0 + (wp.arcDir || 1) * wp.arcSweep,
        (wp.arcDir || 1) < 0);
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
      ctx.strokeStyle = KIND_COLORS.obstacle; ctx.setLineDash([5, 4]); ctx.lineWidth = 1.5;
      ctx.strokeRect(w2sX(x - w / 2), w2sY(y - h / 2), w * State.view.scale, h * State.view.scale);
    } else {
      const d = Drag.ghost.d || 160;
      ctx.strokeStyle = KIND_COLORS[tool] || '#fff'; ctx.setLineDash([5, 4]);
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
    const res = State.result;
    if (res?.mode === 'multi') {
      (res.loops || []).forEach(lr => drawBeltLoop(lr));
    } else {
      drawBeltLegacy(res, '#d8b15a', false);
    }
    drawDimensions();
    drawPulleys();
    drawPlaceGhost();
  }

  return { canvas, ctx, resize, fit, render, w2s, s2w: (x, y) => [s2wX(x), s2wY(y)],
    pickAt, pulleyPos, viewW, viewH };
})();
