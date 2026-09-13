/* 前端几何工具：与 belt/geometry.py 同约定（mm，y 向下，CW=+1）。
   只用于命中检测、弧线离散和即时预览；正式校核结果以后端 /api/analyze 为准。 */
'use strict';
const GEO = (() => {
  const TAU = Math.PI * 2;
  const add = (a, b) => [a[0] + b[0], a[1] + b[1]];
  const sub = (a, b) => [a[0] - b[0], a[1] - b[1]];
  const mul = (a, s) => [a[0] * s, a[1] * s];
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1];
  const cross = (a, b) => a[0] * b[1] - a[1] * b[0];
  const len = a => Math.hypot(a[0], a[1]);
  const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);
  const norm = a => { const d = Math.hypot(a[0], a[1]) || 1; return [a[0] / d, a[1] / d]; };
  const ang = v => Math.atan2(v[1], v[0]);
  const polar = (c, r, a) => [c[0] + r * Math.cos(a), c[1] + r * Math.sin(a)];
  const cwSweep = (a0, a1) => ((a1 - a0) % TAU + TAU) % TAU;

  function arcPoints(c, r, a0, sweepCW, steps) {
    steps = steps || Math.max(8, Math.round(Math.abs(sweepCW) * 180 / Math.PI / 3));
    const pts = [];
    for (let i = 0; i <= steps; i++)
      pts.push(polar(c, r, a0 + sweepCW * i / steps));
    return pts;
  }

  // 与后端 common_tangents 同公式（用于离线下的即时预览，正常流程采用后端切点）
  function tangents(c1, r1, c2, r2, crossed) {
    const d = dist(c1, c2);
    if (d < 1e-9) return [];
    const u = norm(sub(c2, c1));
    const base = ang(u);
    const out = [];
    if (!crossed) {
      const rr = r2 - r1;
      if (Math.abs(rr) > d) return [];
      const a = Math.asin(Math.max(-1, Math.min(1, rr / d)));
      for (const q of [1, -1]) {
        const t = base + q * (Math.PI / 2 + a);
        const n = [Math.cos(t), Math.sin(t)];
        out.push({ p1: add(c1, mul(n, r1)), p2: add(c2, mul(n, r2)), q });
      }
    } else {
      if (d < r1 + r2 - 1e-9) return [];
      const rr = r1 + r2;
      const a = rr <= d ? Math.asin(rr / d) : Math.PI / 2;
      for (const q of [1, -1]) {
        const t = base + q * (Math.PI / 2 - a);
        const n1 = [Math.cos(t), Math.sin(t)];
        out.push({ p1: add(c1, mul(n1, r1)), p2: add(c2, mul(n1, -r2)), q });
      }
    }
    return out;
  }

  function velAt(c, p, dir) {
    const rx = p[0] - c[0], ry = p[1] - c[1];
    return [-dir * ry, dir * rx];
  }

  // 离线快速构建（与 build_paths 同思路），供拖拽过程中即时预览
  function quickPath(state) {
    const ps = state.pulleys;
    const map = Object.fromEntries(ps.map(p => [p.id, p]));
    const order = state.route.order.filter(id => map[id]);
    if (order.length < 2) return null;
    const driver = ps.find(p => p.kind === 'driver') || ps[0];
    const dirs = { [driver.id]: driver.dir >= 0 ? 1 : -1 };
    for (let k = 1; k < order.length; k++) {
      const a = order[k - 1], b = order[k];
      const flip = state.route.crossedEdges[a + '->' + b] ? -1 : 1;
      dirs[b] = dirs[a] * flip;
    }
    const edges = [];
    for (let i = 0; i < order.length; i++) {
      const a = order[i], b = order[(i + 1) % order.length];
      const A = map[a], B = map[b];
      const crossed = !!state.route.crossedEdges[a + '->' + b];
      const cands = tangents([A.x, A.y], A.diameter / 2, [B.x, B.y], B.diameter / 2, crossed);
      let best = null;
      for (const t of cands) {
        const va = norm(velAt([A.x, A.y], t.p1, dirs[a]));
        const vb = norm(velAt([B.x, B.y], t.p2, dirs[b]));
        const s = norm(sub(t.p2, t.p1));
        const score = Math.min(dot(va, s), dot(vb, s));
        if (!best || score > best.score) best = { ...t, score };
      }
      if (!best) best = { p1: [A.x, A.y], p2: [B.x, B.y], score: -2 };
      edges.push({ from: a, to: b, crossed, p1: best.p1, p2: best.p2,
        feasible: best.score >= 0.99 });
    }
    const arcs = {};
    order.forEach((id, i) => {
      const P = map[id], r = P.diameter / 2;
      const pin = edges[(i - 1 + order.length) % order.length].p2,
        pout = edges[i].p1;
      const a0 = ang(sub(pin, [P.x, P.y])), a1 = ang(sub(pout, [P.x, P.y]));
      let sweep = cwSweep(a0, a1);
      if (dirs[id] < 0) sweep = TAU - sweep;
      arcs[id] = { a0, sweep, dir: dirs[id], r };
    });
    return { order, edges, arcs, dirs };
  }

  function pointSegDist(p, a, b) {
    const abx = b[0] - a[0], aby = b[1] - a[1];
    const den = abx * abx + aby * aby || 1;
    let t = ((p[0] - a[0]) * abx + (p[1] - a[1]) * aby) / den;
    t = Math.max(0, Math.min(1, t));
    return Math.hypot(p[0] - (a[0] + abx * t), p[1] - (a[1] + aby * t));
  }

  function obbCorners(o) {
    const a = (o.rot || 0) * Math.PI / 180, c = Math.cos(a), s = Math.sin(a);
    return [[-o.w / 2, -o.h / 2], [o.w / 2, -o.h / 2], [o.w / 2, o.h / 2],
      [-o.w / 2, o.h / 2]].map(([x, y]) => [o.x + x * c - y * s, o.y + x * s + y * c]);
  }

  function pointInObb(p, o) {
    const a = (o.rot || 0) * Math.PI / 180, c = Math.cos(-a), s = Math.sin(-a);
    const dx = p[0] - o.x, dy = p[1] - o.y;
    return Math.abs(dx * c - dy * s) <= o.w / 2 &&
           Math.abs(dx * s + dy * c) <= o.h / 2;
  }

  return { TAU, add, sub, mul, dot, cross, len, dist, norm, ang, polar, cwSweep,
    arcPoints, tangents, velAt, quickPath, pointSegDist, obbCorners, pointInObb };
})();
