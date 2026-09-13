/* 导出带尺寸标注的 SVG（世界坐标 mm，y 向下同样保留）。 */
'use strict';
const SVG = (() => {
  function esc(s) {
    return String(s).replace(/[&<>]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' }[c]));
  }

  function arcPathD(wp) {
    const P = State.pulleys.find(p => p.id === wp.id);
    if (!P) return '';
    const cx = P.x, cy = P.y;
    const r = wp.radius;
    const n = Math.max(12, Math.round(Math.abs(wp.arcSweep) * 180 / Math.PI / 3));
    let d = '';
    for (let i = 0; i <= n; i++) {
      const a = wp.arcA0 + wp.arcDir * wp.arcSweep * i / n;
      const x = cx + r * Math.cos(a), y = cy + r * Math.sin(a);
      d += (i ? 'L ' : 'M ') + x.toFixed(2) + ' ' + y.toFixed(2) + ' ';
    }
    return d;
  }

  function dimensionLine(x1, y1, x2, y2, text, offset) {
    const dx = x2 - x1, dy = y2 - y1;
    const L = Math.hypot(dx, dy) || 1;
    const nx = -dy / L * offset, ny = dx / L * offset;
    const tx = (x1 + x2) / 2 + nx, ty = (y1 + y2) / 2 + ny;
    return `
    <line x1="${x1 + nx}" y1="${y1 + ny}" x2="${x2 + nx}" y2="${y2 + ny}"
          stroke="#2c6aa0" stroke-width="1.2" marker-start="url(#arr)" marker-end="url(#arr)"/>
    <line x1="${x1}" y1="${y1}" x2="${x1 + nx}" y2="${y1 + ny}" stroke="#2c6aa0" stroke-width=".7"/>
    <line x1="${x2}" y1="${y2}" x2="${x2 + nx}" y2="${y2 + ny}" stroke="#2c6aa0" stroke-width=".7"/>
    <text x="${tx}" y="${ty - 3}" text-anchor="middle" class="dim">${esc(text)}</text>`;
  }

  function build() {
    const res = State.result;
    const pts = [];
    State.pulleys.forEach(p => pts.push([p.x - p.diameter / 2, p.y - p.diameter / 2],
      [p.x + p.diameter / 2, p.y + p.diameter / 2]));
    State.obstacles.forEach(o => GEO.obbCorners(o).forEach(c => pts.push(c)));
    const pad = 160;
    const x0 = Math.min(...pts.map(p => p[0])) - pad;
    const y0 = Math.min(...pts.map(p => p[1])) - pad;
    const w = Math.max(...pts.map(p => p[0])) - x0 + pad;
    const h = Math.max(...pts.map(p => p[1])) - y0 + pad;
    const body = [];

    // 护罩
    for (const o of State.obstacles) {
      const cs = GEO.obbCorners(o);
      body.push(`<polygon points="${cs.map(c => c.join(',')).join(' ')}"
        fill="rgba(217,140,140,.12)" stroke="#d98c8c" stroke-width="1.5"/>
        <text x="${o.x}" y="${o.y + 4}" text-anchor="middle" class="lbl">${esc(o.name)}</text>`);
      const gap = res?.obstacleGaps?.[o.id];
      if (gap != null)
        body.push(`<text x="${o.x}" y="${o.y + 18}" text-anchor="middle" class="dim">
          间隙 ${gap.toFixed(1)} mm</text>`);
    }

    // 皮带
    if (res?.edges) {
      let d = '';
      res.edges.forEach((e, i) => {
        d += `M ${e.p1[0].toFixed(2)} ${e.p1[1].toFixed(2)} L ${e.p2[0].toFixed(2)} ${e.p2[1].toFixed(2)} `;
      });
      body.push(`<path d="${d}" fill="none" stroke="#c8983e" stroke-width="6" stroke-linecap="round"/>`);
      res.pulleys.forEach(wp => {
        const d = arcPathD(wp);
        if (d) body.push(`<path d="${d}" fill="none" stroke="#c8983e" stroke-width="6"/>`);
      });
      // 段长标注
      res.edges.forEach(e => {
        if (!e.feasible) return;
        const m = [(e.p1[0] + e.p2[0]) / 2, (e.p1[1] + e.p2[1]) / 2];
        body.push(`<text x="${m[0] + 3}" y="${m[1] - 6}" class="seglen">${e.length.toFixed(0)}</text>`);
      });
    }

    // 轮
    for (const p of State.pulleys) {
      const r = p.diameter / 2;
      const wp = res?.pulleys?.find(x => x.id === p.id);
      const wrap = wp ? wp.wrapDeg.toFixed(1) + '°' : '';
      const rpm = wp ? Math.abs(Math.round(wp.rpm)) + ' rpm' : '';
      body.push(`
      <circle cx="${p.x}" cy="${p.y}" r="${r}" fill="rgba(255,255,255,.03)"
        stroke="${({ driver: '#5b9bd5', driven: '#58c08a', idler: '#b48cd4', tensioner: '#e0b65a' })[p.kind]}"
        stroke-width="2.5"/>
      <circle cx="${p.x}" cy="${p.y}" r="${Math.max(4, r * .12)}" fill="#888"/>
      <text x="${p.x}" y="${p.y - r - 10}" text-anchor="middle" class="lbl">${esc(p.name)} ⌀${p.diameter}</text>
      <text x="${p.x}" y="${p.y + r + 18}" text-anchor="middle" class="dim">${rpm} 包角 ${wrap}</text>
      ${p.locked ? `<text x="${p.x + r - 6}" y="${p.y - r + 14}" class="lbl">🔒</text>` : ''}`);
    }

    // 中心距（两轮）
    if (res?.centerDistance != null && res.pulleys.length === 2) {
      const [a, b] = res.pulleys;
      body.push(dimensionLine(a.x, a.y, b.x, b.y,
        '中心距 ' + res.centerDistance.toFixed(1) + ' mm', -60));
    }

    // 标题块
    const t = res?.tension;
    body.push(`
    <text x="${x0 + 10}" y="${y0 + 20}" class="title">皮带传动布置校核图</text>
    <text x="${x0 + 10}" y="${y0 + 38}" class="dim">
      带长 ${res?.beltLength?.toFixed(0) ?? '-'} mm · 带速 ${res?.beltSpeed?.toFixed(2) ?? '-'} m/s
      ${t ? '· F1 ' + t.f1.toFixed(0) + ' N · F2 ' + t.f2.toFixed(0) +
        ' N · F0 ' + t.f0.toFixed(0) + ' N · Pmax ' + t.pmax.toFixed(2) + ' kW' : ''}
    </text>`);

    const svg = `<svg xmlns="http://www.w3.org/2000/svg" viewBox="${x0} ${y0} ${w} ${h}"
      width="${w}" height="${h}" font-family="Segoe UI, sans-serif">
    <defs>
      <marker id="arr" markerWidth="8" markerHeight="8" refX="4" refY="4" orient="auto">
        <path d="M0,0 L8,4 L0,8 z" fill="#2c6aa0"/></marker>
    </defs>
    <rect x="${x0}" y="${y0}" width="${w}" height="${h}" fill="#fbfaf6"/>
    <g fill="#333">
    <style>
      .lbl{font-size:16px;fill:#243b53;font-weight:600}
      .dim{font-size:13px;fill:#2c6aa0}
      .seglen{font-size:12px;fill:#8a6a20}
      .title{font-size:22px;fill:#102a43;font-weight:700}
    </style>
    ${body.join('\n')}
    </g></svg>`;
    return svg;
  }

  function download(filename, text, mime) {
    const blob = new Blob([text], { type: mime });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = filename;
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 4000);
  }

  return {
    exportSVG() { download('belt-layout.svg', build(), 'image/svg+xml'); },
    exportJSON() {
      const data = { format: 'belt-check', version: 1,
        globals: State.globals, pulleys: State.pulleys,
        obstacles: State.obstacles, route: State.route };
      download('belt-scheme.json', JSON.stringify(data, null, 2), 'application/json');
    },
    importJSON(file) {
      return new Promise((resolve, reject) => {
        const rd = new FileReader();
        rd.onload = () => {
          try {
            const s = JSON.parse(rd.result);
            if (!Array.isArray(s.pulleys) || !s.route) throw new Error('文件不是有效的皮带方案 JSON');
            State.globals = { ...DEFAULT_GLOBALS, ...(s.globals || {}) };
            State.pulleys = s.pulleys;
            State.obstacles = s.obstacles || [];
            State.route = { order: s.route.order || s.pulleys.map(p => p.id),
              crossedEdges: s.route.crossedEdges || {} };
            State.overlayScheme = null;
            resolve();
          } catch (e) { reject(e); }
        };
        rd.readAsText(file);
      });
    }
  };
})();
