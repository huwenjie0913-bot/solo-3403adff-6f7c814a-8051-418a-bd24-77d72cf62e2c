/* 导出带尺寸标注的 SVG（世界坐标 mm，y 向下同样保留）；JSON 导入导出。 */
'use strict';
const SVG = (() => {
  function esc(s) {
    return String(s).replace(/[&<>]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' }[c]));
  }

  function arcPathD(P, wp) {
    const cx = P.x, cy = P.y;
    const r = wp.radius;
    const dir = wp.arcDir || wp.dir || 1;
    const n = Math.max(12, Math.round(Math.abs(wp.arcSweep) * 180 / Math.PI / 3));
    let d = '';
    for (let i = 0; i <= n; i++) {
      const a = wp.arcA0 + dir * wp.arcSweep * i / n;
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
    <line x2="${x2}" y2="${y2}" x1="${x2 + nx}" y1="${y2 + ny}" stroke="#2c6aa0" stroke-width=".7"/>
    <text x="${tx}" y="${ty - 3}" text-anchor="middle" class="dim">${esc(text)}</text>`;
  }

  const KIND_HEX = { driver: '#3b6ea5', driven: '#3d7a52', idler: '#73538c',
    tensioner: '#8a7138' };

  function build() {
    const res = State.result;
    const pts = [];
    State.shafts.forEach(s => pts.push([s.x - 60, s.y - 60], [s.x + 60, s.y + 60]));
    State.pulleys.forEach(p => {
      const s = findShaft(p.shaftId);
      const x = s ? s.x : p.x, y = s ? s.y : p.y;
      pts.push([x - p.diameter / 2, y - p.diameter / 2],
        [x + p.diameter / 2, y + p.diameter / 2]);
    });
    State.obstacles.forEach(o => GEO.obbCorners(o).forEach(c => pts.push(c)));
    const pad = 180;
    const x0 = Math.min(...pts.map(p => p[0])) - pad;
    const y0 = Math.min(...pts.map(p => p[1])) - pad;
    const w = Math.max(...pts.map(p => p[0])) - x0 + pad;
    const h = Math.max(...pts.map(p => p[1])) - y0 + pad;
    const body = [];
    const multi = res?.mode === 'multi';

    // 护罩
    for (const o of State.obstacles) {
      const cs = GEO.obbCorners(o);
      body.push(`<polygon points="${cs.map(c => c.join(',')).join(' ')}"
        fill="rgba(217,140,140,.12)" stroke="#d98c8c" stroke-width="1.5"/>
        <text x="${o.x}" y="${o.y + 4}" text-anchor="middle" class="lbl">${esc(o.name)}</text>`);
      const gap = res?.obstacleGaps?.[o.id];
      if (gap != null)
        body.push(`<text x="${o.x}" y="${o.y + 18}" text-anchor="middle" class="dim">间隙 ${gap.toFixed(1)} mm</text>`);
    }

    if (multi) {
      // 各回路皮带（保留回路颜色与编号）
      res.loops.forEach((lr, li) => {
        const col = lr.color || loopColor(li);
        lr.edges.forEach(e => {
          if (!e.feasible) return;
          body.push(`<line x1="${e.p1[0]}" y1="${e.p1[1]}" x2="${e.p2[0]}" y2="${e.p2[1]}"
            stroke="${col}" stroke-width="5" stroke-linecap="round"/>`);
          const m = [(e.p1[0] + e.p2[0]) / 2, (e.p1[1] + e.p2[1]) / 2];
          body.push(`<text x="${m[0] + 3}" y="${m[1] - 6}" class="seglen"
            fill="${col}">${e.length.toFixed(0)}</text>`);
          if (e.crossed)
            body.push(`<circle cx="${m[0]}" cy="${m[1]}" r="6" fill="none" stroke="${col}"/>`);
        });
        // 回路编号标在主动轮旁
        const drv = res.pulleys.find(p => p.id === lr.driver);
        if (drv) {
          body.push(`<circle cx="${drv.x - drv.radius - 14}" cy="${drv.y - drv.radius - 6}"
            r="11" fill="${col}"/><text x="${drv.x - drv.radius - 14}"
            y="${drv.y - drv.radius - 2}" text-anchor="middle" class="loopnum">${esc(lr.name)}</text>`);
        }
      });
      // 弧段（按回路颜色）
      res.pulleys.forEach(wp => {
        const P = findPulley(wp.id);
        if (!P) return;
        const s = findShaft(P.shaftId);
        const PP = { ...wp, x: s ? s.x : P.x, y: s ? s.y : P.y };
        const d = arcPathD(PP, wp);
        if (d) {
          const li = State.loops.findIndex(l => l.id === wp.loopId);
          body.push(`<path d="${d}" fill="none"
            stroke="${(res.loops.find(l => l.id === wp.loopId) || {}).color || loopColor(li)}"
            stroke-width="5"/>`);
        }
      });
    } else if (res?.edges) {
      res.edges.forEach(e => {
        if (!e.feasible) return;
        body.push(`<line x1="${e.p1[0]}" y1="${e.p1[1]}" x2="${e.p2[0]}" y2="${e.p2[1]}"
          stroke="#c8983e" stroke-width="5" stroke-linecap="round"/>`);
        body.push(`<text x="${(e.p1[0] + e.p2[0]) / 2 + 3}" y="${(e.p1[1] + e.p2[1]) / 2 - 6}"
          class="seglen">${e.length.toFixed(0)}</text>`);
      });
      res.pulleys.forEach(wp => {
        const d = arcPathD(findPulley(wp.id), wp);
        if (d) body.push(`<path d="${d}" fill="none" stroke="#c8983e" stroke-width="5"/>`);
      });
    }

    // 轮（含所属轴标注）
    for (const p of State.pulleys) {
      const s = findShaft(p.shaftId);
      const cx = s ? s.x : p.x, cy = s ? s.y : p.y;
      const r = p.diameter / 2;
      const wp = res?.pulleys?.find(x => x.id === p.id);
      const wrap = wp ? wp.wrapDeg.toFixed(1) + '°' : '';
      const rpm = wp ? Math.abs(Math.round(wp.rpm)) + ' rpm' : '';
      const lp = pulleyLoop(p.id);
      const lcol = lp ? loopColor(State.loops.indexOf(lp)) : '#888';
      body.push(`
      <circle cx="${cx}" cy="${cy}" r="${r}" fill="rgba(255,255,255,.03)"
        stroke="${KIND_HEX[p.kind] || '#666'}" stroke-width="2.5"/>
      <circle cx="${cx}" cy="${cy}" r="${r + 3}" fill="none" stroke="${lcol}"
        stroke-width="1" stroke-dasharray="6 5"/>
      <circle cx="${cx}" cy="${cy}" r="${Math.max(4, r * .12)}" fill="#888"/>
      <text x="${cx}" y="${cy - r - 10}" text-anchor="middle" class="lbl">${esc(p.name)} ⌀${p.diameter}</text>
      <text x="${cx}" y="${cy + r + 18}" text-anchor="middle" class="dim">${rpm} 包角 ${wrap}</text>`);
    }

    // 共享轴标记 + 轴载荷
    if (multi) {
      for (const s of State.shafts) {
        const rs = (res?.shafts || []).find(x => x.id === s.id);
        const multiWheel = shaftPulleys(s.id).length > 1;
        body.push(`<rect x="${s.x - 6}" y="${s.y - 6}" width="12" height="12"
          transform="rotate(45 ${s.x} ${s.y})" fill="#cfd8e3" stroke="#333"/>
          <text x="${s.x + 12}" y="${s.y - 8}" class="shaftname">⌖ ${esc(s.name)}${s.locked ? ' 🔒' : ''}</text>`);
        if (multiWheel && rs) {
          body.push(`<text x="${s.x + 12}" y="${s.y + 20}" class="dim">
            ${rs.rpm == null ? '' : Math.round(rs.rpm) + ' rpm · '}T ${rs.torque.toFixed(1)} N·m · Fr ${rs.radial.toFixed(0)} N</text>`);
        }
      }
      // 中心距（两轮回路）
      res.loops.forEach(lr => {
        if (lr.centerDistance != null && lr.edges.length === 2) {
          const a = res.pulleys.find(p => p.id === lr.order[0]);
          const b = res.pulleys.find(p => p.id === lr.order[1]);
          if (a && b) body.push(dimensionLine(a.x, a.y, b.x, b.y,
            lr.name + ' 中心距 ' + lr.centerDistance.toFixed(1) + ' mm', -46));
        }
      });
    } else if (res?.centerDistance != null && res.pulleys.length === 2) {
      const [a, b] = res.pulleys;
      body.push(dimensionLine(a.x, a.y, b.x, b.y,
        '中心距 ' + res.centerDistance.toFixed(1) + ' mm', -60));
    }

    // 标题块（多级：逐回路功率/转速/张力 + 轴载荷）
    const titleLines = [];
    titleLines.push(`<text x="${x0 + 10}" y="${y0 + 20}" class="title">
      ${multi ? '多级皮带传动布置校核图' : '皮带传动布置校核图'}</text>`);
    if (multi) {
      res.loops.forEach((lr, i) => {
        const t = lr.tension;
        titleLines.push(`<text x="${x0 + 10}" y="${y0 + 40 + i * 16}" class="dim" fill="${lr.color}">
          ${esc(lr.name)}：P ${lr.powerKw.toFixed(2)}→${lr.outPower.toFixed(2)} kW · η ${lr.efficiency.toFixed(2)}
          · n入 ${Math.round(lr.inputRpm)} rpm · v ${lr.beltSpeed.toFixed(2)} m/s · L ${lr.length.toFixed(0)} mm
          ${t ? '· F1 ' + t.f1.toFixed(0) + ' N · F2 ' + t.f2.toFixed(0) + ' N · Pmax ' + t.pmax.toFixed(2) + ' kW' : ''}</text>`);
      });
      const shaftLine = res.shafts.filter(s => shaftPulleys(s.id).length > 1)
        .map(s => `${esc(s.name)} T=${s.torque.toFixed(0)}N·m Fr=${s.radial.toFixed(0)}N`).join('；');
      if (shaftLine)
        titleLines.push(`<text x="${x0 + 10}" y="${y0 + 40 + res.loops.length * 16 + 4}" class="dim">共享轴：${shaftLine}</text>`);
    } else {
      const t = res?.tension;
      titleLines.push(`<text x="${x0 + 10}" y="${y0 + 38}" class="dim">
        带长 ${res?.beltLength?.toFixed(0) ?? '-'} mm · 带速 ${res?.beltSpeed?.toFixed(2) ?? '-'} m/s
        ${t ? '· F1 ' + t.f1.toFixed(0) + ' N · F2 ' + t.f2.toFixed(0) +
          ' N · F0 ' + t.f0.toFixed(0) + ' N · Pmax ' + t.pmax.toFixed(2) + ' kW' : ''}</text>`);
    }
    body.push(titleLines.join('\n'));

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
      .seglen{font-size:12px}
      .shaftname{font-size:13px;fill:#334e68;font-weight:600}
      .loopnum{font-size:11px;fill:#1a1a1a;font-weight:700}
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

  function exportData(includeResult) {
    return {
      format: 'belt-check', version: 2,
      globals: State.globals, pulleys: State.pulleys,
      obstacles: State.obstacles, shafts: State.shafts, loops: State.loops,
      route: State.route,
      ...(includeResult && State.result ? { result: State.result } : {})
    };
  }

  return {
    exportSVG() { download('belt-layout.svg', build(), 'image/svg+xml'); },
    exportJSON() {
      // 保留共享轴、回路编号与计算结果
      download('belt-scheme.json', JSON.stringify(exportData(true), null, 2),
        'application/json');
    },
    importJSON(file) {
      return new Promise((resolve, reject) => {
        const rd = new FileReader();
        rd.onload = () => {
          try {
            const s = JSON.parse(rd.result);
            if (!Array.isArray(s.pulleys)) throw new Error('文件不是有效的皮带方案 JSON');
            applyData(s, true);
            resolve();
          } catch (e) { reject(e); }
        };
        rd.readAsText(file);
      });
    }
  };
})();
