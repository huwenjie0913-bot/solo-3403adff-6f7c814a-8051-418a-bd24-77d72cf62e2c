/* 主程序：装配各模块，渲染告警/依据/汇总，方案库与叠加比较。 */
'use strict';
const $ = id => document.getElementById(id);

const App = (() => {
  let analyzeToken = 0;

  async function invalidate(quickOnly) {
    safeRender();
    if (quickOnly) {
      // 拖拽中本地画快速路径（正式数据仍由节流请求更新）
      State.quick = GEO.quickLoops(State);
    }
    const my = ++analyzeToken;
    await API.scheduleAnalyze(res => {
      if (my !== analyzeToken) return;
      afterAnalyze(res);
    });
  }

  function safeRender() {
    try { Render.render(); }
    catch (e) { console.error('render failed:', e); }
  }

  function afterAnalyze(res) {
    renderWarns(res);
    renderBasis(res);
    renderSummary(res);
    Props.renderProps();
    Render.render();
  }

  function refreshAll() {
    Props.renderGlobal();
    Props.renderLoops();
    Props.renderProps();
    Props.renderRoute();
    invalidate();
  }

  // ---------- 告警 ----------
  function renderWarns(res) {
    const box = $('warn-list'), badge = $('warn-count');
    box.innerHTML = '';
    const ws = res?.warnings || [];
    const errs = ws.filter(w => w.severity === 'error').length;
    const warns = ws.filter(w => w.severity === 'warning').length;
    badge.textContent = errs ? errs + ' 错' : warns ? warns + ' 警' : '✓ 通过';
    badge.className = 'badge ' + (errs ? 'err' : warns ? 'warn' : 'ok');
    if (!ws.length) {
      box.innerHTML = '<p class="hint" style="color:var(--ok)">✓ 未发现几何、包角、张力、轴载或干涉问题。</p>';
      return;
    }
    ws.forEach((w, i) => {
      const div = document.createElement('div');
      div.className = 'warn-item ' + w.severity;
      if (State.highlight.activeWarn === i) div.classList.add('selected');
      if (w.loop) div.style.borderLeftColor = loopColor(w.loop);
      const refNames = [];
      (w.refs.pulleys || []).forEach(id => {
        const p = findPulley(id); if (p) refNames.push(p.name);
      });
      (w.refs.shafts || []).forEach(id => {
        const s = findShaft(id); if (s) refNames.push('轴:' + s.name);
      });
      (w.refs.obstacles || []).forEach(id => {
        const o = findObstacle(id); if (o) refNames.push(o.name);
      });
      const lpName = w.loop
        ? (State.loops.find(l => l.id === w.loop)?.name || w.loop) : null;
      div.innerHTML = `<span class="code">${w.code}${lpName ? ' · ' + esc(lpName) : ''}</span>
        <div>${w.severity === 'error' ? '⛔' : '⚠'} ${esc(w.message)}</div>
        ${refNames.length ? `<div class="refs">关联：${refNames.map(esc).join('、')}</div>` : ''}`;
      div.addEventListener('click', () => activateWarn(w));
      box.appendChild(div);
    });
  }

  function warnsForEdge(lid, i) {
    return (State.result?.warnings || []).filter(w =>
      (w.refs.lrefs || []).some(p => p[0] === lid && p[1] === i));
  }

  function activateWarn(w, keepEdgeSel) {
    const refs = w.refs || {};
    const lrefs = refs.lrefs || [];
    State.highlight = {
      pulleys: refs.pulleys || [],
      edges: [],
      lrefs,
      obstacles: refs.obstacles || [],
      shafts: refs.shafts || [],
      loops: refs.loops || (w.loop ? [w.loop] : []),
      activeWarn: (State.result?.warnings || []).indexOf(w)
    };
    // 同步选中第一个关联对象
    if ((refs.shafts || []).length)
      selectObj('shaft', refs.shafts[0]);
    else if ((refs.pulleys || []).length)
      selectObj('pulley', refs.pulleys[0]);
    else if ((refs.obstacles || []).length)
      selectObj('obstacle', refs.obstacles[0]);
    else if (!keepEdgeSel) selectObj(null, null);
    document.querySelector('.tab[data-tab=basis]').click();
    Props.renderProps(); Props.renderRoute(); Props.renderLoops();
    renderWarns(State.result);
    renderBasis(State.result);
    focusHighlight();
    Render.render();
  }

  function focusHighlight() {
    const pts = [];
    State.highlight.pulleys.forEach(id => {
      const p = findPulley(id);
      const s = p && findShaft(p.shaftId);
      if (p) pts.push(s ? [s.x, s.y] : [p.x, p.y]);
    });
    (State.highlight.shafts || []).forEach(id => {
      const s = findShaft(id); if (s) pts.push([s.x, s.y]);
    });
    State.highlight.obstacles.forEach(id => {
      const o = findObstacle(id); if (o) GEO.obbCorners(o).forEach(c => pts.push(c));
    });
    (State.highlight.lrefs || []).forEach(([lid, i]) => {
      const e = State.result?.loops?.find(l => l.id === lid)?.edges?.[i];
      if (e) pts.push(e.p1, e.p2);
    });
    if (!pts.length) return;
    const cx = pts.reduce((s, p) => s + p[0], 0) / pts.length;
    const cy = pts.reduce((s, p) => s + p[1], 0) / pts.length;
    State.view.ox = Render.viewW() / 2 - cx * State.view.scale;
    State.view.oy = Render.viewH() / 2 - cy * State.view.scale;
  }

  // ---------- 计算依据 / 汇总 ----------
  function renderBasis(res) {
    const idx = State.highlight.activeWarn;
    const w = (idx != null) ? res?.warnings?.[idx] : null;
    if (w) {
      const bc = $('basis-content');
      bc.innerHTML = `<h4 style="color:${w.severity === 'error' ? 'var(--err)' : 'var(--warn)'}">
          ${w.severity === 'error' ? '⛔' : '⚠'} ${esc(w.title)}</h4>
        ${w.basis?.length ? '<ul>' + w.basis.map(b => `<li>${esc(b)}</li>`).join('') + '</ul>'
          : '<p class="hint">该告警为直接几何判定。</p>'}
        <h4>全局计算依据</h4>
        <ul>${(res?.globalBasis || []).map(b => `<li class="formula">${esc(b)}</li>`).join('')}</ul>`;
    } else {
      $('basis-content').innerHTML =
        `<ul>${(res?.globalBasis || []).map(b => `<li class="formula">${esc(b)}</li>`).join('')}</ul>`;
    }
  }

  function renderSummary(res) {
    if (!res) return;
    if (res.mode !== 'multi') { renderSummaryLegacy(res); return; }
    let html = '<h4>共享轴（各级输入/输出转速、扭矩、径向合力）</h4>';
    html += '<table class="cmp"><tr><th>轴</th><th>带轮</th><th>转速</th><th>转向</th>'
      + '<th>扭矩 N·m</th><th>径向合力 N</th><th>回路</th></tr>';
    for (const s of res.shafts) {
      const tBad = s.allowTorque > 0 && s.torque > s.allowTorque;
      const fBad = s.allowRadial > 0 && s.radial > s.allowRadial;
      const wheels = s.pulleys.map(pid => {
        const p = findPulley(pid);
        const rp = resultPulley(pid);
        return p ? `${esc(p.name)} ⌀${p.diameter}` +
          (rp ? `（${Math.round(rp.rpm)} rpm）` : '') : pid;
      }).join('<br>');
      html += `<tr class="shaft-row" data-sid="${s.id}" style="cursor:pointer${
        (State.selection.type === 'shaft' && State.selection.id === s.id)
          ? ';background:#2c4d74' : ''}">
        <td>${esc(s.name)}${s.locked ? ' 🔒' : ''}</td>
        <td style="text-align:left">${wheels}</td>
        <td>${s.rpm == null ? '—' : Math.round(s.rpm)}</td>
        <td>${s.rpm == null ? '—' : (s.dir > 0 ? 'CW' : 'CCW')}</td>
        <td class="${tBad ? 'bad' : ''}">${s.torque.toFixed(1)}${s.allowTorque
          ? `<div class="sub">/ ${s.allowTorque}</div>` : ''}</td>
        <td class="${fBad ? 'bad' : ''}">${s.radial.toFixed(0)}${s.allowRadial
          ? `<div class="sub">/ ${s.allowRadial}</div>` : ''}</td>
        <td>${s.loops.map(lid =>
          `<span style="color:${loopColor(lid)}">${esc((State.loops.find(l => l.id === lid) || {}).name || lid)}</span>`)
          .join('<br>')}</td></tr>`;
    }
    html += '</table>';

    html += '<h4>各级皮带回路</h4><table class="cmp"><tr><th>回路</th><th>功率 kW</th>'
      + '<th>η</th> <th>输入 rpm</th><th>输出 rpm</th><th>带速 m/s</th><th>带长 mm</th>'
      + '<th>F1 N</th><th>F2 N</th><th>Pmax kW</th></tr>';
    for (const lr of res.loops) {
      const t = lr.tension;
      const outRpm = lr.order.map(id => resultPulley(id)).filter(p => p && p.kind === 'driven')
        .map(p => Math.round(p.rpm)).join(', ');
      html += `<tr style="cursor:pointer" class="loop-row" data-lid="${lr.id}">
        <td><span style="color:${loopColor(lr.id)}">${esc(lr.name)}</span>${!lr.ok ? ' ⛔' : ''}</td>
        <td>${lr.powerKw.toFixed(2)}<div class="sub">出 ${lr.outPower.toFixed(2)}</div></td>
        <td>${lr.efficiency.toFixed(2)}</td>
        <td>${Math.round(lr.inputRpm)}</td>
        <td>${outRpm || '—'}</td>
        <td>${lr.beltSpeed.toFixed(2)}</td>
        <td>${lr.length.toFixed(0)}</td>
        <td>${t ? t.f1.toFixed(0) : '—'}</td>
        <td>${t ? t.f2.toFixed(0) : '—'}</td>
        <td>${t ? t.pmax.toFixed(2) : '—'}</td></tr>`;
    }
    html += '</table>';

    const travelers = res.loops.flatMap(lr => lr.traveler.map(tr => ({ ...tr, loop: lr })));
    if (travelers.length) {
      html += '<h4>张紧轮行程</h4><div class="kvgrid">';
      for (const tr of travelers) {
        const p = findPulley(tr.id);
        html += `<div style="border-left:3px solid ${loopColor(tr.loop.id)};padding-left:6px">
          ${esc(p?.name || tr.id)} 缩短效率 <b>${tr.rate.toFixed(2)}</b></div>
          <div>需补偿带长 <b>${tr.need.toFixed(1)} mm</b></div>
          <div>需要行程 / 可用 <b class="${tr.neededTravel > tr.available ? 'bad' : 'good'}">
          ${tr.neededTravel ? tr.neededTravel.toFixed(1) : '—'} / ${tr.available} mm</b></div>`;
      }
      html += '</div>';
    }
    $('summary-content').innerHTML = html;
    $('summary-content').querySelectorAll('.shaft-row').forEach(tr =>
      tr.addEventListener('click', () => {
        selectObj('shaft', tr.dataset.sid);
        const rs = resultShaft(tr.dataset.sid);
        State.highlight = { pulleys: rs?.pulleys || [], edges: [], lrefs: [],
          obstacles: [], shafts: [tr.dataset.sid], loops: rs?.loops || [], activeWarn: null };
        Props.renderProps(); Props.renderRoute(); renderSummary(State.result); Render.render();
      }));
    $('summary-content').querySelectorAll('.loop-row').forEach(tr =>
      tr.addEventListener('click', () => {
        State.activeLoopId = tr.dataset.lid;
        const lr = resultLoop(tr.dataset.lid);
        State.highlight = { pulleys: lr?.order || [], edges: [], lrefs: [],
          obstacles: [], shafts: [], loops: [tr.dataset.lid], activeWarn: null };
        Props.renderLoops(); Props.renderRoute(); Props.renderProps();
        Render.render();
      }));
  }

  function renderSummaryLegacy(res) {
    const t = res.tension;
    const rows = [];
    rows.push(['皮带总长度', res.beltLength?.toFixed(1), 'mm']);
    if (res.centerDistance != null) rows.push(['中心距', res.centerDistance.toFixed(1), 'mm']);
    rows.push(['带速', res.beltSpeed.toFixed(3), 'm/s']);
    if (t) {
      rows.push(['有效拉力 Fe', t.fe.toFixed(0), 'N']);
      rows.push(['紧边张力 F1', t.f1.toFixed(0), 'N']);
      rows.push(['松边张力 F2', t.f2.toFixed(0), 'N']);
      rows.push(['初拉力 F0 ≈', t.f0.toFixed(0), 'N']);
      rows.push(['极限张力比 e^μθ', t.ratio.toFixed(2), '']);
      rows.push(['最大可传递功率 Pmax', t.pmax.toFixed(2), 'kW']);
    }
    let html = '<div class="kvgrid">' + rows.map(r =>
      `<div>${r[0]} <b>${r[1]} ${r[2]}</b></div>`).join('') + '</div>';
    html += '<h4>各轮</h4><table class="cmp"><tr><th>轮</th><th>直径</th><th>转速 rpm</th><th>转向</th><th>包角</th></tr>';
    for (const p of res.pulleys) {
      const bad = p.wrapDeg < State.globals.minWrapDeg;
      html += `<tr><td>${esc(p.name)}</td><td>${p.diameter}</td>
        <td>${Math.abs(p.rpm).toFixed(0)}</td><td>${p.dir > 0 ? 'CW' : 'CCW'}</td>
        <td class="${bad ? 'bad' : 'good'}">${p.wrapDeg.toFixed(1)}°</td></tr>`;
    }
    html += '</table>';
    $('summary-content').innerHTML = html;
  }

  // ---------- 删除 ----------
  function deleteSelection() {
    const { type, id } = State.selection;
    if (type === 'pulley') {
      const p = findPulley(id);
      const sid = p?.shaftId;
      State.pulleys = State.pulleys.filter(x => x.id !== id);
      State.loops.forEach(lp => {
        lp.order = lp.order.filter(x => x !== id);
        Object.keys(lp.crossedEdges).forEach(k => {
          if (k.startsWith(id + '->') || k.endsWith('->' + id)) delete lp.crossedEdges[k];
        });
      });
      // 轴上再无带轮则删除空轴
      if (sid && !State.pulleys.some(x => x.shaftId === sid))
        State.shafts = State.shafts.filter(s => s.id !== sid);
    } else if (type === 'obstacle') {
      State.obstacles = State.obstacles.filter(o => o.id !== id);
    } else if (type === 'shaft') {
      if (shaftPulleys(id).length) { alert('请先把轴上的带轮归到其他轴或删除，再删除空轴。'); return; }
      State.shafts = State.shafts.filter(s => s.id !== id);
    }
    selectObj(null, null);
    persist(); refreshAll();
  }

  function lockAll() {
    // 锁定已确定的轴位（张紧轮所在轴仍可动，便于只调整其余轴和轮径试排）
    const movable = State.shafts.filter(s =>
      !s.locked && shaftPulleys(s.id).some(p => p.kind !== 'tensioner'));
    const anyUnlocked = movable.length > 0;
    State.shafts.forEach(s => {
      if (shaftPulleys(s.id).some(p => p.kind !== 'tensioner')) s.locked = anyUnlocked;
      shaftPulleys(s.id).forEach(p => { p.locked = s.locked; });
    });
    persist();
    Props.renderProps(); Props.renderLoops(); Render.render();
    $('btn-lock-all').classList.toggle('active', anyUnlocked);
  }

  // ---------- 方案库 ----------
  async function refreshSchemeList() {
    const ul = $('scheme-list');
    ul.innerHTML = '';
    const items = await API.listSchemes();
    items.forEach(s => {
      const li = document.createElement('li');
      const d = new Date(s.updated_at * 1000);
      const ver = s.version ? `<span class="ver-tag" title="方案版本">v${s.version}</span>` : '';
      li.innerHTML = `<input type="checkbox" class="cmp" title="勾选加入比较">
        <span class="nm" title="${esc(s.name)} · ${d.toLocaleString()}">${ver}${esc(s.name)}</span>
        <button class="mini act-load">载入</button>
        <button class="mini act-save">存</button>
        <button class="mini act-del" style="color:var(--err)">删</button>`;
      li.querySelector('.nm').addEventListener('click', () => li.querySelector('.act-load').click());
      li.querySelector('.act-load').addEventListener('click', async () => {
        const full = await API.loadScheme(s.id);
        applyScheme(full.data);
      });
      li.querySelector('.act-save').addEventListener('click', async () => {
        await API.updateScheme(s.id, s.name);
        flash('已覆盖保存到「' + s.name + '」（版本 +1）');
        refreshSchemeList();
      });
      li.querySelector('.act-del').addEventListener('click', async () => {
        if (!confirm('删除方案「' + s.name + '」？')) return;
        await API.deleteScheme(s.id);
        refreshSchemeList();
      });
      ul.appendChild(li);
    });
  }

  function applyScheme(data) {
    applyData(data, true);
    Render.fit();
    refreshAll();
  }

  let flashTimer = null;
  function flash(msg) {
    let el = document.getElementById('flash-msg');
    if (!el) {
      el = document.createElement('div');
      el.id = 'flash-msg';
      el.style.cssText = 'position:fixed;top:56px;left:50%;transform:translateX(-50%);' +
        'background:#2c4d74;border:1px solid var(--accent);color:#fff;padding:6px 16px;' +
        'border-radius:5px;z-index:60;font-size:13px';
      document.body.appendChild(el);
    }
    el.textContent = msg;
    el.style.display = 'block';
    clearTimeout(flashTimer);
    flashTimer = setTimeout(() => el.style.display = 'none', 2200);
  }

  // ---------- 叠加比较 ----------
  async function openCompare() {
    const lis = [...document.querySelectorAll('#scheme-list li')];
    const pickedIdx = [];
    lis.forEach((li, i) => { if (li.querySelector('input.cmp').checked) pickedIdx.push(i); });
    if (pickedIdx.length !== 2) { alert('请在方案库中勾选恰好 2 个方案进行比较。'); return; }
    const list = await API.listSchemes();
    const picked = pickedIdx.map(i => list[i]);
    if (picked.some(s => !s)) { await refreshSchemeList(); alert('方案列表已更新，请重新勾选。'); return; }
    const full = await Promise.all(picked.map(s => API.loadScheme(s.id)));
    const cmp = await API.compare(full[0].data, full[1].data);
    State.overlayScheme = { scheme: full[1].data, result: cmp.b };
    let html = `<p>比较 <b>${esc(full[0].name)}</b>（当前实线） 与
      <b>${esc(full[1].name)}</b>（画布虚线叠加）</p>
      <table class="cmp"><tr><th>指标</th><th>A：${esc(full[0].name)}</th>
      <th>B：${esc(full[1].name)}</th><th>差值 B−A</th></tr>`;
    for (const d of cmp.diffs) {
      const fmt = v => v == null ? '—' : (Math.abs(v) >= 100 ? v.toFixed(0) : v.toFixed(2));
      let cls = '';
      if (d.delta != null && ['beltLength'].includes(d.key)) cls = d.delta <= 0 ? 'good' : 'bad';
      if (d.delta != null && ['f1', 'errors', 'warns'].includes(d.key))
        cls = d.delta <= 0 ? 'good' : 'bad';
      if (d.delta != null && ['pmax', 'minWrap'].includes(d.key))
        cls = d.delta >= 0 ? 'good' : 'bad';
      html += `<tr><td>${d.label}</td><td>${fmt(d.a)}</td><td>${fmt(d.b)}</td>
        <td class="${cls}">${d.delta == null ? '—' : (d.delta > 0 ? '+' : '') + fmt(d.delta)}</td></tr>`;
    }
    html += '</table><h4>告警数</h4><div class="kvgrid">';
    for (const [tag, rr] of [['A', cmp.a], ['B', cmp.b]]) {
      const e = rr.warnings.filter(w => w.severity === 'error').length;
      const wn = rr.warnings.filter(w => w.severity === 'warning').length;
      html += `<div>${tag} 错误 <b class="${e ? 'bad' : 'good'}">${e}</b></div>
        <div>${tag} 警告 <b class="${wn ? 'bad' : 'good'}">${wn}</b></div>`;
    }
    html += '</div>';
    $('compare-body').innerHTML = html;
    $('compare-overlay').classList.remove('hidden');
    Render.render();
  }

  function esc(s) { return Props.escapeHtml(String(s ?? '')); }

  // ---------- 初始化 ----------
  function bind() {
    document.querySelectorAll('.tab').forEach(t => t.addEventListener('click', () => {
      document.querySelectorAll('.tab').forEach(x => x.classList.remove('active'));
      t.classList.add('active');
      $('tab-basis').classList.toggle('hidden', t.dataset.tab !== 'basis');
      $('tab-summary').classList.toggle('hidden', t.dataset.tab !== 'summary');
    }));
    $('btn-lock-all').addEventListener('click', lockAll);
    $('btn-add-loop').addEventListener('click', () => Loops.addLoop());
    $('btn-open-cross').addEventListener('click', () => {
      const lp = activeLoop();
      if (!lp || lp.order.length !== 2) {
        alert('开口/交叉一键切换仅适用于当前回路的两轮布置；多轮请在“带段交叉标记”中逐段设置。');
        return;
      }
      const [a, b] = lp.order, k1 = a + '->' + b, k2 = b + '->' + a;
      const crossed = lp.crossedEdges[k1] || lp.crossedEdges[k2];
      if (crossed) { delete lp.crossedEdges[k1]; delete lp.crossedEdges[k2]; }
      else { lp.crossedEdges[k1] = true; lp.crossedEdges[k2] = true; }
      persist(); refreshAll();
    });
    $('btn-fit').addEventListener('click', () => { Render.fit(); Render.render(); });
    $('btn-save').addEventListener('click', async () => {
      const name = ($('scheme-name').value || '多级方案 ' + new Date().toLocaleString()).trim();
      await API.saveScheme(name);
      $('scheme-name').value = '';
      refreshSchemeList();
      flash('已保存到 SQLite 方案库');
    });
    $('btn-new').addEventListener('click', () => {
      if (!confirm('新建空白方案？未保存内容将丢失。')) return;
      uidSeq = 1;
      const empty = { globals: { ...DEFAULT_GLOBALS }, pulleys: [], obstacles: [],
        shafts: [], loops: [] };
      applyScheme(empty);
    });
    $('btn-reset-demo').addEventListener('click', () => applyScheme(demoScheme()));
    $('btn-compare').addEventListener('click', openCompare);
    $('compare-close').addEventListener('click', () => {
      $('compare-overlay').classList.add('hidden');
      State.overlayScheme = null;
      Render.render();
    });
    $('btn-svg').addEventListener('click', () => {
      if (!State.result) return alert('尚无校核结果');
      SVG.exportSVG();
    });
    $('btn-export').addEventListener('click', SVG.exportJSON);
    $('btn-import').addEventListener('click', () => $('file-import').click());
    $('file-import').addEventListener('change', async e => {
      const f = e.target.files[0];
      if (!f) return;
      try {
        await SVG.importJSON(f);
        Render.fit(); refreshAll(); flash('已导入 JSON（共享轴与回路保留）');
      } catch (err) { alert('导入失败：' + err.message); }
      e.target.value = '';
    });
    window.addEventListener('resize', () => { Render.resize(); Render.render(); });
  }

  function init() {
    Render.resize();
    bind();
    Interact.init();
    if (!restoreAutosave()) {
      const s = demoScheme();
      applyData(s, true);
    }
    Render.fit();
    refreshAll();
    refreshSchemeList();
  }

  return { init, invalidate, refreshAll, deleteSelection, activateWarn, warnsForEdge,
    applyScheme, flash };
})();

window.addEventListener('DOMContentLoaded', App.init);
