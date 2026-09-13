/* 左侧参数面板：全局参数、多级回路、选中对象属性、当前回路绕行顺序。 */
'use strict';
const Props = (() => {
  const $ = id => document.getElementById(id);

  const GLOBAL_FIELDS = [
    ['inputRpm', '电机转速', 'rpm', 'number'],
    ['powerKw', '默认功率', 'kW', 'number', 0.01],
    ['efficiency', '默认效率 η', '', 'number', 0.01],
    ['friction', '摩擦系数 μ', '', 'number', 0.01],
    ['allowTension', '许用张力', 'N', 'number'],
    ['minWrapDeg', '最小包角', '°', 'number'],
    ['safetyGap', '安全间隙', 'mm', 'number'],
    ['tensionerTravel', '张紧行程', 'mm', 'number'],
    ['stretchRate', '带伸长率', '比例', 'number', 0.001]
  ];

  function renderGlobal() {
    const box = $('global-form');
    box.innerHTML = '';
    for (const [key, label, unit, type, step] of GLOBAL_FIELDS) {
      const row = document.createElement('div');
      row.className = 'fld';
      row.innerHTML = `<label>${label}</label>
        <input type="${type}" step="${step || 'any'}" value="${State.globals[key] ?? ''}">
        <span class="u">${unit}</span>`;
      const inp = row.querySelector('input');
      inp.addEventListener('input', () => {
        const v = parseFloat(inp.value);
        if (!Number.isNaN(v)) {
          State.globals[key] = key === 'stretchRate' && v > 0.5 ? v / 100
            : key === 'efficiency' && v > 1 ? v / 100 : v;
          State.dirty = true; persist();
          App.invalidate();
        }
      });
      box.appendChild(row);
    }
  }

  function field(label, html, unit) {
    return `<div class="fld"><label>${label}</label>${html}
            <span class="u">${unit || ''}</span></div>`;
  }
  const numIn = (key, val, step) =>
    `<input data-key="${key}" type="number" step="${step || 'any'}" value="${val ?? ''}">`;
  const txtIn = (key, val) =>
    `<input data-key="${key}" type="text" value="${val ?? ''}">`;

  function bindInputs(box, obj, after) {
    box.querySelectorAll('input[data-key],select[data-key]').forEach(el => {
      const key = el.dataset.key;
      if (String(key).startsWith('__') || key === 'shaftId') return; // 特殊绑定
      el.addEventListener('input', () => {
        const k = el.dataset.key;
        if (el.type === 'checkbox') obj[k] = el.checked;
        else if (el.type === 'number') {
          const v = parseFloat(el.value);
          if (!Number.isNaN(v)) obj[k] = v;
        } else obj[k] = el.value;
        State.dirty = true; persist(); after && after();
      });
    });
  }

  const KIND_NAME = { driver: '主动轮', driven: '从动轮', idler: '惰轮', tensioner: '张紧轮' };

  // ---------- 多级回路面板 ----------
  function renderLoops() {
    const box = $('loops-form');
    box.innerHTML = '';
    const badge = $('loop-count');
    badge.textContent = State.loops.length;
    badge.className = 'badge ' + (State.loops.length > 1 ? 'ok' : '');

    State.loops.forEach((lp, i) => {
      const col = loopColor(i);
      const active = lp.id === State.activeLoopId;
      const rl = resultLoop(lp.id);
      const div = document.createElement('div');
      div.className = 'loop-card' + (active ? ' active' : '');
      div.style.borderLeftColor = col;
      const ok = rl ? rl.ok : null;
      div.innerHTML = `
        <div class="loop-head">
          <input type="radio" name="active-loop" ${active ? 'checked' : ''} title="切换当前编辑回路">
          <span class="loop-swatch" style="background:${col}"></span>
          <input data-key="name" type="text" value="${escapeHtml(lp.name)}" class="loop-name">
          <label class="loop-input" title="勾选：该回路主动轮轴为电机输入轴">
            <input type="checkbox" data-key="isInput" ${lp.isInput ? 'checked' : ''}> 输入</label>
          <button class="mini act-del-loop" style="color:var(--err)">删</button>
        </div>
        <div class="loop-grid">
          <label>功率 <input type="number" step="0.01" data-key="powerKw" value="${lp.powerKw ?? ''}"> kW</label>
          <label>效率 <input type="number" step="0.01" data-key="efficiency" value="${lp.efficiency ?? 0.96}"></label>
          ${lp.isInput ? `<label>转速 <input type="number" data-key="inputRpm"
             value="${lp.inputRpm ?? State.globals.inputRpm}"> rpm</label>` : ''}
        </div>
        <details class="loop-adv"><summary>高级（μ/许用张力/包角/张紧）</summary>
          <div class="loop-grid">
            <label>μ <input type="number" step="0.01" data-key="friction"
              value="${lp.friction ?? State.globals.friction}"></label>
            <label>许用张力 <input type="number" data-key="allowTension"
              value="${lp.allowTension ?? State.globals.allowTension}"> N</label>
            <label>最小包角 <input type="number" data-key="minWrapDeg"
              value="${lp.minWrapDeg ?? State.globals.minWrapDeg}">°</label>
            <label>张紧行程 <input type="number" data-key="tensionerTravel"
              value="${lp.tensionerTravel ?? State.globals.tensionerTravel}"> mm</label>
          </div>
        </details>
        <div class="loop-stats">
          <span class="${ok === false ? 'tv-err' : ''}">${ok == null ? '—' : ok ? '✓' : '✗ 几何/绕向错误'}</span>
          ${rl ? `n入 ${Math.round(rl.inputRpm)} rpm · v ${rl.beltSpeed.toFixed(2)} m/s · L ${rl.length.toFixed(0)} mm` : ''}
        </div>`;
      // 切换当前回路
      div.querySelector('input[name="active-loop"]').addEventListener('change', () => {
        State.activeLoopId = lp.id; persist(); App.refreshAll();
      });
      div.querySelector('.loop-head').addEventListener('click', e => {
        if (e.target.tagName === 'INPUT' || e.target.tagName === 'BUTTON') return;
        State.activeLoopId = lp.id; persist(); App.refreshAll();
      });
      bindInputs(div, lp, () => {
        if (!lp.inputRpm) lp.inputRpm = State.globals.inputRpm;
        App.invalidate();
      });
      div.querySelector('.act-del-loop').addEventListener('click', () => {
        if (State.loops.length <= 1) return alert('至少保留一条回路');
        if (!confirm('删除回路「' + lp.name + '」？其中的带轮不会被删除，但将不参与校核。')) return;
        Loops.removeLoop(lp.id);
      });
      box.appendChild(div);
    });

    const addBtn = document.createElement('button');
    addBtn.className = 'mini';
    addBtn.style.cssText = 'margin-top:6px';
    addBtn.textContent = '＋ 新建回路';
    addBtn.addEventListener('click', () => Loops.addLoop());
    box.appendChild(addBtn);
  }

  // ---------- 选中对象属性 ----------
  function renderProps() {
    const box = $('props-form');
    const { type, id } = State.selection;
    if (type === 'pulley') {
      const p = findPulley(id);
      if (!p) { box.innerHTML = ''; return; }
      const shaft = findShaft(p.shaftId);
      const computed = resultPulley(id);
      const siblings = shaftPulleys(p.shaftId).filter(q => q.id !== p.id);
      const otherShafts = State.shafts.filter(s => s.id !== p.shaftId);
      box.innerHTML = `
        <div class="row-line"><span class="pill ${p.kind}"
          style="border-color:${loopColor(pulleyLoop(p.id)?.id || 0)}">${KIND_NAME[p.kind]}</span>
          <b>${escapeHtml(p.id)}</b>
          <label class="row-line" style="margin-left:auto">
            <input type="checkbox" id="chk-lock" ${shaft?.locked ? 'checked' : ''}> 锁轴</label>
        </div>
        ${field('名称', txtIn('name', p.name))}
        ${field('直径', numIn('diameter', p.diameter), 'mm')}
        ${field('所属轴', `<select data-key="shaftId">
          ${State.shafts.map(s => `<option value="${s.id}" ${s.id === p.shaftId ? 'selected' : ''}>
            ${escapeHtml(s.name)} (${s.id})</option>`).join('')}</select>
          <button class="mini" id="btn-new-shaft" title="拆到一根新轴">拆新轴</button>`)}
        ${siblings.length ? `<div class="hint">同轴带轮：${siblings.map(q =>
          `<span class="co-axis">${escapeHtml(q.name)} ⌀${q.diameter}</span>`).join(' ')}</div>` : ''}
        ${otherShafts.length ? `<div class="btn-row" id="merge-shaft-row">
          <span class="hint" style="margin:0">归并到：</span>
          ${otherShafts.map(s =>
            `<button class="mini" data-merge="${s.id}" title="移动到 ${escapeHtml(s.name)}">${escapeHtml(s.name)}</button>`)
            .join('')}</div>` : ''}
        ${field('轴名称', txtIn('__shaftName', shaft?.name), '')}
        ${field('轴 X', numIn('__sx', Math.round(shaft?.x ?? 0)), 'mm')}
        ${field('轴 Y', numIn('__sy', Math.round(shaft?.y ?? 0)), 'mm')}
        ${field('许用扭矩', numIn('allowTorque', shaft?.allowTorque || 0), 'N·m')}
        ${field('许用径向', numIn('allowRadial', shaft?.allowRadial || 0), 'N')}
        ${p.kind === 'driver'
          ? field('转向', `<select data-key="dir">
              <option value="1" ${p.dir > 0 ? 'selected' : ''}>顺时针 CW</option>
              <option value="-1" ${p.dir < 0 ? 'selected' : ''}>逆时针 CCW</option></select>`)
          : ''}
        ${p.kind === 'driven'
          ? field('目标转速', numIn('targetRpm', p.targetRpm ?? ''), 'rpm/空') : ''}
        ${p.kind === 'driven'
          ? field('目标转向', `<select data-key="targetDir">
              <option value="">不限</option>
              <option value="1" ${p.targetDir === 1 ? 'selected' : ''}>CW</option>
              <option value="-1" ${p.targetDir === -1 ? 'selected' : ''}>CCW</option></select>`)
          : ''}
        ${shaftCard(shaft)}
        <div class="kvgrid" style="margin-top:6px">
          ${computed ? `<div>转速 <b>${Math.round(computed.rpm)} rpm</b></div>
            <div>转向 <b>${computed.dir > 0 ? 'CW' : 'CCW'}</b></div>
            <div>包角 <b class="${computed.wrapDeg < (pulleyLoop(p.id)
              && Number(pulleyLoop(p.id).minWrapDeg) || State.globals.minWrapDeg) ? 'tv-err' : ''}">
              ${computed.wrapDeg.toFixed(1)}°</b></div>` : ''}
        </div>
        <div class="btn-row">
          ${['driven', 'idler', 'tensioner', 'driver'].filter(k => k !== p.kind)
            .map(k => `<button class="mini" data-kind="${k}">改为${KIND_NAME[k]}</button>`).join('')}
          <button class="mini" data-del="1" style="margin-left:auto;color:var(--err)">删除</button>
        </div>`;

      bindInputs(box, p, () => App.invalidate());
      box.querySelector('select[data-key=shaftId]').addEventListener('change', e => {
        Loops.movePulleyToShaft(p.id, e.target.value);
        App.refreshAll();
      });
      box.querySelector('#btn-new-shaft')?.addEventListener('click', () => {
        Loops.splitPulleyShaft(p.id); App.refreshAll();
      });
      box.querySelectorAll('button[data-merge]').forEach(b => b.addEventListener('click', () => {
        Loops.mergeShaft(p.shaftId, b.dataset.merge); App.refreshAll();
      }));
      // 轴字段直接写轴对象
      const sh = shaft;
      const bindSh = (key, k) => {
        const el = box.querySelector(`[data-key="${key}"]`);
        el?.addEventListener('input', () => {
          const v = parseFloat(el.value);
          if (!Number.isNaN(v)) sh[k] = v;
          if (k === 'x' || k === 'y') {
            shaftPulleys(sh.id).forEach(q => { q.x = sh.x; q.y = sh.y; });
          }
          persist(); App.invalidate();
        });
      };
      box.querySelector('[data-key=__shaftName]')?.addEventListener('input', e => {
        sh.name = e.target.value; persist(); Render.render();
      });
      bindSh('__sx', 'x'); bindSh('__sy', 'y');
      bindSh('allowTorque', 'allowTorque'); bindSh('allowRadial', 'allowRadial');

      box.querySelector('#chk-lock').addEventListener('change', e => {
        sh.locked = e.target.checked;
        shaftPulleys(sh.id).forEach(q => { q.locked = sh.locked; });
        persist(); App.invalidate();
      });
      box.querySelectorAll('button[data-kind]').forEach(b => b.addEventListener('click', () => {
        Loops.changeKind(p.id, b.dataset.kind);
      }));
      box.querySelector('button[data-del]').addEventListener('click', () => App.deleteSelection());
    } else if (type === 'shaft') {
      const s = findShaft(id);
      if (s) box.innerHTML = shaftPropsHtml(s);
      bindShaftProps(box, s);
    } else if (type === 'obstacle') {
      const o = findObstacle(id);
      if (!o) { box.innerHTML = ''; return; }
      box.innerHTML = `
        <div class="row-line"><span class="pill obstacle">矩形护罩</span><b>${escapeHtml(o.id)}</b></div>
        ${field('名称', txtIn('name', o.name))}
        ${field('中心 X', numIn('x', Math.round(o.x)), 'mm')}
        ${field('中心 Y', numIn('y', Math.round(o.y)), 'mm')}
        ${field('宽度', numIn('w', o.w), 'mm')}
        ${field('高度', numIn('h', o.h), 'mm')}
        ${field('旋转角', numIn('rot', o.rot || 0), '°')}
        <div class="btn-row"><button class="mini" data-del="1" style="color:var(--err)">删除护罩</button></div>`;
      bindInputs(box, o, () => App.invalidate());
      box.querySelector('button[data-del]').addEventListener('click', () => App.deleteSelection());
    } else {
      box.innerHTML = '<p class="hint">选中画布上的轮或轴后在此编辑。两个轮归到同一根轴即共享转速/扭矩。</p>';
    }
  }

  function shaftCard(shaft) {
    if (!shaft) return '';
    const rs = resultShaft(shaft.id);
    if (!rs) return '';
    const tBad = shaft.allowTorque > 0 && rs.torque > shaft.allowTorque;
    const fBad = shaft.allowRadial > 0 && rs.radial > shaft.allowRadial;
    return `<div class="shaft-card">
      <div>转速 <b>${rs.rpm == null ? '—' : Math.round(rs.rpm) + ' rpm ' + (rs.dir > 0 ? 'CW' : 'CCW')}</b></div>
      <div>扭矩 <b class="${tBad ? 'tv-err' : ''}">${rs.torque.toFixed(1)}${shaft.allowTorque ? ' / ' + shaft.allowTorque : ''} N·m</b></div>
      <div>径向合力 <b class="${fBad ? 'tv-err' : ''}">${rs.radial.toFixed(0)}${shaft.allowRadial ? ' / ' + shaft.allowRadial : ''} N</b></div>
      <div>关联回路 ${rs.loops.map(lid =>
        `<span class="loop-chip" style="color:${loopColor(lid)}">${escapeHtml((State.loops.find(l => l.id === lid) || {}).name || lid)}</span>`).join(' ')}</div>
    </div>`;
  }

  function shaftPropsHtml(s) {
    const rs = resultShaft(s.id);
    const ps = shaftPulleys(s.id);
    return `
      <div class="row-line"><span class="pill" style="color:#8fc4ff;border-color:#3b6ea5">共享轴</span>
        <b>${escapeHtml(s.id)}</b>
        <label class="row-line" style="margin-left:auto">
          <input type="checkbox" id="chk-lock" ${s.locked ? 'checked' : ''}> 锁定轴位</label>
      </div>
      ${field('轴名称', txtIn('name', s.name))}
      ${field('中心 X', numIn('x', Math.round(s.x)), 'mm')}
      ${field('中心 Y', numIn('y', Math.round(s.y)), 'mm')}
      ${field('许用扭矩', numIn('allowTorque', s.allowTorque || 0), 'N·m')}
      ${field('许用径向', numIn('allowRadial', s.allowRadial || 0), 'N')}
      ${shaftCard(s)}
      <div class="hint">轴上带轮（${ps.length}）：</div>
      <div class="btn-row" style="flex-wrap:wrap">
        ${ps.map(q => `<button class="mini" data-pulley="${q.id}">${escapeHtml(q.name)}</button>`).join('')}
      </div>
      <div class="btn-row"><button class="mini" data-del="1" style="color:var(--err)">删除空轴</button></div>`;
  }

  function bindShaftProps(box, s) {
    if (!s) return;
    bindInputs(box, s, () => {
      shaftPulleys(s.id).forEach(p => { p.x = s.x; p.y = s.y; });
      App.invalidate();
    });
    box.querySelector('#chk-lock')?.addEventListener('change', e => {
      s.locked = e.target.checked;
      shaftPulleys(s.id).forEach(p => { p.locked = s.locked; });
      persist(); App.invalidate();
    });
    box.querySelectorAll('button[data-pulley]').forEach(b => b.addEventListener('click', () => {
      selectObj('pulley', b.dataset.pulley); App.refreshAll();
    }));
    box.querySelector('button[data-del]')?.addEventListener('click', () => {
      if (shaftPulleys(s.id).length) return alert('轴上仍有带轮，请先把它们归到其他轴。');
      State.shafts = State.shafts.filter(x => x.id !== s.id);
      selectObj(null, null); persist(); App.refreshAll();
    });
  }

  // ---------- 当前回路绕行顺序 ----------
  function renderRoute() {
    const lp = activeLoop();
    const ol = $('route-list');
    const eb = $('edge-cross-list');
    ol.innerHTML = '';
    eb.innerHTML = '';
    $('route-loop-name').textContent = lp ? lp.name : '—';
    $('route-loop-name').style.color = lp ? loopColor(State.loops.indexOf(lp)) : 'var(--accent)';
    if (!lp) return;

    lp.order.forEach((id, i) => {
      const p = findPulley(id);
      if (!p) return;
      const li = document.createElement('li');
      li.style.borderLeft = `3px solid ${loopColor(pulleyLoop(p.id)?.id || lp.id)}`;
      li.style.paddingLeft = '5px';
      if (State.selection.type === 'pulley' && State.selection.id === id) li.classList.add('sel');
      li.draggable = true;
      li.innerHTML = `<span>${i + 1}. ${escapeHtml(p.name)}
        <span class="kind-tag">${KIND_NAME[p.kind]} · ⌀${p.diameter} · ${escapeHtml(findShaft(p.shaftId)?.name || '')}</span></span>
        <span><button class="mini act-rm" title="移出当前回路（轮保留）">出</button></span>`;
      li.addEventListener('click', e => {
        if (e.target.classList.contains('act-rm')) return;
        selectObj('pulley', id); App.refreshAll();
      });
      li.querySelector('.act-rm').addEventListener('click', () => Loops.removeFromLoop(lp.id, id));
      li.addEventListener('dragover', e => e.preventDefault());
      li.addEventListener('drop', e => {
        e.preventDefault();
        const draggedId = lp._dragId;
        if (draggedId == null) return;
        const a = lp.order.indexOf(draggedId);
        reorder(lp, a, i);
      });
      li.addEventListener('dragstart', () => { lp._dragId = id; li.classList.add('drag'); });
      li.addEventListener('dragend', () => li.classList.remove('drag'));
      ol.appendChild(li);
    });

    // 不在当前回路的轮：可加入
    const outsiders = State.pulleys.filter(p => !lp.order.includes(p.id)
      && !pulleyLoop(p.id));
    if (outsiders.length) {
      const li = document.createElement('li');
      li.className = 'add-pulley-row';
      li.innerHTML = '<span class="hint" style="margin:0">加入未绕入轮：</span>' +
        outsiders.map(p => `<button class="mini" data-add="${p.id}">${escapeHtml(p.name)}</button>`)
          .join(' ');
      li.querySelectorAll('button[data-add]').forEach(b => b.addEventListener('click', () => {
        Loops.addToLoop(lp.id, b.dataset.add);
      }));
      ol.appendChild(li);
    }

    // 交叉标记
    const order = lp.order;
    for (let i = 0; i < order.length; i++) {
      const a = order[i], b = order[(i + 1) % order.length];
      const pa = findPulley(a), pb = findPulley(b);
      if (!pa || !pb) continue;
      const key = a + '->' + b;
      const lab = document.createElement('label');
      lab.innerHTML = `<input type="checkbox" ${lp.crossedEdges[key] ? 'checked' : ''}>
        ${escapeHtml(pa.name)} → ${escapeHtml(pb.name)}（交叉）`;
      lab.querySelector('input').addEventListener('change', e => {
        if (e.target.checked) lp.crossedEdges[key] = true;
        else delete lp.crossedEdges[key];
        persist(); App.refreshAll();
      });
      eb.appendChild(lab);
    }
    const ml = $('mode-label');
    if (ml) {
      const crossed = order.length === 2 &&
        Object.keys(lp.crossedEdges || {}).length > 0;
      ml.textContent = crossed ? '交叉带' : '开口带';
    }
  }

  function reorder(lp, from, to) {
    if (from < 0 || from === to) return;
    const arr = lp.order;
    const oldEdges = [];
    for (let i = 0; i < arr.length; i++)
      oldEdges.push([arr[i], arr[(i + 1) % arr.length]]);
    const oldCross = oldEdges.map(([a, b]) => !!lp.crossedEdges[a + '->' + b]);
    const [x] = arr.splice(from, 1);
    arr.splice(to, 0, x);
    lp.crossedEdges = {};
    for (let i = 0; i < arr.length; i++) {
      const pair = [arr[i], arr[(i + 1) % arr.length]];
      const oldIdx = oldEdges.findIndex(([a, b]) => a === pair[0] && b === pair[1]);
      if (oldIdx >= 0 && oldCross[oldIdx])
        lp.crossedEdges[pair[0] + '->' + pair[1]] = true;
    }
    persist(); App.refreshAll();
  }

  function escapeHtml(s) {
    return String(s).replace(/[&<>"']/g, c =>
      ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
  }

  return { renderGlobal, renderLoops, renderProps, renderRoute, escapeHtml };
})();
