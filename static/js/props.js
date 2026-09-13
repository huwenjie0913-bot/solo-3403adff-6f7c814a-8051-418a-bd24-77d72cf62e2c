/* 左侧参数面板：全局参数、选中对象属性、绕行顺序。 */
'use strict';
const Props = (() => {
  const $ = id => document.getElementById(id);

  const GLOBAL_FIELDS = [
    ['inputRpm', '输入转速', 'rpm', 'number'],
    ['powerKw', '传递功率', 'kW', 'number', 0.01],
    ['friction', '摩擦系数 μ', '', 'number', 0.01],
    ['allowTension', '许用张力', 'N', 'number'],
    ['minWrapDeg', '最小包角', '°', 'number'],
    ['safetyGap', '安全间隙', 'mm', 'number'],
    ['tensionerTravel', '张紧行程', 'mm', 'number'],
    ['stretchRate', '带伸长率', '比例', 'number', 0.001],
    ['installAllowance', '安装余量', 'mm(0=自动)', 'number']
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
          State.globals[key] = key === 'stretchRate' && v > 0.5 ? v / 100 : v;
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

  function renderProps() {
    const box = $('props-form');
    const { type, id } = State.selection;
    if (type === 'pulley') {
      const p = findPulley(id);
      if (!p) { box.innerHTML = ''; return; }
      const computed = (State.result?.pulleys || []).find(x => x.id === id);
      box.innerHTML = `
        <div class="row-line"><span class="pill ${p.kind}">${KIND_NAME[p.kind]}</span>
          <b>${escapeHtml(p.id)}</b>
          <label class="row-line" style="margin-left:auto">
            <input type="checkbox" id="chk-lock" ${p.locked ? 'checked' : ''}> 锁定</label>
        </div>
        ${field('名称', txtIn('name', p.name))}
        ${field('中心 X', numIn('x', Math.round(p.x * 10) / 10), 'mm')}
        ${field('中心 Y', numIn('y', Math.round(p.y * 10) / 10), 'mm')}
        ${field('直径', numIn('diameter', p.diameter), 'mm')}
        ${p.kind === 'driver'
          ? field('转向', `<select data-key="dir">
              <option value="1" ${p.dir > 0 ? 'selected' : ''}>顺时针 CW</option>
              <option value="-1" ${p.dir < 0 ? 'selected' : ''}>逆时针 CCW</option></select>`)
          : ''}
        ${p.kind === 'driven'
          ? field('目标转速', numIn('targetRpm', p.targetRpm ?? ''), 'rpm/空')
          : ''}
        ${p.kind === 'driven'
          ? field('目标转向', `<select data-key="targetDir">
              <option value="">不限</option>
              <option value="1" ${p.targetDir === 1 ? 'selected' : ''}>CW</option>
              <option value="-1" ${p.targetDir === -1 ? 'selected' : ''}>CCW</option></select>`)
          : ''}
        <div class="kvgrid" style="margin-top:6px">
          ${computed ? `<div>转速 <b>${Math.round(computed.rpm)} rpm</b></div>
            <div>转向 <b>${computed.dir > 0 ? 'CW' : 'CCW'}</b></div>
            <div>包角 <b class="${computed.wrapDeg < State.globals.minWrapDeg ? 'tv-err' : ''}">
              ${computed.wrapDeg.toFixed(1)}°</b></div>` : ''}
        </div>
        <div class="btn-row">
          ${['driven', 'idler', 'tensioner', 'driver'].filter(k => k !== p.kind)
            .map(k => `<button class="mini" data-kind="${k}">改为${KIND_NAME[k]}</button>`).join('')}
          <button class="mini" data-del="1" style="margin-left:auto;color:var(--err)">删除</button>
        </div>`;
      bindInputs(box, p, () => App.invalidate());
      box.querySelector('#chk-lock').addEventListener('change', e => {
        p.locked = e.target.checked; persist(); App.invalidate();
      });
      box.querySelectorAll('button[data-kind]').forEach(b => b.addEventListener('click', () => {
        // 全局只允许一个主动轮
        if (b.dataset.kind === 'driver')
          State.pulleys.forEach(q => { if (q.kind === 'driver') q.kind = 'driven'; });
        p.kind = b.dataset.kind;
        if (p.kind === 'driver' && !('dir' in p)) p.dir = 1;
        persist(); App.refreshAll();
      }));
      box.querySelector('button[data-del]').addEventListener('click', () => {
        App.deleteSelection();
      });
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
      box.innerHTML = '<p class="hint">选中画布上的轮或护罩后在此编辑。</p>';
    }
  }

  function renderRoute() {
    const ol = $('route-list');
    ol.innerHTML = '';
    State.route.order.forEach((id, i) => {
      const p = findPulley(id);
      if (!p) return;
      const li = document.createElement('li');
      if (State.selection.type === 'pulley' && State.selection.id === id) li.classList.add('sel');
      li.draggable = true;
      li.innerHTML = `<span>${i + 1}. ${escapeHtml(p.name)}
        <span class="kind-tag">${KIND_NAME[p.kind]} · ⌀${p.diameter}</span></span>
        <span>${p.locked ? '🔒' : ''}</span>`;
      li.addEventListener('click', () => { selectObj('pulley', id); App.refreshAll(); });
      li.addEventListener('dragstart', () => li.classList.add('drag'));
      li.addEventListener('dragend', () => li.classList.remove('drag'));
      li.addEventListener('dragover', e => e.preventDefault());
      li.addEventListener('drop', e => {
        e.preventDefault();
        const from = State.route.order.indexOf(State.selection.id);
        // HTML5 drag：用被拖 li 的索引（拖拽开始时记录）
        const draggedId = State.route._dragId;
        if (draggedId == null) return;
        const a = State.route.order.indexOf(draggedId);
        reorder(a, i);
      });
      li.addEventListener('dragstart', () => { State.route._dragId = id; });
      ol.appendChild(li);
    });

    // 交叉标记列表
    const eb = $('edge-cross-list');
    eb.innerHTML = '';
    const order = State.route.order;
    for (let i = 0; i < order.length; i++) {
      const a = order[i], b = order[(i + 1) % order.length];
      const pa = findPulley(a), pb = findPulley(b);
      if (!pa || !pb) continue;
      const key = a + '->' + b;
      const lab = document.createElement('label');
      lab.innerHTML = `<input type="checkbox" ${State.route.crossedEdges[key] ? 'checked' : ''}>
        ${escapeHtml(pa.name)} → ${escapeHtml(pb.name)}（交叉）`;
      lab.querySelector('input').addEventListener('change', e => {
        if (e.target.checked) State.route.crossedEdges[key] = true;
        else delete State.route.crossedEdges[key];
        persist(); App.refreshAll();
      });
      eb.appendChild(lab);
    }
    $('mode-label').textContent = isTwoWheelCrossed() ? '交叉带' : '开口带';
  }

  function reorder(from, to) {
    if (from < 0 || from === to) return;
    const arr = State.route.order;
    const oldEdges = [];
    for (let i = 0; i < arr.length; i++)
      oldEdges.push([arr[i], arr[(i + 1) % arr.length]]);
    const oldCross = oldEdges.map(([a, b]) =>
      !!State.route.crossedEdges[a + '->' + b]);
    const [x] = arr.splice(from, 1);
    arr.splice(to, 0, x);
    // 按新顺序重建交叉标记：移动的轮与其前后边沿用旧标记
    State.route.crossedEdges = {};
    for (let i = 0; i < arr.length; i++) {
      const pair = [arr[i], arr[(i + 1) % arr.length]];
      const oldIdx = oldEdges.findIndex(([a, b]) => a === pair[0] && b === pair[1]);
      if (oldIdx >= 0 && oldCross[oldIdx])
        State.route.crossedEdges[pair[0] + '->' + pair[1]] = true;
    }
    persist(); App.refreshAll();
  }

  function isTwoWheelCrossed() {
    const o = State.route.order;
    return o.length === 2 &&
      Object.keys(State.route.crossedEdges).length > 0;
  }

  function toggleTwoWheelMode() {
    const o = State.route.order;
    if (o.length !== 2) { alert('开口/交叉一键切换仅适用于两轮布置；多轮请在“带段交叉标记”中逐段设置。'); return; }
    const k1 = o[0] + '->' + o[1], k2 = o[1] + '->' + o[0];
    if (isTwoWheelCrossed()) {
      delete State.route.crossedEdges[k1];
      delete State.route.crossedEdges[k2];
    } else {
      State.route.crossedEdges[k1] = true;
      State.route.crossedEdges[k2] = true;
    }
    persist(); App.refreshAll();
  }

  function escapeHtml(s) {
    return String(s).replace(/[&<>"']/g, c =>
      ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
  }

  return { renderGlobal, renderProps, renderRoute, toggleTwoWheelMode,
    isTwoWheelCrossed, escapeHtml };
})();
