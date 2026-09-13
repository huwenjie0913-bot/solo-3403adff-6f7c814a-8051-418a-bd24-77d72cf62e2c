/* 与 Flask 后端通信：校核、对比、方案库。校核请求带节流与过期保护。 */
'use strict';
const API = (() => {
  async function postJSON(url, body) {
    const r = await fetch(url, {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body)
    });
    if (!r.ok) throw new Error((await r.json().catch(() => ({}))).error || r.statusText);
    return r.json();
  }
  async function putJSON(url, body) {
    const r = await fetch(url, {
      method: 'PUT', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body)
    });
    if (!r.ok) throw new Error('请求失败 ' + r.status);
    return r.json();
  }

  function payload() {
    return { globals: State.globals, pulleys: State.pulleys,
      obstacles: State.obstacles, route: State.route };
  }

  let seq = 0, timer = null, inflight = null;
  function scheduleAnalyze(cb, immediate) {
    return new Promise(resolve => {
      clearTimeout(timer);
      const run = async () => {
        const my = ++seq;
        try {
          const res = await postJSON('/api/analyze', payload());
          if (my !== seq) return; // 已有更新的请求
          State.result = res;
          cb && cb(res);
        } catch (e) {
          if (my === seq) State.result = { error: e.message, warnings: [] };
        } finally { resolve(); }
      };
      timer = setTimeout(run, immediate ? 0 : 60);
    });
  }

  async function analyzeOnce(scheme) {
    return postJSON('/api/analyze', scheme || payload());
  }
  async function compare(a, b) { return postJSON('/api/compare', { a, b }); }

  async function listSchemes() {
    const r = await fetch('/api/schemes');
    return r.json();
  }
  async function loadScheme(id) {
    const r = await fetch('/api/schemes/' + id);
    if (!r.ok) throw new Error('载入失败');
    return r.json();
  }
  async function saveScheme(name) {
    return postJSON('/api/schemes', { name, data: payload() });
  }
  async function updateScheme(id, name) {
    return putJSON('/api/schemes/' + id, { name, data: payload() });
  }
  async function deleteScheme(id) {
    const r = await fetch('/api/schemes/' + id, { method: 'DELETE' });
    if (!r.ok) throw new Error('删除失败');
  }

  return { scheduleAnalyze, analyzeOnce, compare, listSchemes, loadScheme,
    saveScheme, updateScheme, deleteScheme, payload };
})();
