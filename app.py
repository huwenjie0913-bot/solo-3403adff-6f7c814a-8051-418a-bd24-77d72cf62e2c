"""Flask 入口：几何计算、方案存取、方案对比。"""
import glob
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# 受限环境（系统 Python 无 pip / 无 Flask）下，自动加载仓库内 .pylibs 中的依赖。
# 标准环境请优先：python3 -m pip install -r requirements.txt
try:
    import flask  # noqa: F401
except ImportError:
    _here = os.path.dirname(os.path.abspath(__file__))
    _candidates = sorted(glob.glob(os.path.join(_here, ".pylibs", "lib", "python*",
                                               "site-packages")))
    if _candidates:
        sys.path.insert(0, _candidates[-1])

from flask import Flask, jsonify, render_template, request  # noqa: E402

from belt import analyzer, store  # noqa: E402

app = Flask(__name__, static_folder="static", template_folder="templates")
store.init_db()


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/analyze", methods=["POST"])
def api_analyze():
    scheme = request.get_json(force=True, silent=True)
    if not isinstance(scheme, dict):
        return jsonify({"error": "请求体必须是方案 JSON 对象"}), 400
    try:
        return jsonify(analyzer.analyze(scheme))
    except KeyError as exc:
        return jsonify({"error": "方案缺少字段：%s" % exc}), 400
    except ZeroDivisionError:
        return jsonify({"error": "几何退化（轮径为 0 或轮心重合）"}), 400


@app.route("/api/compare", methods=["POST"])
def api_compare():
    payload = request.get_json(force=True, silent=True)
    if not isinstance(payload, dict) or "a" not in payload or "b" not in payload:
        return jsonify({"error": "需要 {a: 方案, b: 方案}"}), 400
    return jsonify(analyzer.compare(payload["a"], payload["b"]))


@app.route("/api/schemes")
def api_list():
    return jsonify(store.list_schemes())


@app.route("/api/schemes", methods=["POST"])
def api_create():
    payload = request.get_json(force=True, silent=True) or {}
    name = (payload.get("name") or "未命名方案").strip()[:80]
    data = payload.get("data")
    if not isinstance(data, dict):
        return jsonify({"error": "data 必须是方案对象"}), 400
    sid = store.create_scheme(name, data)
    return jsonify({"id": sid}), 201


@app.route("/api/schemes/<int:sid>")
def api_get(sid):
    s = store.get_scheme(sid)
    if not s:
        return jsonify({"error": "方案不存在"}), 404
    return jsonify(s)


@app.route("/api/schemes/<int:sid>", methods=["PUT"])
def api_update(sid):
    payload = request.get_json(force=True, silent=True) or {}
    name = (payload.get("name") or "未命名方案").strip()[:80]
    data = payload.get("data")
    if not isinstance(data, dict):
        return jsonify({"error": "data 必须是方案对象"}), 400
    if not store.update_scheme(sid, name, data):
        return jsonify({"error": "方案不存在"}), 404
    return jsonify({"ok": True})


@app.route("/api/schemes/<int:sid>", methods=["DELETE"])
def api_delete(sid):
    if not store.delete_scheme(sid):
        return jsonify({"error": "方案不存在"}), 404
    return jsonify({"ok": True})


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=int(os.environ.get("PORT", 5000)), debug=False)
