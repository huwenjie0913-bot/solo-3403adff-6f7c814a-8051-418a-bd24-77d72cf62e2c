"""皮带传动布置校核引擎。

输入（与前端 state、SQLite 中保存的 JSON 完全一致）：
{
  "globals": {
    "inputRpm": 主动轮转速 rpm,
    "powerKw": 传递功率 kW,
    "friction": 摩擦系数 μ,
    "allowTension": 许用张力 N,
    "minWrapDeg": 最小包角 °,
    "safetyGap": 安全间隙 mm,
    "tensionerTravel": 张紧行程 mm,
    "stretchRate": 带伸长率（占带长比例，默认 0.01）,
    "installAllowance": 安装余量 mm（默认取中心距 0.5%，面板可覆盖）
  },
  "pulleys": [{id,name,kind(driver/driven/idler/tensioner),x,y,diameter,locked,
               targetRpm,targetDir}],
  "obstacles": [{id,name,x,y,w,h,rot}],
  "route": {"order": [id...], "crossedEdges": {"idA->idB": true}}
}
"""
import math

from . import geometry as g

TAN_TOL = 0.99  # 切点速度与带段方向的最低一致性


def _err(msg, refs=None):
    return {"severity": "error", "message": msg, "refs": refs or {}}


def _wheel(pulleys, wid):
    for p in pulleys:
        if p["id"] == wid:
            return p
    return None


def build_paths(scheme, displacements=None, pulleys=None, route_order=None,
                crossed_map=None, driver_id=None, driver_dir=None):
    """按绕行顺序构建有向带段与轮上弧段。

    返回 dict：ok, order, wheels(id 顺序), edges[{from,to,crossed,p1,p2,q,feasible,
    score,length}], arcs[{wheel,a0,sweep,dir,r,length,wrapDeg}], dirs, length,
    fatal（无法构造切线的错误列表）
    displacements: {id: (x,y)} 临时移动轮位（张紧行程试算用）
    pulleys/route_order/crossed_map：多级模式下只构建某一条回路时的覆盖参数
    driver_id/driver_dir：指定主动轮及其绝对转向（多级模式下转向由上游轴决定）
    """
    displacements = displacements or {}
    pulleys = pulleys or scheme["pulleys"]
    route_def = scheme.get("route", {}) or {}
    if route_order is None:
        route_order = route_def.get("order") or [p["id"] for p in pulleys]
    order = list(route_order)
    if crossed_map is None:
        crossed_map = route_def.get("crossedEdges", {}) or {}
    fatal = []

    pmap = {p["id"]: p for p in pulleys}
    missing = [i for i in order if i not in pmap]
    if missing:
        return {"ok": False, "fatal": [_err("绕行顺序引用了不存在的轮：%s" % ",".join(missing))]}
    n = len(order)
    if n < 2:
        return {"ok": False, "fatal": [_err("至少需要 2 个轮才能构成带传动")]}

    def pos(wid):
        return displacements.get(wid, (pmap[wid]["x"], pmap[wid]["y"]))

    def rad(wid):
        return pmap[wid]["diameter"] / 2.0

    # 找主动轮作为转向传播起点
    if driver_id is None:
        driver_id = next((p["id"] for p in pulleys if p.get("kind") == "driver"), order[0])
    if driver_dir is None:
        driver_dir = 1 if pmap[driver_id].get("dir", 1) >= 0 else -1
    else:
        driver_dir = 1 if driver_dir >= 0 else -1
    if driver_id not in order:
        return {"ok": False, "fatal": [_err("主动轮不在绕行顺序中")]}

    edges_def = []
    for i in range(n):
        a, b = order[i], order[(i + 1) % n]
        edges_def.append({"from": a, "to": b, "crossed": bool(crossed_map.get("%s->%s" % (a, b)))})

    # 转向沿环传播：交叉边使从动轮反向；闭环要求反向次数为偶数
    dirs = {driver_id: driver_dir}
    start = order.index(driver_id)
    flips = 0
    for k in range(1, n + 1):
        i = (start + k) % n
        prev_i = (i - 1) % n
        e = edges_def[prev_i]
        flip = -1 if e["crossed"] else 1
        flips += 1 if e["crossed"] else 0
        if k < n:
            dirs[order[i]] = dirs[order[prev_i]] * flip
    closure_ok = (flips % 2 == 0)

    # 为每条边选公切线（速度方向必须与带段行进方向一致）
    edges = []
    all_ok = True
    for e in edges_def:
        a, b = e["from"], e["to"]
        ca, cb = pos(a), pos(b)
        ra, rb = rad(a), rad(b)
        if g.dist(ca, cb) < 1e-6:
            fatal.append(_err("%s 与 %s 轮心重合，无法布置带段" % (a, b),
                              {"pulleys": [a, b], "edges": [len(edges)]}))
            edges.append({**e, "p1": ca, "p2": cb, "q": 1, "feasible": False,
                          "score": -1, "length": 0.0})
            all_ok = False
            continue
        cands = g.common_tangents(ca, ra, cb, rb, e["crossed"])
        best = None
        for (p1, p2), q in cands:
            va = g.norm(g.vel_at(ca, ra, p1, dirs[a]))
            vb = g.norm(g.vel_at(cb, rb, p2, dirs[b]))
            s = g.norm(g.sub(p2, p1))
            score = min(g.dot(va, s), g.dot(vb, s))
            if best is None or score > best["score"]:
                best = {"p1": p1, "p2": p2, "q": q, "score": score}
        if best is None:
            best = {"p1": polar_h(ca, ra, cb), "p2": polar_h(cb, rb, ca),
                    "q": 1, "score": -2}
        feasible = best["score"] >= TAN_TOL
        if not feasible:
            all_ok = False
            fatal.append(_err(
                "%s→%s 的%s公切线与轮面速度方向不一致（转向冲突或轮径/位置不合理）"
                % (a, b, "内" if e["crossed"] else "外"),
                {"pulleys": [a, b], "edges": [len(edges)]}))
        edges.append({**e, "p1": best["p1"], "p2": best["p2"], "q": best["q"],
                      "feasible": feasible, "score": round(best["score"], 4),
                      "length": g.dist(best["p1"], best["p2"]) if feasible else 0.0})

    # 轮上弧段：入切点 -> 沿转向 -> 出切点
    arcs = {}
    for i, wid in enumerate(order):
        c = pos(wid)
        r = rad(wid)
        pin = edges[i - 1]["p2"]
        pout = edges[i]["p1"]
        a0 = g.ang(g.sub(pin, c))
        a1 = g.ang(g.sub(pout, c))
        if dirs[wid] > 0:
            sweep = g.cw_sweep(a0, a1)
        else:
            sweep = g.TWO_PI - g.cw_sweep(a0, a1)
        arcs[wid] = {"wheel": wid, "center": c, "a0": a0, "a1": a1,
                     "sweep": sweep, "dir": dirs[wid], "r": r,
                     "length": r * sweep,
                     "wrapDeg": math.degrees(sweep)}

    total = sum(e["length"] for e in edges) + sum(a["length"] for a in arcs.values())
    return {"ok": all_ok and closure_ok, "closure_ok": closure_ok, "flips": flips,
            "order": order, "edges": edges, "arcs": arcs, "dirs": dirs,
            "driver": driver_id, "length": total, "fatal": fatal}


def polar_h(c, r, target):
    """退化情形的占位切点：指向另一圆心。"""
    return g.polar(c, r, g.ang(g.sub(target, c)))


def belt_polyline(path):
    """按带行进方向生成整条闭合带的离散折线（mm）。"""
    order, edges, arcs = path["order"], path["edges"], path["arcs"]
    pts = []
    for i, wid in enumerate(order):
        arc = arcs[wid]
        seg = arc_poly(arc)
        if i == 0:
            pts.extend(seg)
        else:
            pts.extend(seg[1:])
        e = edges[i]
        pts.append(e["p2"])
    return pts


def arc_poly(arc, step_deg=3.0):
    return g.arc_polyline(arc["center"], arc["r"], arc["a0"],
                          arc["dir"] * arc["sweep"],
                          max(6, int(math.degrees(arc["sweep"]) / step_deg)))


def _clean_num(v, default, lo=None, hi=None):
    try:
        x = float(v)
    except (TypeError, ValueError):
        return default
    if lo is not None and x < lo:
        x = lo
    if hi is not None and x > hi:
        x = hi
    return x


# 多级回路调色板（前端 static/js/state.js LOOP_COLORS 保持一致）
LOOP_PALETTE = ["#d8b15a", "#5b9bd5", "#58c08a", "#e0845a", "#b48cd4",
                "#5ac0c0", "#d96c8b", "#9bb05c"]


def analyze_multi(scheme):
    """多级传动校核：多条独立皮带回路 + 共享轴。

    转速/转向/扭矩沿“回路-共享轴”图传播；几何、包角、张力、张紧行程、
    干涉等校核逐回路复用 build_paths；再汇总轴上扭矩与径向合力。
    """
    G = dict(scheme.get("globals", {}) or {})
    pulleys_in = scheme.get("pulleys", []) or []
    obstacles = scheme.get("obstacles", []) or []
    loops_in = scheme.get("loops", []) or []
    shafts_in = scheme.get("shafts", []) or []

    warnings = []

    def warn(severity, code, title, refs=None, message=None, basis=None, loop=None):
        warnings.append({"severity": severity, "code": code, "title": title,
                         "message": message or title, "refs": refs or {},
                         "basis": basis or [], "loop": loop})

    def pref(loop_id, msg):
        """告警消息加回路前缀。"""
        return msg  # loop 字段已在结构化结果中给出，消息保持简洁

    # ---------- 参数清洗（与单级模式一致） ----------
    for key, default in (("inputRpm", 0.0), ("powerKw", 0.0), ("friction", 0.0),
                         ("allowTension", 0.0), ("stretchRate", 0.01)):
        G[key] = _clean_num(G.get(key, default), default, 0.0)
    G["minWrapDeg"] = _clean_num(G.get("minWrapDeg", 120), 120, 0.0, 360.0)
    G["safetyGap"] = _clean_num(G.get("safetyGap", 10), 10, 0.0)
    G["tensionerTravel"] = _clean_num(G.get("tensionerTravel", 0), 0, 0.0)
    G["efficiency"] = min(1.0, max(0.0, _clean_num(G.get("efficiency", 0.96), 0.96, 0.0, 1.0)))

    for p in pulleys_in:
        p["diameter"] = _clean_num(p.get("diameter", 0), 0, 1.0)
        try:
            p["x"] = float(p.get("x", 0)); p["y"] = float(p.get("y", 0))
        except (TypeError, ValueError):
            p["x"], p["y"] = 0.0, 0.0
    for o in obstacles:
        for k in ("x", "y", "w", "h"):
            o[k] = _clean_num(o.get(k, 0), 0, 0.0 if k in ("x", "y") else 1.0)
        o["rot"] = _clean_num(o.get("rot", 0), 0.0, -360.0, 360.0)

    # ---------- 轴归一化：缺 shaftId 的轮自动建独立轴；坐标以轴为准 ----------
    shaft_map = {}
    for i, s in enumerate(shafts_in):
        s = s if isinstance(s, dict) else {}
        sid = s.get("id") or ("s%d" % (i + 1))
        shaft_map[sid] = {
            "id": sid,
            "name": s.get("name") or ("轴 %d" % (i + 1)),
            "x": _clean_num(s.get("x", 0), 0.0),
            "y": _clean_num(s.get("y", 0), 0.0),
            "locked": bool(s.get("locked", False)),
            "allowTorque": _clean_num(s.get("allowTorque", 0), 0.0, 0.0),
            "allowRadial": _clean_num(s.get("allowRadial", 0), 0.0, 0.0),
        }

    auto_seq = len(shaft_map)
    for p in pulleys_in:
        sid = p.get("shaftId")
        if sid not in shaft_map:
            auto_seq += 1
            sid = sid if isinstance(sid, str) and sid else ("s%d" % auto_seq)
            shaft_map[sid] = {"id": sid, "name": "独立轴 " + p.get("name", p["id"]),
                              "x": p["x"], "y": p["y"], "locked": bool(p.get("locked")),
                              "allowTorque": 0.0, "allowRadial": 0.0}
            p["shaftId"] = sid
        else:
            p["locked"] = shaft_map[sid]["locked"]

    # 轮的有效坐标取轴坐标（build_paths 使用 copies，不改前端数据）
    pcopies = []
    for p in pulleys_in:
        q = dict(p)
        s = shaft_map[p["shaftId"]]
        q["x"], q["y"] = s["x"], s["y"]
        pcopies.append(q)
    pmap = {p["id"]: p for p in pcopies}

    # ---------- 回路归一化与结构校验 ----------
    loops = []
    seen_pulley_loop = {}
    fatal_struct = False
    for i, l0 in enumerate(loops_in):
        l0 = l0 if isinstance(l0, dict) else {}
        lid = l0.get("id") or ("L%d" % (i + 1))
        order = [str(x) for x in (l0.get("order") or [])]
        crossed = l0.get("crossedEdges", {}) or {}
        lp = {
            "id": lid, "index": i,
            "name": l0.get("name") or ("回路 %d" % (i + 1)),
            "order": order, "crossed": crossed,
            "powerKw": _clean_num(l0.get("powerKw", G["powerKw"]), G["powerKw"], 0.0),
            "efficiency": min(1.0, max(0.0,
                _clean_num(l0.get("efficiency", G["efficiency"]), G["efficiency"], 0.0, 1.0))),
            "inputRpm": _clean_num(l0.get("inputRpm", G["inputRpm"]), G["inputRpm"], 0.0),
            "friction": _clean_num(l0.get("friction", G["friction"]), G["friction"], 0.0),
            "allowTension": _clean_num(l0.get("allowTension", G["allowTension"]),
                                       G["allowTension"], 0.0),
            "minWrapDeg": _clean_num(l0.get("minWrapDeg", G["minWrapDeg"]),
                                     G["minWrapDeg"], 0.0, 360.0),
            "tensionerTravel": _clean_num(l0.get("tensionerTravel", G["tensionerTravel"]),
                                          G["tensionerTravel"], 0.0),
            "stretchRate": _clean_num(l0.get("stretchRate", G["stretchRate"]),
                                      G["stretchRate"], 0.0),
            "isInput": bool(l0.get("isInput", False)),
        }
        # 引用与重复校验
        missing = [w for w in order if w not in pmap]
        if missing:
            warn("error", "LOOP_MISSING_PULLEY",
                 "%s 引用了不存在的带轮：%s" % (lp["name"], ",".join(missing)),
                 {"loops": [lid]}, loop=lid)
            lp["skip"] = True
        dups = {w for w in order if order.count(w) > 1}
        if dups:
            warn("error", "LOOP_DUP_PULLEY",
                 "%s 中同一带轮出现多次：%s" % (lp["name"], ",".join(sorted(dups))),
                 {"pulleys": sorted(dups), "loops": [lid]}, loop=lid)
            lp["skip"] = True
        if len(order) < 2 and not lp.get("skip"):
            warn("error", "LOOP_TOO_SHORT", "%s 至少需要 2 个带轮" % lp["name"],
                 {"loops": [lid]}, loop=lid)
            lp["skip"] = True
        for w in order:
            if w in seen_pulley_loop:
                warn("error", "PULLEY_IN_TWO_LOOPS",
                     "%s 同时属于 %s 和 %s：一个带轮只能绕一条皮带"
                     % (pmap[w]["name"], seen_pulley_loop[w], lp["name"]),
                     {"pulleys": [w], "loops": [seen_pulley_loop[w], lid]}, loop=lid)
                lp["skip"] = True
            else:
                seen_pulley_loop[w] = lid
        loops.append(lp)

    if not loops:
        return {"ok": False, "mode": "multi",
                "warnings": [{"severity": "error", "code": "NO_LOOP",
                              "title": "尚未建立任何回路",
                              "message": "多级模式至少需要一条皮带回路（每条 ≥2 个带轮）",
                              "refs": {}, "basis": [], "loop": None}],
                "pulleys": [], "shafts": [], "loops": [], "edges": [],
                "beltLength": 0, "beltSpeed": 0, "tension": None,
                "obstacleGaps": {}, "globalBasis": []}

    unrouted = [p["id"] for p in pcopies if p["id"] not in seen_pulley_loop]
    if unrouted:
        warn("warning", "PULLEY_UNROUTED",
             "以下带轮不属于任何回路，不参与校核：%s"
             % ",".join(pmap[w]["name"] for w in unrouted),
             {"pulleys": unrouted})

    # 轴 -> 经过的回路与带轮
    shaft_loops = {sid: [] for sid in shaft_map}
    shaft_pulleys = {sid: [] for sid in shaft_map}
    for lp in loops:
        if lp.get("skip"):
            continue
        for w in lp["order"]:
            sid = pmap[w]["shaftId"]
            shaft_pulleys[sid].append(w)
            if lp["id"] not in shaft_loops[sid]:
                shaft_loops[sid].append(lp["id"])
    for sid, lps in shaft_loops.items():
        if len({}.fromkeys(lps)) != len(lps):
            pass
    # 同一回路中两个轮共轴 => 轮心重合，几何无法成立
    for lp in loops:
        if lp.get("skip"):
            continue
        sids = [pmap[w]["shaftId"] for w in lp["order"]]
        dup_shafts = {s for s in sids if sids.count(s) > 1}
        for sid in dup_shafts:
            ws = [w for w in lp["order"] if pmap[w]["shaftId"] == sid]
            warn("error", "SAME_SHAFT_IN_LOOP",
                 "%s：%s 共轴（%s），同一皮带不能绕过两个同速同轴的轮"
                 % (lp["name"], "、".join(pmap[w]["name"] for w in ws),
                    shaft_map[sid]["name"]),
                 {"pulleys": ws, "shafts": [sid], "loops": [lp["id"]]}, loop=lp["id"])
            lp["skip"] = True

    # ---------- 逐回路几何（A：相对转向，driver 固定 +1） ----------
    loop_results = {}
    for lp in loops:
        if lp.get("skip"):
            loop_results[lp["id"]] = {"ok": False, "order": lp["order"],
                                      "edges": [], "arcs": {}, "relDirs": {},
                                      "length": 0.0, "fatal_msgs": [],
                                      "closure_ok": True, "flips": 0}
            continue
        lp_pulleys = [pmap[w] for w in lp["order"]]
        drivers = [p for p in lp_pulleys if p.get("kind") == "driver"]
        if len(drivers) != 1:
            warn("error", "DRIVER_COUNT",
                 "%s 主动轮数量应为 1（当前 %d 个）" % (lp["name"], len(drivers)),
                 {"pulleys": [p["id"] for p in drivers], "loops": [lp["id"]]},
                 loop=lp["id"])
            loop_results[lp["id"]] = {"ok": False, "order": lp["order"],
                                      "edges": [], "arcs": {}, "relDirs": {},
                                      "length": 0.0, "fatal_msgs": [],
                                      "closure_ok": True, "flips": 0}
            lp["skip"] = True
            continue
        lp["driver"] = drivers[0]["id"]
        path = build_paths(scheme, pulleys=lp_pulleys, route_order=lp["order"],
                           crossed_map=lp["crossed"], driver_id=lp["driver"],
                           driver_dir=1)
        for f in path.get("fatal", []):
            refs = dict(f.get("refs", {})); refs["loops"] = [lp["id"]]
            edge_idx = None
            if refs.get("edges"):
                edge_idx = refs["edges"][0]
                refs["lrefs"] = [[lp["id"], edge_idx]]
            warn("error", "TANGENT", "带段无法构造", refs,
                 message="%s：%s" % (lp["name"], f["message"]), loop=lp["id"])
        if not path.get("closure_ok", False):
            warn("error", "ROUTE_CONFLICT", "%s 绕行绕向冲突" % lp["name"],
                 {"loops": [lp["id"]]},
                 message="%s 绕行一周共 %d 次交叉，必须为偶数才能闭合（带的正反面约束）"
                         % (lp["name"], path.get("flips", 0)),
                 basis=["每经过一条交叉带段，下一个轮转向反转一次；",
                        "绕主动轮一周后转向必须与初始一致，故交叉段数应为偶数。"],
                 loop=lp["id"])
        loop_results[lp["id"]] = {
            "ok": path["ok"], "order": path["order"], "edges": path["edges"],
            "arcs": path["arcs"], "relDirs": path["dirs"], "length": path["length"],
            "fatal_msgs": [f["message"] for f in path.get("fatal", [])],
            "closure_ok": path.get("closure_ok", True), "flips": path.get("flips", 0)}

    # ---------- 轮-轮重叠（同回路、不同轴） ----------
    for lp in loops:
        if lp.get("skip"):
            continue
        ids = lp["order"]
        for a in range(len(ids)):
            for b in range(a + 1, len(ids)):
                pa, pb = pmap[ids[a]], pmap[ids[b]]
                if pa["shaftId"] == pb["shaftId"]:
                    continue
                ca = (shaft_map[pa["shaftId"]]["x"], shaft_map[pa["shaftId"]]["y"])
                cb = (shaft_map[pb["shaftId"]]["x"], shaft_map[pb["shaftId"]]["y"])
                overlap = g.circle_overlap(ca, pa["diameter"] / 2,
                                           cb, pb["diameter"] / 2)
                if overlap > 2:
                    warn("error", "WHEEL_OVERLAP",
                         "%s 与 %s 重叠 %.1f mm" % (pa["name"], pb["name"], overlap),
                         {"pulleys": [pa["id"], pb["id"]], "loops": [lp["id"]]},
                         basis=["%s：中心距 %.1f mm，半径和 %.1f mm，重叠 %.1f mm"
                                % (lp["name"], g.dist(ca, cb),
                                   pa["diameter"] / 2 + pb["diameter"] / 2, overlap)],
                         loop=lp["id"])

    # ---------- 非相邻带段相交 / 带段擦碰非相邻轮（限本回路） ----------
    for lp in loops:
        r0 = loop_results[lp["id"]]
        if lp.get("skip") or not r0["edges"]:
            continue
        order, edges, arcs = r0["order"], r0["edges"], r0["arcs"]
        n = len(order)

        def lref(i):
            return {"loops": [lp["id"]], "lrefs": [[lp["id"], i]]}

        for i in range(n):
            for j in range(i + 1, n):
                if j == i or (j + 1) % n == i or (i + 1) % n == j:
                    continue
                ei, ej = edges[i], edges[j]
                if not ei["feasible"] or not ej["feasible"]:
                    continue
                if g.segments_intersect(ei["p1"], ei["p2"], ej["p1"], ej["p2"]):
                    warn("error", "SEGMENT_CROSS",
                         "%s 带段相交：%s→%s 与 %s→%s"
                         % (lp["name"], ei["from"], ei["to"], ej["from"], ej["to"]),
                         {"pulleys": [ei["from"], ei["to"], ej["from"], ej["to"]],
                          "loops": [lp["id"]], "lrefs": [[lp["id"], i], [lp["id"], j]]},
                         message="%s：非交叉模式下两条直线带段彼此穿越，皮带无法这样安装"
                                 % lp["name"], loop=lp["id"])
        for i, e in enumerate(edges):
            if not e["feasible"]:
                continue
            endpoints = {e["from"], e["to"]}
            for wid in order:
                if wid in endpoints:
                    continue
                pw = pmap[wid]
                c = (shaft_map[pw["shaftId"]]["x"], shaft_map[pw["shaftId"]]["y"])
                r = pw["diameter"] / 2
                d, _t = g.point_segment_distance(c, e["p1"], e["p2"])
                if d < r - 1:
                    refs = {"pulleys": [e["from"], e["to"], wid], "loops": [lp["id"]],
                            "lrefs": [[lp["id"], i]]}
                    warn("error", "SEGMENT_WHEEL",
                         "%s 带段 %s→%s 擦碰 %s（切入 %.1f mm）"
                         % (lp["name"], e["from"], e["to"], pw["name"], r - d),
                         refs, basis=["%s：轮心到带段距离 %.1f mm < 半径 %.1f mm"
                                      % (lp["name"], d, r)], loop=lp["id"])

        # 包角（逐回路阈值）
        min_wrap = math.radians(float(lp["minWrapDeg"]))
        for wid in order:
            sweep = arcs[wid]["sweep"]
            if sweep + 1e-9 < min_wrap:
                warn("warning", "SMALL_WRAP",
                     "%s：%s 包角不足 %.1f° < %.0f°"
                     % (lp["name"], pmap[wid]["name"],
                        math.degrees(sweep), lp["minWrapDeg"]),
                     {"pulleys": [wid], "loops": [lp["id"]]},
                     basis=["%s：包角 θ 由入切点沿轮转向扫到出切点求得；θ = %.2f°，许用 %.0f°"
                            % (lp["name"], math.degrees(sweep), lp["minWrapDeg"])],
                     loop=lp["id"])

    # ---------- B：转速/转向沿“回路-共享轴”图传播 ----------
    shaft_rpm = {}     # 有符号转速
    pulley_rpm = {}
    pulley_dir = {}
    processed_loops = set()

    # 种子：标记为输入（isInput）的回路；默认第一条回路为输入
    input_loops = [lp for lp in loops if not lp.get("skip") and lp.get("isInput")]
    if not input_loops:
        first_ok = next((lp for lp in loops if not lp.get("skip")), None)
        if first_ok:
            input_loops = [first_ok]
    input_loop_ids = {lp["id"] for lp in input_loops}

    queue = []
    for lp in input_loops:
        drv = pmap[lp["driver"]]
        sid = drv["shaftId"]
        n0 = lp["inputRpm"] if lp["inputRpm"] > 0 else G["inputRpm"]
        sdir0 = 1 if drv.get("dir", 1) >= 0 else -1
        if sid in shaft_rpm:
            pass
        else:
            shaft_rpm[sid] = n0 * sdir0
            queue.append(sid)
        lp["seedRpm"] = n0

    def loop_prop(lp, seed_shaft, seed_rpm_signed):
        """从已知轴转速出发，按回路带轮比传播，返回 {shaftId: signedRpm} 新值。"""
        rel = loop_results[lp["id"]]["relDirs"]
        drv = lp["driver"]
        drel = rel.get(drv, 1) or 1
        r_drv = pmap[drv]["diameter"] / 2.0
        out = {}
        drv_signed = seed_rpm_signed  # 主动轮轴转速即种子
        for w in lp["order"]:
            ratio = (r_drv / (pmap[w]["diameter"] / 2.0)) * (rel.get(w, 1) / drel)
            out[pmap[w]["shaftId"]] = (w, drv_signed * ratio)
        return out

    head = 0
    while head < len(queue):
        sid = queue[head]; head += 1
        for lid in shaft_loops.get(sid, []):
            if lid in processed_loops:
                continue
            lp = next(x for x in loops if x["id"] == lid)
            if lp.get("skip"):
                processed_loops.add(lid)
                continue
            vals = loop_prop(lp, sid, shaft_rpm[sid])
            conflict = False
            for s2, (w, val) in vals.items():
                if s2 in shaft_rpm:
                    old = shaft_rpm[s2]
                    if abs(old) > 1e-9 and abs(val - old) > max(2.0, abs(old) * 0.02):
                        warn("error", "SHAFT_SPEED_CONFLICT",
                             "轴 %s 转速矛盾：%s 要求 %.0f rpm，%s 推得 %.0f rpm"
                             % (shaft_map[s2]["name"], "上游", old, lp["name"], val),
                             {"shafts": [s2, sid], "loops": [lid] +
                              [l for l in shaft_loops[s2] if l != lid]},
                             message="闭环转速不一致：轴 %s 经不同回路得到 %.0f 与 %.0f rpm"
                                     % (shaft_map[s2]["name"], old, val),
                             basis=["n从 = n主·d主/d从；沿各回路传播后，同一共享轴的转速必须相等。",
                                    "差异 %.1f%%，请调整轮径比或检查输入转速。"
                                    % (100 * abs(val - old) / max(abs(old), 1e-9))],
                             loop=lid)
                        conflict = True
                    elif abs(old) > 1e-9 and (old > 0) != (val > 0):
                        warn("error", "SHAFT_DIR_CONFLICT",
                             "轴 %s 转向矛盾：%s 要求 %s，%s 推得 %s"
                             % (shaft_map[s2]["name"], "上游",
                                "CW" if old > 0 else "CCW", lp["name"],
                                "CW" if val > 0 else "CCW"),
                             {"shafts": [s2], "loops": [lid]},
                             message="闭环转向不一致：轴 %s 被推为两个相反方向，"
                                     "请调整交叉段或轮系布置" % shaft_map[s2]["name"],
                             loop=lid)
                        conflict = True
                else:
                    shaft_rpm[s2] = val
                    queue.append(s2)
            processed_loops.add(lid)
            if conflict:
                break

    for lp in loops:
        if lp.get("skip"):
            continue
        if lp["id"] not in processed_loops:
            warn("error", "LOOP_DISCONNECTED",
                 "%s 未连接到输入轴（电机回路），无法确定转速" % lp["name"],
                 {"loops": [lp["id"]]},
                 message="%s 与输入回路之间没有共享轴相连，请检查轴的归并关系" % lp["name"],
                 basis=["多级传动中，每条回路必须通过共享轴从电机回路获得转速。"],
                 loop=lp["id"])

    # 带轮有符号转速/绝对转向
    for lp in loops:
        r0 = loop_results[lp["id"]]
        rel = r0.get("relDirs", {})
        for w in lp.get("order", []):
            sid = pmap[w]["shaftId"]
            signed = shaft_rpm.get(sid)
            if signed is not None:
                pulley_rpm[w] = signed
                pulley_dir[w] = 1 if signed >= 0 else -1
            else:
                pulley_rpm[w] = 0.0
                pulley_dir[w] = rel.get(w, 1)

    # 目标转速/转向校核
    for p in pcopies:
        if p.get("kind") != "driven":
            continue
        actual = pulley_rpm.get(p["id"], 0.0)
        t_rpm = p.get("targetRpm")
        if t_rpm and abs(actual) > 0 and abs(abs(actual) - t_rpm) / t_rpm > 0.02:
            lp_id = seen_pulley_loop.get(p["id"])
            warn("warning", "RPM_MISMATCH",
                 "%s 转速 %.0f rpm，偏离目标 %.0f rpm 超过 2%%"
                 % (p["name"], abs(actual), t_rpm),
                 {"pulleys": [p["id"]], "loops": [lp_id] if lp_id else []},
                 basis=["n = n轴 × d主动/d从动 = %.0f rpm（多级：沿共享轴逐级传播）"
                        % abs(actual)], loop=lp_id)
        t_dir = p.get("targetDir")
        if t_dir in (1, -1) and actual != 0:
            actual_dir = 1 if actual > 0 else -1
            if actual_dir != t_dir:
                lp_id = seen_pulley_loop.get(p["id"])
                warn("warning", "DIR_MISMATCH",
                     "%s 转向与要求相反（当前 %s，要求 %s）"
                     % (p["name"], "CW" if actual_dir > 0 else "CCW",
                        "CW" if t_dir > 0 else "CCW"),
                     {"pulleys": [p["id"]], "loops": [lp_id] if lp_id else []},
                     basis=["开口带从动轮与主动轮同向，每经过 1 条交叉段转向反转一次；",
                            "多级传动中转向还要沿共享轴向下一条回路延续。"],
                     loop=lp_id)

    # ---------- 护罩干涉（全部回路的带段与轮弧） ----------
    belt_features = []
    for lp in loops:
        r0 = loop_results[lp["id"]]
        if lp.get("skip"):
            continue
        for i, e in enumerate(r0["edges"]):
            if e["feasible"]:
                belt_features.append(("edge", lp["id"], i, [e["p1"], e["p2"]]))
        for wid in lp["order"]:
            belt_features.append(("arc", lp["id"], wid,
                                  arc_poly(r0["arcs"][wid])))
    safety = float(G.get("safetyGap", 10))
    obstacle_gaps = {}
    for o in obstacles:
        hit_wheels = []
        for p in pcopies:
            c = (shaft_map[p["shaftId"]]["x"], shaft_map[p["shaftId"]]["y"])
            if g.circle_obb_overlap(c, p["diameter"] / 2, o):
                hit_wheels.append(p["id"])
        if hit_wheels:
            lp_ids = sorted({seen_pulley_loop.get(w) for w in hit_wheels
                             if seen_pulley_loop.get(w)})
            warn("error", "WHEEL_OBSTACLE",
                 "%s 与护罩/障碍 %s 重叠"
                 % (",".join(pmap[w]["name"] for w in hit_wheels),
                    o.get("name", o["id"])),
                 {"pulleys": hit_wheels, "obstacles": [o["id"]],
                  "loops": [x for x in lp_ids if x]})
        feat_gaps = []
        for kind, lid, fid, poly in belt_features:
            feat_gaps.append((kind, lid, fid, g.polyline_min_distance_to_obb(poly, o)))
        gap = min((d for _, _, _, d in feat_gaps), default=None)
        obstacle_gaps[o["id"]] = gap
        if gap is None or gap >= safety:
            continue
        hit = [(k, lid, fid) for k, lid, fid, d in feat_gaps
               if d <= max(gap + 0.5, safety - 1e-9)]
        hit_edges = [[lid, fid] for k, lid, fid in hit if k == "edge"]
        hit_arcs = [fid for k, lid, fid in hit if k == "arc"]
        hit_loops = sorted({lid for k, lid, fid in hit})
        oname = o.get("name", o["id"])

        def feat_desc():
            es = ["%s 段 %s→%s" % (lid,
                  loop_results[lid]["edges"][fid]["from"],
                  loop_results[lid]["edges"][fid]["to"])
                  for k, lid, fid in hit if k == "edge"]
            ar = ["%s(%s)" % (pmap[fid]["name"], lid)
                  for k, lid, fid in hit if k == "arc"]
            return "、".join(es + ar)

        refs = {"obstacles": [o["id"]], "lrefs": hit_edges,
                "pulleys": hit_arcs, "loops": hit_loops}
        if gap <= 0.5:
            warn("error", "BELT_COLLISION",
                 "皮带与 %s 碰撞（%s）" % (oname, feat_desc()), refs,
                 basis=["逐段计算各回路带（直线段 + 轮上弧段 3° 离散）到旋转矩形边界的最小距离；",
                        "碰撞特征：%s，最小距离 0 mm。" % feat_desc()])
        else:
            warn("warning", "GUARD_GAP",
                 "皮带与 %s 间隙 %.1f mm < 安全间隙 %.0f mm（%s）"
                 % (oname, gap, safety, feat_desc()), refs,
                 basis=["逐段计算各回路带到旋转矩形边界的最小距离；",
                        "实测 %.1f mm，要求 %.0f mm；间隙不足特征：%s。"
                        % (gap, safety, feat_desc())])

    # ---------- C：逐回路张力、径向力、张紧行程 ----------
    shaft_fx = {sid: 0.0 for sid in shaft_map}
    shaft_fy = {sid: 0.0 for sid in shaft_map}
    shaft_torque = {sid: 0.0 for sid in shaft_map}
    out_loops = []
    total_length = 0.0
    first_speed = 0.0
    any_tension = None

    for lp in loops:
        r0 = loop_results[lp["id"]]
        order = r0.get("order", [])
        edges = r0.get("edges", [])
        arcs = r0.get("arcs", {})
        entry = {"id": lp["id"], "name": lp["name"], "index": lp["index"],
                 "color": LOOP_PALETTE[lp["index"] % len(LOOP_PALETTE)],
                 "ok": r0["ok"] and not lp.get("skip"),
                 "order": order, "edges": [], "length": r0["length"],
                 "beltSpeed": 0.0, "tension": None, "traveler": [],
                 "powerKw": lp["powerKw"], "efficiency": lp["efficiency"],
                 "outPower": lp["powerKw"] * lp["efficiency"],
                 "inputRpm": abs(pulley_rpm.get(lp["driver"], 0.0))
                             if lp.get("driver") and pulley_rpm.get(lp["driver"]) is not None
                             else lp.get("seedRpm", lp["inputRpm"]),
                 "driver": lp.get("driver"), "centerDistance": None,
                 "closureOk": r0.get("closure_ok", True)}
        total_length += r0["length"]
        if lp.get("skip") or not edges:
            out_loops.append(entry)
            continue

        n = len(order)
        drv_rpm = abs(pulley_rpm.get(lp["driver"], 0.0))
        drv_d = pmap[lp["driver"]]["diameter"]
        belt_speed = math.pi * drv_d * drv_rpm / 60000.0
        entry["beltSpeed"] = belt_speed
        if lp["id"] in input_loop_ids:
            first_speed = max(first_speed, belt_speed)

        # 紧/松边判定：从主动轮出发，遇承载轮（主/从）翻转，惰轮保持
        side = {}  # 边索引 -> 'tight' / 'slack'
        d0 = order.index(lp["driver"])
        side[d0] = "tight"
        for k in range(1, n + 1):
            idx = (d0 + k) % n
            in_edge = (idx - 1) % n
            w = order[idx if k < n else d0]
            flip = pmap[w].get("kind") in ("driver", "driven")
            side[idx % n] = side[in_edge] if not flip else (
                "slack" if side[in_edge] == "tight" else "tight")

        tension = None
        if belt_speed > 0:
            power = lp["powerKw"]
            mu = lp["friction"]
            allow = lp["allowTension"]
            load_wraps = [arcs[w]["sweep"] for w in order
                          if pmap[w].get("kind") in ("driver", "driven")]
            theta = min(load_wraps) if load_wraps else \
                min(arcs[w]["sweep"] for w in order)
            ratio = math.exp(mu * theta)
            fe = 1000.0 * power / belt_speed
            if ratio <= 1.0001:
                warn("error", "NO_GRIP", "%s 包角接近 0，无法传递载荷" % lp["name"],
                     {"loops": [lp["id"]]}, basis=["μθ 过小，e^(μθ)≈1。"],
                     loop=lp["id"])
            else:
                f2 = fe / (ratio - 1)
                f1 = fe + f2
                f0 = (f1 + f2) / 2.0
                pmax = belt_speed * allow * (1 - 1 / ratio) / 1000.0
                tw = min(order, key=lambda w: arcs[w]["sweep"]
                         if pmap[w].get("kind") in ("driver", "driven") else 99)
                tension = {"fe": fe, "f1": f1, "f2": f2, "f0": f0,
                           "ratio": ratio, "thetaWheel": tw, "pmax": pmax,
                           "beltSpeed": belt_speed, "thetaDeg": math.degrees(theta)}
                any_tension = tension
                tb = ["[%s] v = π·d主·n/60000 = π×%.0f×%.0f/60000 = %.2f m/s"
                      % (lp["name"], drv_d, drv_rpm, belt_speed),
                      "有效拉力 Fe = 1000P/v = 1000×%.2f/%.2f = %.0f N"
                      % (power, belt_speed, fe),
                      "最小承载轮包角 θ = %.1f°（%s）"
                      % (math.degrees(theta), pmap[tw]["name"]),
                      "极限张力比 e^(μθ) = %.2f；松边 F2 = %.0f N；紧边 F1 = %.0f N"
                      % (ratio, f2, f1),
                      "初拉力 F0 ≈ (F1+F2)/2 = %.0f N；最大可传递功率 Pmax = %.2f kW"
                      % (f0, pmax)]
                if f1 > allow and allow > 0:
                    warn("error", "TENSION",
                         "%s 紧边张力 %.0f N > 许用 %.0f N（最多传 %.2f kW）"
                         % (lp["name"], f1, allow, pmax),
                         {"pulleys": [tw], "loops": [lp["id"]]},
                         basis=tb, loop=lp["id"])
                elif pmax < power:
                    warn("warning", "POWER_LOW",
                         "%s 最大可传递功率 %.2f kW < 需求 %.2f kW"
                         % (lp["name"], pmax, power),
                         {"loops": [lp["id"]]}, basis=tb, loop=lp["id"])
        entry["tension"] = tension

        # 各轮径向力（紧/松边张力沿带段方向的矢量和）
        if tension:
            for ci, wid in enumerate(order):
                e_out = edges[ci]      # wid -> next
                e_in = edges[ci - 1]   # prev -> wid
                fx = fy = 0.0
                for e, force, at_p1 in ((e_out, tension["f1"]
                                         if side[ci] == "tight" else tension["f2"], True),
                                        (e_in, tension["f1"]
                                         if side[(ci - 1) % n] == "tight" else tension["f2"], False)):
                    if not e["feasible"]:
                        continue
                    if at_p1:
                        v = g.norm(g.sub(e["p2"], e["p1"]))
                    else:
                        v = g.norm(g.sub(e["p1"], e["p2"]))
                    fx += force * v[0]; fy += force * v[1]
                sid = pmap[wid]["shaftId"]
                shaft_fx[sid] += fx
                shaft_fy[sid] += fy

        # 轴上扭矩 T=9550P/n。同一根轴上各级带轮转速相同：
        # 非输入回路的主动轮功率由上游回路 P·η 提供（共享轴功率连续）；
        # 输入回路的主动轮按该回路填写的功率；各从动轮按本回路 η 后的输出功率。
        if drv_rpm > 0:
            drv_sid = pmap[lp["driver"]]["shaftId"]
            if lp["id"] in input_loop_ids:
                p_at_driver = lp["powerKw"]
            else:
                # 找所有把动力输出到本回路主动轮所在轴的上游从动轮（按回路效率折减）
                p_at_driver = 0.0
                for up in loops:
                    if up.get("skip") or up["id"] == lp["id"]:
                        continue
                    up_on = [w for w in up.get("order", [])
                             if pmap[w].get("kind") == "driven"
                             and pmap[w]["shaftId"] == drv_sid]
                    if up_on:
                        n_driven = sum(1 for w in up["order"]
                                       if pmap[w].get("kind") == "driven")
                        p_at_driver += (up["powerKw"] * up["efficiency"]
                                        * len(up_on) / max(1, n_driven))
            entry["shaftInputKw"] = p_at_driver
            t_drv = 9550.0 * p_at_driver / drv_rpm if p_at_driver > 0 else 0.0
            shaft_torque[drv_sid] = max(shaft_torque[drv_sid], t_drv)
            drivens = [w for w in order if pmap[w].get("kind") == "driven"]
            for w in drivens:
                nr = abs(pulley_rpm.get(w, 0.0))
                if nr > 0:
                    share = lp["powerKw"] * lp["efficiency"] / max(1, len(drivens))
                    tw = 9550.0 * share / nr
                    shaft_torque[pmap[w]["shaftId"]] = \
                        max(shaft_torque[pmap[w]["shaftId"]], tw)

        # 张紧行程
        stretch_rate = lp["stretchRate"]
        for wid in order:
            p = pmap[wid]
            if p.get("kind") != "tensioner":
                continue
            idx = order.index(wid)
            prev_w = order[idx - 1]
            next_w = order[(idx + 1) % n]
            m = ((pmap[prev_w]["x"] + pmap[next_w]["x"]) / 2,
                 (pmap[prev_w]["y"] + pmap[next_w]["y"]) / 2)
            direction = g.norm(g.sub(m, (p["x"], p["y"])))
            if g.length(direction) < 1e-9:
                direction = (0, 1)
            step = 2.0
            moved = (p["x"] + direction[0] * step, p["y"] + direction[1] * step)
            lp_pulleys = [pmap[w] for w in order]
            path2 = build_paths(scheme, {wid: moved}, pulleys=lp_pulleys,
                                route_order=order, crossed_map=lp["crossed"],
                                driver_id=lp["driver"], driver_dir=1)
            rate = (r0["length"] - path2["length"]) / step
            span = 0.005 * (g.dist((p["x"], p["y"]), (pmap[prev_w]["x"], pmap[prev_w]["y"]))
                            + g.dist((p["x"], p["y"]), (pmap[next_w]["x"], pmap[next_w]["y"])))
            install_a = span
            need = r0["length"] * stretch_rate + install_a
            avail = lp["tensionerTravel"]
            needed_travel = need / rate if rate > 0.01 else None
            entry["traveler"].append({"id": wid, "rate": rate, "need": need,
                                      "install": install_a,
                                      "stretch": r0["length"] * stretch_rate,
                                      "neededTravel": needed_travel,
                                      "available": avail, "direction": direction})
            tb = ["[%s] 张紧轮内压 %.0f mm 试算，带长缩短 %.1f mm，效率 %.2f"
                  % (lp["name"], step, r0["length"] - path2["length"], rate),
                  "需补偿 %.1f mm，对应行程 %s mm，可用 %.0f mm"
                  % (need, ("%.1f" % needed_travel) if needed_travel else "∞", avail)]
            if rate <= 0.01:
                warn("warning", "TENSIONER_DIR",
                     "%s：%s 沿当前内压方向几乎不能改变带长" % (lp["name"], p["name"]),
                     {"pulleys": [wid], "loops": [lp["id"]]}, basis=tb, loop=lp["id"])
            elif needed_travel is not None and avail < needed_travel:
                warn("warning", "TENSIONER_TRAVEL",
                     "%s：%s 行程不足：需 %.1f mm，可用 %.0f mm"
                     % (lp["name"], p["name"], needed_travel, avail),
                     {"pulleys": [wid], "loops": [lp["id"]]}, basis=tb,
                     loop=lp["id"])

        if n == 2:
            entry["centerDistance"] = g.dist(
                (pmap[order[0]]["x"], pmap[order[0]]["y"]),
                (pmap[order[1]]["x"], pmap[order[1]]["y"]))
        for i, e in enumerate(edges):
            entry["edges"].append({"index": i, "from": e["from"], "to": e["to"],
                                   "crossed": e["crossed"], "feasible": e["feasible"],
                                   "p1": e["p1"], "p2": e["p2"],
                                   "length": e["length"],
                                   "side": side.get(i)})
        out_loops.append(entry)

    # ---------- 轴汇总与许用校核 ----------
    out_shafts = []
    for sid, s in shaft_map.items():
        radial = math.hypot(shaft_fx[sid], shaft_fy[sid])
        torque = shaft_torque[sid]
        rpm_signed = shaft_rpm.get(sid)
        info = {**s, "rpm": abs(rpm_signed) if rpm_signed is not None else None,
                "dir": (1 if (rpm_signed or 0) >= 0 else -1) if rpm_signed is not None else 0,
                "torque": torque, "inTorque": torque, "outTorque": torque,
                "radial": radial, "radialX": shaft_fx[sid], "radialY": shaft_fy[sid],
                "loops": shaft_loops[sid],
                "pulleys": shaft_pulleys[sid]}
        if s["allowTorque"] > 0 and torque > s["allowTorque"]:
            warn("error", "SHAFT_TORQUE",
                 "轴 %s 扭矩 %.1f N·m > 许用 %.1f N·m"
                 % (s["name"], torque, s["allowTorque"]),
                 {"shafts": [sid], "loops": shaft_loops[sid]},
                 message="轴 %s 扭矩超限：T=9550P/n 求得 %.1f N·m，许用 %.1f N·m"
                         % (s["name"], torque, s["allowTorque"]),
                 basis=["各承载轮 T = 9550·P/n（主动端 P 为回路功率，从动端 P 为 P·η）；",
                        "同一根轴上各级带轮转速相同，扭矩取最大一级 %.1f N·m。" % torque])
        if s["allowRadial"] > 0 and radial > s["allowRadial"]:
            warn("error", "SHAFT_RADIAL",
                 "轴 %s 径向合力 %.0f N > 许用 %.0f N"
                 % (s["name"], radial, s["allowRadial"]),
                 {"shafts": [sid], "loops": shaft_loops[sid]},
                 basis=["各回路紧边 F1、松边 F2 沿带段方向矢量叠加到共享轴；",
                        "合力 Fr = √(Fx²+Fy²) = %.0f N，许用 %.0f N。"
                        % (radial, s["allowRadial"])])
        out_shafts.append(info)

    # ---------- 输出带轮（扁平，含回路号与弧段） ----------
    out_pulleys = []
    for lp in loops:
        r0 = loop_results[lp["id"]]
        for wid in lp.get("order", []):
            p = pmap[wid]
            s = shaft_map[p["shaftId"]]
            arc = r0.get("arcs", {}).get(wid)
            out_pulleys.append({
                "id": wid, "name": p["name"], "kind": p.get("kind"),
                "x": s["x"], "y": s["y"], "diameter": p["diameter"],
                "radius": p["diameter"] / 2.0, "locked": s["locked"],
                "shaftId": p["shaftId"], "loopId": lp["id"],
                "rpm": pulley_rpm.get(wid, 0.0), "dir": pulley_dir.get(wid, 0),
                "wrapDeg": math.degrees(arc["sweep"]) if arc else 0.0,
                "arcA0": arc["a0"] if arc else 0.0,
                "arcSweep": arc["sweep"] if arc else 0.0,
                "arcDir": (pulley_dir.get(wid, 1)) if arc else 0,
                "relArcDir": arc["dir"] if arc else 0,
                "targetRpm": p.get("targetRpm"), "targetDir": p.get("targetDir")})

    # 弧段绘制方向应取几何相对方向（不随轴绝对转向翻转）
    for op in out_pulleys:
        lp0 = next((x for x in loops if x["id"] == op["loopId"]), None)
        if lp0 and not lp0.get("skip"):
            op["arcDir"] = loop_results[lp0["id"]]["arcs"][op["id"]]["dir"] \
                if op["id"] in loop_results[lp0["id"]].get("arcs", {}) else op["dir"]

    flat_edges = []
    for entry in out_loops:
        for e in entry["edges"]:
            flat_edges.append({**e, "loopId": entry["id"]})

    return {
        "ok": not any(w["severity"] == "error" for w in warnings),
        "mode": "multi",
        "warnings": warnings,
        "globals": G,
        "pulleys": out_pulleys,
        "shafts": out_shafts,
        "loops": out_loops,
        "edges": flat_edges,
        "beltLength": total_length,
        "beltSpeed": first_speed,
        "tension": any_tension,
        "obstacleGaps": obstacle_gaps,
        "globalBasis": [
            "多级传动：电机转速沿“回路 → 共享轴 → 下一回路”传播，"
            "n从 = n轴·d主/d从，交叉段反转转向。",
            "每条回路独立计算切点、包角、带长与张力；共享轴上各级带轮转速必须一致，"
            "否则报转速/转向矛盾。",
            "轴上扭矩 T = 9550·P/n（N·m），取各级输入/输出扭矩最大值；"
            "径向合力为各回路紧/松边张力的矢量和。",
            "功率在回路间按效率传递：P出 = P入·η（默认 η=%.2f）。" % G["efficiency"],
        ],
    }


def analyze(scheme):
    if isinstance(scheme.get("loops"), list) and scheme["loops"]:
        return analyze_multi(scheme)
    G = scheme.get("globals", {})
    pulleys = scheme.get("pulleys", [])
    obstacles = scheme.get("obstacles", [])

    warnings = []
    basis_lines = []

    def warn(severity, code, title, refs=None, message=None, basis=None):
        warnings.append({"severity": severity, "code": code, "title": title,
                         "message": message or title, "refs": refs or {},
                         "basis": basis or []})

    # ---------- 输入校验 ----------
    def clean_num(v, default, lo=None, hi=None):
        try:
            x = float(v)
        except (TypeError, ValueError):
            return default
        if lo is not None and x < lo:
            x = lo
        if hi is not None and x > hi:
            x = hi
        return x

    for key, default in (("inputRpm", 0.0), ("powerKw", 0.0), ("friction", 0.0),
                         ("allowTension", 0.0), ("stretchRate", 0.01)):
        G[key] = clean_num(G.get(key, default), default, 0.0)
    G["minWrapDeg"] = clean_num(G.get("minWrapDeg", 120), 120, 0.0, 360.0)
    G["safetyGap"] = clean_num(G.get("safetyGap", 10), 10, 0.0)
    G["tensionerTravel"] = clean_num(G.get("tensionerTravel", 0), 0, 0.0)
    for p in pulleys:
        p["diameter"] = clean_num(p.get("diameter", 0), 0, 1.0)
        try:
            p["x"] = float(p.get("x", 0)); p["y"] = float(p.get("y", 0))
        except (TypeError, ValueError):
            p["x"], p["y"] = 0.0, 0.0
    for o in obstacles:
        for k in ("x", "y", "w", "h"):
            o[k] = clean_num(o.get(k, 0), 0, 0.0 if k in ("x", "y") else 1.0)
        o["rot"] = clean_num(o.get("rot", 0), 0.0, -360.0, 360.0)

    # ---------- 基础校验 ----------
    empty_result = {"ok": False, "warnings": [], "pulleys": [], "edges": [],
                    "beltLength": None, "centerDistance": None, "beltSpeed": 0,
                    "tension": None, "traveler": [], "obstacleGaps": {},
                    "globalBasis": []}
    if len(pulleys) < 2:
        empty_result["warnings"] = [{"severity": "error", "code": "NO_WHEELS",
            "title": "轮数不足", "message": "至少放置 2 个轮", "refs": {}, "basis": []}]
        return empty_result
    drivers = [p for p in pulleys if p.get("kind") == "driver"]
    if len(drivers) != 1:
        warn("error", "DRIVER_COUNT", "主动轮数量应为 1（当前 %d 个）" % len(drivers),
             {"pulleys": [p["id"] for p in drivers]})

    path = build_paths(scheme)

    # 构造错误（含闭环绕向冲突）
    for f in path.get("fatal", []):
        warn("error", "TANGENT", "带段无法构造", f["refs"], f["message"])
    if not path.get("closure_ok", False) and "order" in path:
        warn("error", "ROUTE_CONFLICT", "绕行绕向冲突",
             message="绕行一周共 %d 次交叉，必须为偶数才能闭合（带的正反面约束）"
                     % path.get("flips", 0),
             basis=["每经过一条交叉带段，下一个轮转向反转一次；",
                    "绕主动轮一周后转向必须与初始一致，故交叉段数应为偶数。"])

    if "order" not in path:
        # 结构性错误（顺序引用缺失、轮数不足等），无法继续几何校核
        result = {**empty_result, "warnings": warnings, "ok": False}
        return result
    order = path["order"]
    pmap = {p["id"]: p for p in pulleys}
    edges, arcs, dirs = path["edges"], path["arcs"], path["dirs"]

    def rp(wid):
        return pmap[wid]["diameter"] / 2.0

    # ---------- 轮-轮重叠 ----------
    for i in range(len(pulleys)):
        for j in range(i + 1, len(pulleys)):
            a, b = pulleys[i], pulleys[j]
            overlap = g.circle_overlap((a["x"], a["y"]), a["diameter"] / 2,
                                       (b["x"], b["y"]), b["diameter"] / 2)
            if overlap > 2:
                warn("error", "WHEEL_OVERLAP", "%s 与 %s 重叠 %.1f mm"
                     % (a["name"], b["name"], overlap),
                     {"pulleys": [a["id"], b["id"]]},
                     basis=["中心距 %.1f mm，半径和 %.1f mm，重叠 %.1f mm"
                            % (g.dist((a["x"], a["y"]), (b["x"], b["y"])),
                               a["diameter"] / 2 + b["diameter"] / 2, overlap)])

    # ---------- 非相邻带段相交 ----------
    n = len(order)
    for i in range(n):
        for j in range(i + 1, n):
            if j == i or (j + 1) % n == i or (i + 1) % n == j:
                continue  # 相邻段共享一个轮
            ei, ej = edges[i], edges[j]
            if not ei["feasible"] or not ej["feasible"]:
                continue
            if g.segments_intersect(ei["p1"], ei["p2"], ej["p1"], ej["p2"]):
                warn("error", "SEGMENT_CROSS",
                     "带段相交：%s→%s 与 %s→%s"
                     % (ei["from"], ei["to"], ej["from"], ej["to"]),
                     {"pulleys": [ei["from"], ei["to"], ej["from"], ej["to"]],
                      "edges": [i, j]},
                     message="非交叉模式下两条直线带段彼此穿越，皮带无法这样安装",
                     basis=["仅两论间成对的内公切线允许交叉（交叉带模式）；",
                            "多轮布置中单个交叉段与其他段相交属于真实空间干涉。"])

    # ---------- 带段擦碰非相邻轮 ----------
    for i, e in enumerate(edges):
        if not e["feasible"]:
            continue
        endpoints = {e["from"], e["to"]}
        for wid in order:
            if wid in endpoints:
                continue
            c = (pmap[wid]["x"], pmap[wid]["y"])
            r = rp(wid)
            d, t = g.point_segment_distance(c, e["p1"], e["p2"])
            if d < r - 1:
                warn("error", "SEGMENT_WHEEL",
                     "带段 %s→%s 擦碰 %s（切入 %.1f mm）"
                     % (e["from"], e["to"], pmap[wid]["name"], r - d),
                     {"pulleys": [e["from"], e["to"], wid], "edges": [i]},
                     basis=["轮心到带段距离 %.1f mm < 半径 %.1f mm" % (d, r)])

    # ---------- 包角 ----------
    min_wrap = math.radians(float(G.get("minWrapDeg", 120)))
    for wid in order:
        wrap = arcs[wid]["sweep"]
        if wrap + 1e-9 < min_wrap:
            warn("warning", "SMALL_WRAP",
                 "%s 包角不足：%.1f° < %.0f°"
                 % (pmap[wid]["name"], math.degrees(wrap), math.degrees(min_wrap)),
                 {"pulleys": [wid]},
                 basis=["包角 θ 由入切点沿轮转向扫到出切点求得；",
                        "θ = %.2f°，许用最小值 %.0f°。"
                        % (math.degrees(wrap), math.degrees(min_wrap))])

    # ---------- 障碍（护罩）干涉与安全间隙 ----------
    # 每条带段/每个轮上弧段分别求到矩形的间隙，便于告警时定位到具体带段
    belt_features = []
    for i, e in enumerate(edges):
        if e["feasible"]:
            belt_features.append(("edge", i, [e["p1"], e["p2"]]))
    for wid in order:
        belt_features.append(("arc", wid, arc_poly(arcs[wid])))
    safety = float(G.get("safetyGap", 10))
    obstacle_gaps = {}
    for o in obstacles:
        # 轮体与矩形
        hit_wheels = []
        for p in pulleys:
            if g.circle_obb_overlap((p["x"], p["y"]), p["diameter"] / 2, o):
                hit_wheels.append(p["id"])
        if hit_wheels:
            warn("error", "WHEEL_OBSTACLE",
                 "%s 与护罩/障碍 %s 重叠" % (",".join(pmap[w]["name"] for w in hit_wheels),
                                       o.get("name", o["id"])),
                 {"pulleys": hit_wheels, "obstacles": [o["id"]]})
        # 逐条特征计算间隙并汇总
        feat_gaps = []
        for kind, fid, poly in belt_features:
            dmin = g.polyline_min_distance_to_obb(poly, o)
            feat_gaps.append((kind, fid, dmin))
        gap = min((d for _, _, d in feat_gaps), default=None)
        obstacle_gaps[o["id"]] = gap
        if gap is None or gap >= safety:
            continue
        hit_edges = [fid for kind, fid, d in feat_gaps
                     if kind == "edge" and d <= max(gap + 0.5, safety - 1e-9)]
        hit_arcs = [fid for kind, fid, d in feat_gaps
                    if kind == "arc" and d <= max(gap + 0.5, safety - 1e-9)]
        oname = o.get("name", o["id"])
        refs = {"obstacles": [o["id"]], "edges": hit_edges, "pulleys": hit_arcs}

        def feat_names(kinds, arcs_):
            parts = []
            if kinds:
                parts.append("直线段 " + "、".join(
                    "%s→%s" % (edges[k]["from"], edges[k]["to"]) for k in kinds))
            if arcs_:
                parts.append("轮弧 " + "、".join(pmap[w]["name"] for w in arcs_))
            return "、".join(parts)

        if gap <= 0.5:
            near = [(k, f, d) for k, f, d in feat_gaps if d <= 0.5]
            ne, na = [f for k, f, _ in near if k == "edge"], \
                     [f for k, f, _ in near if k == "arc"]
            warn("error", "BELT_COLLISION",
                 "皮带与 %s 碰撞（%s）" % (oname, feat_names(ne, na)),
                 {"obstacles": [o["id"]], "edges": ne, "pulleys": na},
                 basis=["逐段计算带（直线段 + 轮上弧段按 3° 离散）到旋转矩形边界的最小距离；",
                        "碰撞特征：%s，最小距离 0 mm。" % feat_names(ne, na)])
        else:
            basis = ["逐段计算带（直线段 + 轮上弧段按 3° 离散）到旋转矩形边界的最小距离；",
                     "实测 %.1f mm，要求 %.0f mm；" % (gap, safety),
                     "间隙不足特征：%s。" % feat_names(hit_edges, hit_arcs)]
            warn("warning", "GUARD_GAP",
                 "皮带与 %s 间隙 %.1f mm < 安全间隙 %.0f mm（%s）"
                 % (oname, gap, safety, feat_names(hit_edges, hit_arcs)),
                 refs, basis=basis)

    # ---------- 转速传播 ----------
    driver = next((p for p in pulleys if p.get("kind") == "driver"), pulleys[0])
    n_in = float(G.get("inputRpm", 1440))
    rpms = {}
    d0 = order.index(driver["id"]) if driver["id"] in order else 0
    mag = {order[d0]: n_in}
    for k in range(1, n):
        i = (d0 + k) % n
        prev = (i - 1) % n
        wid, pwid = order[i], order[prev]
        mag[wid] = mag[pwid] * rp(pwid) / rp(wid)
    for wid in order:
        rpms[wid] = mag[wid] * dirs[wid]
    belt_speed = math.pi * driver["diameter"] * n_in / 60000.0  # m/s

    for p in pulleys:
        if p.get("kind") != "driven":
            continue
        t_rpm = p.get("targetRpm")
        if t_rpm:
            actual = abs(rpms.get(p["id"], 0))
            if actual > 0 and abs(actual - t_rpm) / t_rpm > 0.02:
                warn("warning", "RPM_MISMATCH",
                     "%s 转速 %.0f rpm，偏离目标 %.0f rpm 超过 2%%"
                     % (p["name"], actual, t_rpm),
                     {"pulleys": [p["id"]]},
                     basis=["n = n主 × d主 / d从 = %.0f × %.0f / %.0f = %.0f rpm"
                            % (n_in, driver["diameter"], p["diameter"], actual)])
        t_dir = p.get("targetDir")
        if t_dir in (1, -1):
            actual_dir = dirs.get(p["id"], 0)
            if actual_dir != t_dir:
                warn("warning", "DIR_MISMATCH",
                     "%s 转向与要求相反（当前 %s，要求 %s）"
                     % (p["name"], "CW" if actual_dir > 0 else "CCW",
                        "CW" if t_dir > 0 else "CCW"),
                     {"pulleys": [p["id"]]},
                     basis=["开口带从动轮与主动轮同向，每经过 1 条交叉段转向反转一次。"])

    # ---------- 张力 / 功率校核 ----------
    tension = None
    if belt_speed > 0:
        power = float(G.get("powerKw", 5))
        mu = float(G.get("friction", 0.3))
        allow = float(G.get("allowTension", 800))
        load_wraps = [arcs[w]["sweep"] for w in order
                      if pmap[w].get("kind") in ("driver", "driven")]
        theta = min(load_wraps) if load_wraps else min(arcs[w]["sweep"] for w in order)
        ratio = math.exp(mu * theta)
        fe = 1000.0 * power / belt_speed
        if ratio <= 1.0001:
            warn("error", "NO_GRIP", "包角接近 0，无法传递载荷",
                 basis=["μθ 过小，e^(μθ)≈1。"])
        else:
            f2 = fe / (ratio - 1)
            f1 = fe + f2
            f0 = (f1 + f2) / 2.0
            pmax = belt_speed * allow * (1 - 1 / ratio) / 1000.0
            tension = {"fe": fe, "f1": f1, "f2": f2, "f0": f0,
                       "ratio": ratio, "thetaWheel": min(
                           order, key=lambda w: arcs[w]["sweep"]
                           if pmap[w].get("kind") in ("driver", "driven") else 99),
                       "pmax": pmax, "beltSpeed": belt_speed, "thetaDeg": math.degrees(theta)}
            tb = ["v = π·d主·n / 60000 = π×%.0f×%.0f/60000 = %.2f m/s"
                  % (driver["diameter"], n_in, belt_speed),
                  "有效拉力 Fe = 1000P/v = 1000×%.2f/%.2f = %.0f N" % (power, belt_speed, fe),
                  "最小承载轮包角 θ = %.1f°（%s）"
                  % (math.degrees(theta), pmap[tension["thetaWheel"]]["name"]),
                  "极限张力比 e^(μθ) = e^(%.2f×%.3f) = %.2f" % (theta, mu, ratio),
                  "松边 F2 = Fe/(e^(μθ)-1) = %.0f N" % f2,
                  "紧边 F1 = Fe+F2 = %.0f N" % f1,
                  "初拉力近似 F0 ≈ (F1+F2)/2 = %.0f N（忽略离心张力）" % f0,
                  "该布置最大可传递功率 Pmax = v·[F]·(1-1/e^(μθ))/1000 = %.2f kW" % pmax]
            if f1 > allow:
                warn("error", "TENSION",
                     "紧边张力 %.0f N > 许用 %.0f N（最多传 %.2f kW）"
                     % (f1, allow, pmax),
                     {"pulleys": [tension["thetaWheel"]]}, basis=tb)
            elif pmax < power:
                warn("warning", "POWER_LOW",
                     "最大可传递功率 %.2f kW < 需求 %.2f kW" % (pmax, power),
                     basis=tb)
            basis_lines.extend(tb)

    # ---------- 张紧行程校核 ----------
    travel_info = []
    stretch_rate = float(G.get("stretchRate", 0.01))
    inst_allow_global = G.get("installAllowance")
    for p in pulleys:
        if p.get("kind") != "tensioner":
            continue
        wid = p["id"]
        idx = order.index(wid) if wid in order else -1
        if idx < 0:
            continue
        prev_w = order[idx - 1]
        next_w = order[(idx + 1) % n]
        m = ((pmap[prev_w]["x"] + pmap[next_w]["x"]) / 2,
             (pmap[prev_w]["y"] + pmap[next_w]["y"]) / 2)
        direction = g.norm(g.sub(m, (p["x"], p["y"])))
        if g.length(direction) < 1e-9:
            direction = (0, 1)
        step = 2.0
        moved = (p["x"] + direction[0] * step, p["y"] + direction[1] * step)
        path2 = build_paths(scheme, {wid: moved})
        # 沿内压方向（指向相邻两轮中点）试算：rate>0 表示该方向有效缩短皮带
        rate = (path["length"] - path2["length"]) / step
        # 安装余量：前后两段中心距和的 0.5%
        span = 0.005 * (g.dist((p["x"], p["y"]), (pmap[prev_w]["x"], pmap[prev_w]["y"]))
                        + g.dist((p["x"], p["y"]), (pmap[next_w]["x"], pmap[next_w]["y"])))
        install_a = float(inst_allow_global) if inst_allow_global else span
        need = path["length"] * stretch_rate + install_a
        avail = float(G.get("tensionerTravel", 0))
        needed_travel = need / rate if rate > 0.01 else None
        info = {"id": wid, "rate": rate, "need": need, "install": install_a,
                "stretch": path["length"] * stretch_rate,
                "neededTravel": needed_travel, "available": avail,
                "direction": direction}
        travel_info.append(info)
        tb = ["张紧轮沿指向相邻两轮中点方向内压 %.0f mm 试算，带长缩短 %.1f mm"
              % (step, path["length"] - path2["length"]),
              "缩短效率 %.2f mm 带长 / mm 行程" % rate,
              "伸长补偿 = 带长 × %.2f%% = %.1f mm"
              % (stretch_rate * 100, path["length"] * stretch_rate),
              "安装余量 = 相邻跨距和 × 0.5%% = %.1f mm" % install_a,
              "需补偿带长合计 %.1f mm，对应张紧轮行程 %.1f mm，可用行程 %.0f mm"
              % (need, needed_travel if needed_travel else float("inf"), avail)]
        if rate <= 0.01:
            warn("warning", "TENSIONER_DIR",
                 "%s 沿当前内压方向几乎不能改变带长，请调整张紧轮位置"
                 % p["name"], {"pulleys": [wid]}, basis=tb)
        elif needed_travel is not None and avail < needed_travel:
            warn("warning", "TENSIONER_TRAVEL",
                 "%s 行程不足：需 %.1f mm，可用 %.0f mm"
                 % (p["name"], needed_travel, avail),
                 {"pulleys": [wid]}, basis=tb)

    # ---------- 输出结构 ----------
    center_dist = None
    if n == 2:
        center_dist = g.dist((pmap[order[0]]["x"], pmap[order[0]]["y"]),
                             (pmap[order[1]]["x"], pmap[order[1]]["y"]))

    out_pulleys = []
    for wid in order:
        p = pmap[wid]
        out_pulleys.append({
            "id": wid, "name": p["name"], "kind": p.get("kind"),
            "x": p["x"], "y": p["y"], "diameter": p["diameter"],
            "radius": rp(wid), "locked": bool(p.get("locked")),
            "rpm": rpms.get(wid, 0.0), "dir": dirs.get(wid, 0),
            "wrapDeg": math.degrees(arcs[wid]["sweep"]),
            "arcA0": arcs[wid]["a0"], "arcSweep": arcs[wid]["sweep"],
            "arcDir": arcs[wid]["dir"],
        })

    out_edges = []
    for i, e in enumerate(edges):
        out_edges.append({"index": i, "from": e["from"], "to": e["to"],
                          "crossed": e["crossed"], "feasible": e["feasible"],
                          "p1": e["p1"], "p2": e["p2"], "length": e["length"]})

    return {
        "ok": not any(w["severity"] == "error" for w in warnings),
        "warnings": warnings,
        "pulleys": out_pulleys,
        "edges": out_edges,
        "beltLength": path["length"],
        "centerDistance": center_dist,
        "beltSpeed": belt_speed,
        "tension": tension,
        "traveler": travel_info,
        "obstacleGaps": obstacle_gaps,
        "globalBasis": [
            "几何单位 mm；带长 = Σ直线段长 + Σ(包角弧度 × 半径)。",
            "切点由两圆公切线解出，并要求两轮在切点处的表面速度与带段行进方向一致。",
            "转速 n从 = n主 × d主 / d从；交叉段使下一旋转向。",
        ] + basis_lines,
    }


def compare(scheme_a, scheme_b):
    a, b = analyze(scheme_a), analyze(scheme_b)

    def metrics(r):
        t = r.get("tension") or {}
        if r.get("mode") == "multi":
            loops = r.get("loops", [])
            speeds = [l["beltSpeed"] for l in loops if l.get("beltSpeed")]
            tens = [l.get("tension") for l in loops if l.get("tension")]
            return {
                "beltLength": r.get("beltLength"),
                "beltSpeed": speeds[0] if speeds else 0,
                "minWrap": min((p["wrapDeg"] for p in r["pulleys"]), default=None),
                "f1": max((x["f1"] for x in tens), default=None),
                "f2": max((x["f2"] for x in tens), default=None),
                "f0": max((x["f0"] for x in tens), default=None),
                "pmax": min((x["pmax"] for x in tens), default=None),
                "errors": sum(1 for w in r["warnings"] if w["severity"] == "error"),
                "warns": sum(1 for w in r["warnings"] if w["severity"] == "warning"),
            }
        return {
            "beltLength": r.get("beltLength"),
            "beltSpeed": r.get("beltSpeed"),
            "minWrap": min((p["wrapDeg"] for p in r["pulleys"]), default=None),
            "f1": t.get("f1"), "f2": t.get("f2"), "f0": t.get("f0"),
            "pmax": t.get("pmax"),
            "errors": sum(1 for w in r["warnings"] if w["severity"] == "error"),
            "warns": sum(1 for w in r["warnings"] if w["severity"] == "warning"),
        }

    ma, mb = metrics(a), metrics(b)
    diffs = []
    labels = {"beltLength": "带长 mm", "beltSpeed": "带速 m/s",
              "minWrap": "最小包角 °", "f1": "紧边张力 N", "f2": "松边张力 N",
              "f0": "初拉力 N", "pmax": "最大传递功率 kW",
              "errors": "错误数", "warns": "告警数"}
    for k, label in labels.items():
        va, vb = ma[k], mb[k]
        d = None
        if va is not None and vb is not None:
            d = vb - va
        diffs.append({"key": k, "label": label, "a": va, "b": vb, "delta": d})
    return {"a": a, "b": b, "metricsA": ma, "metricsB": mb, "diffs": diffs}
