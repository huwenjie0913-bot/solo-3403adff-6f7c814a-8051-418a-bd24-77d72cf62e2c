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


def build_paths(scheme, displacements=None):
    """按绕行顺序构建有向带段与轮上弧段。

    返回 dict：ok, order, wheels(id 顺序), edges[{from,to,crossed,p1,p2,q,feasible,
    score,length}], arcs[{wheel,a0,sweep,dir,r,length,wrapDeg}], dirs, length,
    fatal（无法构造切线的错误列表）
    displacements: {id: (x,y)} 临时移动轮位（张紧行程试算用）
    """
    displacements = displacements or {}
    pulleys = scheme["pulleys"]
    order = list(scheme.get("route", {}).get("order") or [p["id"] for p in pulleys])
    crossed_map = scheme.get("route", {}).get("crossedEdges", {}) or {}
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
    driver_id = next((p["id"] for p in pulleys if p.get("kind") == "driver"), order[0])
    driver_dir = 1 if pmap[driver_id].get("dir", 1) >= 0 else -1
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


def analyze(scheme):
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
