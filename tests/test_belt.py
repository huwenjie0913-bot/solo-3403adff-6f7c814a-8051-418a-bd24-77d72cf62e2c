"""几何与校核引擎测试：对照开口/交叉带解析公式。"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from belt import geometry as g  # noqa: E402
from belt import analyzer as az  # noqa: E402


def mk_p(id_, name, kind, x, y, d, **kw):
    p = {"id": id_, "name": name, "kind": kind, "x": x, "y": y,
         "diameter": d, "locked": False, "dir": 1}
    p.update(kw)
    return p


GLOBALS = {"inputRpm": 1440, "powerKw": 5, "friction": 0.3, "allowTension": 2000,
           "minWrapDeg": 120, "safetyGap": 10, "tensionerTravel": 50,
           "stretchRate": 0.01}


def open_scheme():
    return {"globals": dict(GLOBALS),
            "pulleys": [mk_p("A", "电机轮", "driver", 0, 0, 100),
                        mk_p("B", "从动轮", "driven", 500, 0, 200,
                             targetRpm=720, targetDir=1)],
            "obstacles": [],
            "route": {"order": ["A", "B"], "crossedEdges": {}}}


def test_open_belt_analytic():
    d, d1, d2 = 500.0, 100.0, 200.0
    r1, r2 = d1 / 2, d2 / 2
    alpha = math.asin((r2 - r1) / d)  # 开口带 sin α = (r2-r1)/C
    L_ref = 2 * d * math.cos(alpha) + math.pi * (r1 + r2) + 2 * (r2 - r1) * alpha
    theta1 = math.pi - 2 * alpha
    theta2 = math.pi + 2 * alpha

    r = az.analyze(open_scheme())
    assert r["ok"], r["warnings"]
    assert abs(r["beltLength"] - L_ref) < 1e-6, (r["beltLength"], L_ref)
    wa = {p["id"]: p for p in r["pulleys"]}
    assert abs(wa["A"]["wrapDeg"] - math.degrees(theta1)) < 1e-7
    assert abs(wa["B"]["wrapDeg"] - math.degrees(theta2)) < 1e-7
    # 两直段长度
    seg = math.sqrt(d * d - (r2 - r1) ** 2)
    assert abs(r["edges"][0]["length"] - seg) < 1e-7
    assert abs(r["edges"][1]["length"] - seg) < 1e-7
    # 转速与方向
    assert abs(wa["B"]["rpm"] - 720.0) < 1e-9
    assert wa["A"]["dir"] == 1 and wa["B"]["dir"] == 1
    # 带速
    assert abs(r["beltSpeed"] - math.pi * 100 * 1440 / 60000) < 1e-9
    print("open belt OK: L=%.4f (ref %.4f), wraps %.2f/%.2f"
          % (r["beltLength"], L_ref, wa["A"]["wrapDeg"], wa["B"]["wrapDeg"]))


def test_crossed_belt_analytic():
    d, d1, d2 = 500.0, 100.0, 200.0
    r1, r2 = d1 / 2, d2 / 2
    alpha = math.asin((r1 + r2) / d)
    L_ref = 2 * d * math.cos(alpha) + math.pi * (r1 + r2) + 2 * (r1 + r2) * alpha
    theta = math.pi + 2 * alpha

    s = open_scheme()
    s["route"]["crossedEdges"] = {"A->B": True, "B->A": True}
    r = az.analyze(s)
    assert r["ok"], r["warnings"]
    assert abs(r["beltLength"] - L_ref) < 1e-6, (r["beltLength"], L_ref)
    wa = {p["id"]: p for p in r["pulleys"]}
    assert abs(wa["A"]["wrapDeg"] - math.degrees(theta)) < 1e-7
    assert abs(wa["B"]["wrapDeg"] - math.degrees(theta)) < 1e-7
    assert wa["B"]["dir"] == -1  # 交叉带反向
    assert abs(wa["B"]["rpm"] - -720.0) < 1e-9
    print("crossed belt OK: L=%.4f (ref %.4f), wrap %.2f"
          % (r["beltLength"], L_ref, math.degrees(theta)))


def test_wrap_warning_and_tension():
    s = open_scheme()
    s["pulleys"][0]["diameter"] = 60
    s["pulleys"][1]["diameter"] = 500
    s["pulleys"][1]["x"] = 300
    r = az.analyze(s)
    codes = [w["code"] for w in r["warnings"]]
    wa = {p["id"]: p for p in r["pulleys"]}
    assert wa["A"]["wrapDeg"] < 120, wa["A"]["wrapDeg"]
    assert "SMALL_WRAP" in codes
    # 小轮 + 高功率应触发张力问题
    s2 = open_scheme()
    s2["pulleys"][0]["diameter"] = 80
    s2["globals"]["allowTension"] = 200
    r2 = az.analyze(s2)
    codes2 = [w["code"] for w in r2["warnings"]]
    assert "TENSION" in codes2 or "POWER_LOW" in codes2
    print("wrap/tension warnings OK:", codes, codes2)


def test_overlap_and_obstacle():
    s = open_scheme()
    s["pulleys"][1]["x"] = 120
    r = az.analyze(s)
    assert any(w["code"] == "WHEEL_OVERLAP" for w in r["warnings"])

    s = open_scheme()
    # 开口带直线段位于 y=±50 之间；放一个压到带的矩形
    s["obstacles"] = [{"id": "O1", "name": "护罩", "x": 250, "y": 55,
                       "w": 300, "h": 20, "rot": 0}]
    r = az.analyze(s)
    codes = [w["code"] for w in r["warnings"]]
    assert "GUARD_GAP" in codes or "BELT_COLLISION" in codes, codes
    # 远离后应无告警
    s["obstacles"][0]["y"] = 120
    r2 = az.analyze(s)
    assert not any(w["code"] in ("GUARD_GAP", "BELT_COLLISION")
                   for w in r2["warnings"])
    print("obstacle checks OK")


def test_three_wheel_serpentine():
    # 三轮全 CW 外包三角形：所有公切线为外切线，无交叉段
    s = {"globals": dict(GLOBALS, minWrapDeg=60),
         "pulleys": [mk_p("A", "主动", "driver", 0, 0, 200),
                     mk_p("B", "从动", "driven", 700, 100, 300),
                     mk_p("C", "张紧", "tensioner", 300, -450, 100)],
         "obstacles": [],
         "route": {"order": ["A", "B", "C"], "crossedEdges": {}}}
    r = az.analyze(s)
    assert r["ok"], [(w["code"], w["message"]) for w in r["warnings"]]
    # 外切线包围凸包：每个轮取外侧优弧，三轮包角和 = 360°+360° = 720°
    total = sum(p["wrapDeg"] for p in r["pulleys"])
    assert abs(total - 720.0) < 1e-6, total
    assert all(p["dir"] == 1 for p in r["pulleys"])
    print("serpentine OK: wraps", [(p["id"], round(p["wrapDeg"], 2))
                                   for p in r["pulleys"]])


def test_rpm_target_mismatch():
    s = open_scheme()
    s["pulleys"][1]["targetRpm"] = 900  # 实际 720
    r = az.analyze(s)
    assert any(w["code"] == "RPM_MISMATCH" for w in r["warnings"])


def test_tangent_direct():
    tans = g.common_tangents((0, 0), 50, (500, 0), 100, False)
    assert len(tans) == 2
    for (p1, p2), q in tans:
        assert abs(g.dist(p1, (0, 0)) - 50) < 1e-9
        assert abs(g.dist(p2, (500, 0)) - 100) < 1e-9
        s = g.norm(g.sub(p2, p1))
        n1 = g.norm(g.sub(p1, (0, 0)))
        n2 = g.norm(g.sub(p2, (500, 0)))
        # 外切线：带段与切点半径垂直，两切点半径同向平行
        assert abs(g.dot(s, n1)) < 1e-9
        assert abs(n1[0] - n2[0]) < 1e-9 and abs(n1[1] - n2[1]) < 1e-9
    # 等径轮切点恰在正上/正下方
    tans2 = g.common_tangents((0, 0), 50, (500, 0), 50, False)
    pts = sorted([(t[0][0][1], t[0][1][1]) for t in tans2])
    assert abs(pts[0][0] + 50) < 1e-9 and abs(pts[1][0] - 50) < 1e-9


def test_segment_cross_detection():
    assert g.segments_intersect((0, 0), (10, 10), (0, 10), (10, 0))
    assert not g.segments_intersect((0, 0), (10, 0), (0, 5), (10, 5))


if __name__ == "__main__":
    test_tangent_direct()
    test_segment_cross_detection()
    test_open_belt_analytic()
    test_crossed_belt_analytic()
    test_wrap_warning_and_tension()
    test_overlap_and_obstacle()
    test_three_wheel_serpentine()
    test_rpm_target_mismatch()
    print("\nALL TESTS PASSED")
