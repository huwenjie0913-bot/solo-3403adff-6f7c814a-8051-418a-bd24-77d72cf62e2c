"""多级传动模式测试：共享轴转速传播、闭环矛盾、轴扭矩与径向合力。"""
import copy
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from belt import analyzer as az  # noqa: E402

GLOBALS = {"inputRpm": 1440, "powerKw": 5.0, "friction": 0.3, "allowTension": 2000,
           "minWrapDeg": 120, "safetyGap": 15, "tensionerTravel": 60,
           "stretchRate": 0.01, "efficiency": 0.96}


def mp(i, n, k, x, y, d, shaft, **kw):
    p = {"id": i, "name": n, "kind": k, "x": x, "y": y, "diameter": d,
         "shaftId": shaft, "locked": False, "dir": 1}
    p.update(kw)
    return p


def two_stage():
    return {
        "globals": dict(GLOBALS),
        "pulleys": [
            mp("p1", "电机轮", "driver", 0, 0, 160, "s1"),
            mp("p2", "中间大带轮", "driven", 600, 0, 320, "s2"),
            mp("p3", "中间小带轮", "driver", 600, 260, 140, "s2"),
            mp("p4", "工作轴轮", "driven", 1200, 260, 280, "s3",
               targetRpm=360, targetDir=1),
        ],
        "obstacles": [],
        "shafts": [
            {"id": "s1", "name": "电机轴", "x": 0, "y": 0, "allowTorque": 60,
             "allowRadial": 3000},
            {"id": "s2", "name": "中间轴", "x": 600, "y": 0, "allowTorque": 120,
             "allowRadial": 3000},
            {"id": "s3", "name": "工作轴", "x": 1200, "y": 260, "allowTorque": 300,
             "allowRadial": 3000},
        ],
        "loops": [
            {"id": "L1", "name": "回路1", "order": ["p1", "p2"],
             "crossedEdges": {}, "isInput": True, "powerKw": 5.0, "efficiency": 0.96},
            {"id": "L2", "name": "回路2", "order": ["p3", "p4"],
             "crossedEdges": {}, "powerKw": 4.8, "efficiency": 0.96},
        ],
        "route": {"order": ["p1", "p2"], "crossedEdges": {}},
    }


def shafts(r):
    return {s["id"]: s for s in r["shafts"]}


def test_mode_and_propagation():
    r = az.analyze(two_stage())
    assert r["mode"] == "multi"
    assert r["ok"], [(w["code"], w["message"]) for w in r["warnings"]]
    sm = shafts(r)
    assert abs(sm["s1"]["rpm"] - 1440) < 1e-9
    assert abs(sm["s2"]["rpm"] - 720) < 1e-9
    assert abs(sm["s3"]["rpm"] - 360) < 1e-9
    # 同一共享轴上的两个轮转速相同
    pr = {p["id"]: p for p in r["pulleys"]}
    assert abs(pr["p2"]["rpm"] - pr["p3"]["rpm"]) < 1e-9
    # 中间轴同时属于两条回路
    assert set(sm["s2"]["loops"]) == {"L1", "L2"}
    # 下游回路回显输入转速 = 共享轴转速
    l2 = next(l for l in r["loops"] if l["id"] == "L2")
    assert abs(l2["inputRpm"] - 720) < 1e-9
    print("multi propagation OK:", {k: round(v["rpm"], 1) for k, v in sm.items()})


def test_torque_and_radial():
    r = az.analyze(two_stage())
    sm = shafts(r)
    # T = 9550 P/n：电机轴 33.2；工作轴 9550*4.608/360 = 122.2
    assert abs(sm["s1"]["inTorque"] - 9550.0 * 5.0 / 1440) < 1e-6
    assert abs(sm["s3"]["outTorque"] - 9550.0 * 4.8 * 0.96 / 360) < 1e-6
    # 中间轴：L1 从动端输出功率 5×0.96=4.8 kW，与 L2 主动端连续
    assert abs(sm["s2"]["torque"] - 9550.0 * 4.8 / 720) < 1e-6
    assert abs(sm["s2"]["inTorque"] - sm["s2"]["outTorque"]) < 1e-9
    # 共享轴径向合力 > 任一单回路分量（两回路方向不同）
    assert sm["s2"]["radial"] > 1
    assert all(s["radial"] <= s["allowRadial"] for s in r["shafts"])
    print("torque/radial OK:",
          {k: (round(v["inTorque"], 1), round(v["outTorque"], 1),
               round(v["radial"], 0)) for k, v in sm.items()})


def test_speed_conflict_on_closed_loop():
    s = two_stage()
    s["pulleys"][3]["diameter"] = 200  # p4: 720*140/200 = 504，目标 360
    r = az.analyze(s)
    # 目标偏差只触发 RPM_MISMATCH（两个回路只在 s2 共享，工作轴末端不构成闭环）
    codes = [w["code"] for w in r["warnings"]]
    assert "RPM_MISMATCH" in codes
    # 两条输入回路同时对中间轴指定不同转速 -> 闭环矛盾必须指出轴和回路
    s2 = two_stage()
    s2["loops"][1]["isInput"] = True
    s2["loops"][1]["inputRpm"] = 999
    r2 = az.analyze(s2)
    w = next((x for x in r2["warnings"] if x["code"] == "SHAFT_SPEED_CONFLICT"), None)
    assert w is not None, [x["code"] for x in r2["warnings"]]
    assert "s2" in w["refs"]["shafts"]
    assert set(w["refs"]["loops"]) == {"L1", "L2"}
    print("speed conflict OK:", w["message"])


def test_crossed_stage_reverses_direction():
    s = two_stage()
    s["loops"][1]["crossedEdges"] = {"p3->p4": True, "p4->p3": True}
    r = az.analyze(s)
    pr = {p["id"]: p for p in r["pulleys"]}
    assert pr["p4"]["dir"] == -1 and pr["p4"]["rpm"] < 0
    assert any(w["code"] == "DIR_MISMATCH" for w in r["warnings"])
    print("crossed reversal OK")


def test_shaft_overload():
    s = two_stage()
    s["shafts"][2]["allowTorque"] = 50   # 实际 122.2
    s["shafts"][0]["allowRadial"] = 100  # 实际约 1000
    r = az.analyze(s)
    codes = {w["code"]: w for w in r["warnings"]}
    assert "SHAFT_TORQUE" in codes
    assert "SHAFT_RADIAL" in codes
    assert "s3" in codes["SHAFT_TORQUE"]["refs"]["shafts"]
    print("shaft overload OK")


def test_same_pulley_in_two_loops():
    s = two_stage()
    s["loops"][1]["order"] = ["p3", "p2"]  # p2 已属于 L1
    r = az.analyze(s)
    assert any(w["code"] == "PULLEY_IN_TWO_LOOPS" for w in r["warnings"])


def test_same_shaft_within_loop():
    s = two_stage()
    s["pulleys"][1]["shaftId"] = "s1"  # p1、p2 同轴同回路
    r = az.analyze(s)
    assert any(w["code"] == "SAME_SHAFT_IN_LOOP" for w in r["warnings"])


def test_legacy_single_loop_unchanged():
    # 无 loops 字段时仍走旧引擎，结构字段保持
    s = {"globals": dict(GLOBALS),
         "pulleys": [
             {"id": "A", "name": "电机轮", "kind": "driver", "x": 0, "y": 0,
              "diameter": 100, "locked": False, "dir": 1},
             {"id": "B", "name": "从动轮", "kind": "driven", "x": 500, "y": 0,
              "diameter": 200, "locked": False, "dir": 1}],
         "obstacles": [],
         "route": {"order": ["A", "B"], "crossedEdges": {}}}
    r = az.analyze(s)
    assert r.get("mode") != "multi"
    assert r["ok"] and abs(r["beltLength"] - 1476.2431) < 1e-3
    assert "shafts" not in r


if __name__ == "__main__":
    test_mode_and_propagation()
    test_torque_and_radial()
    test_speed_conflict_on_closed_loop()
    test_crossed_stage_reverses_direction()
    test_shaft_overload()
    test_same_pulley_in_two_loops()
    test_same_shaft_within_loop()
    test_legacy_single_loop_unchanged()
    print("\nALL MULTI-STAGE TESTS PASSED")
