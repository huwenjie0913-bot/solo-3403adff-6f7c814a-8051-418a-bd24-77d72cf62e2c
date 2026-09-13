"""皮带传动几何内核。

世界坐标：单位 mm，y 轴向下（与 Canvas 一致）。
角度约定：atan2 角（东=0，顺时针为正），所有角度均为弧度。
转向：dir=+1 表示画面上的顺时针（CW），-1 表示逆时针（CCW）。
"""
import math

PI = math.pi
TWO_PI = 2.0 * math.pi


# ---------- 基础向量运算 ----------
def add(a, b):
    return (a[0] + b[0], a[1] + b[1])


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def mul(a, s):
    return (a[0] * s, a[1] * s)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def cross2(a, b):
    """二维叉积标量；y 向下坐标系里正数表示 b 在 a 的顺时针一侧。"""
    return a[0] * b[1] - a[1] * b[0]


def length(a):
    return math.hypot(a[0], a[1])


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def norm(a):
    d = math.hypot(a[0], a[1])
    if d <= 1e-12:
        return (0.0, 0.0)
    return (a[0] / d, a[1] / d)


def rot(v, a):
    c, s = math.cos(a), math.sin(a)
    return (c * v[0] - s * v[1], s * v[0] + c * v[1])


def ang(v):
    return math.atan2(v[1], v[0])


def polar(c, r, a):
    return (c[0] + r * math.cos(a), c[1] + r * math.sin(a))


def ang_diff(b, a):
    """从角度 a 到角度 b 的有向差，范围 (-pi, pi]。"""
    d = (b - a + PI) % TWO_PI - PI
    return d


def cw_sweep(a, b):
    """沿顺时针方向从 a 扫到 b 的正角，范围 [0, 2pi)。"""
    return (b - a) % TWO_PI


def vel_at(c, r, p, direction):
    """轮心 c、半径 r、转向 direction(+1 画面顺时针 CW) 的轮子上，点 p 的速度向量。

    y 向下坐标系中视觉顺时针 = atan2 角增大：v = ω·(-ry, rx)。
    """
    rx, ry = p[0] - c[0], p[1] - c[1]
    return (-direction * ry, direction * rx)


# ---------- 圆-圆公切线 ----------
def common_tangents(c1, r1, c2, r2, crossed):
    """返回两圆的全部公切线线段。

    crossed=False 外公切线（2 条），True 内公切线（2 条）。
    返回 [((p1,p2), q), ...]，q 标识切线方向 ±1：
      外切：沿 c1->c2 看，q=+1 在右侧（y 向下即下侧）
      内切：q=+1 为两圆心连线下方的内切线
    """
    dx, dy = c2[0] - c1[0], c2[1] - c1[1]
    d = math.hypot(dx, dy)
    if d <= 1e-9:
        return []
    u = (dx / d, dy / d)
    base = ang(u)
    results = []
    if not crossed:
        rr = r2 - r1
        if abs(rr) > d:
            return []  # 一个圆完全包住另一个且内切以上，无外切线
        a = math.asin(max(-1.0, min(1.0, rr / d)))
        for q in (1, -1):
            # 带段向量 d·u+(r2-r1)n 与 n 垂直 => u·n=(r1-r2)/d
            # => 法向角 t = base ±(π/2+α)
            t = base + q * (PI / 2 + a)
            n = (math.cos(t), math.sin(t))
            p1 = (c1[0] + r1 * n[0], c1[1] + r1 * n[1])
            p2 = (c2[0] + r2 * n[0], c2[1] + r2 * n[1])
            results.append(((p1, p2), q))
    else:
        if d < r1 + r2 - 1e-9:
            return []  # 圆相交，无内切线
        rr = r1 + r2
        a = math.asin(max(-1.0, min(1.0, rr / d))) if rr <= d else PI / 2
        for q in (1, -1):
            # 两圆切点半径反向 n2=-n1；带段 d·u-r2·n2-r1·n1 = d·u+(r2-r1)n1
            # 与 n1 垂直 => u·n1=(r1+r2)/d => t=base±(π/2-α)
            t = base + q * (PI / 2 - a)
            n1 = (math.cos(t), math.sin(t))
            n2 = (-n1[0], -n1[1])
            p1 = (c1[0] + r1 * n1[0], c1[1] + r1 * n1[1])
            p2 = (c2[0] + r2 * n2[0], c2[1] + r2 * n2[1])
            results.append(((p1, p2), q))
    return results


# ---------- 线段 / 矩形 距离与相交 ----------
def point_segment_distance(p, a, b):
    ab = sub(b, a)
    denom = dot(ab, ab)
    if denom <= 1e-12:
        return dist(p, a), 0.0
    t = max(0.0, min(1.0, dot(sub(p, a), ab) / denom))
    q = (a[0] + ab[0] * t, a[1] + ab[1] * t)
    return dist(p, q), t


def segments_intersect(a, b, c, d):
    """严格相交（端点接触不算）。"""
    r = sub(b, a)
    s = sub(d, c)
    denom = cross2(r, s)
    if abs(denom) < 1e-12:
        return False
    t = cross2(sub(c, a), s) / denom
    u = cross2(sub(c, a), r) / denom
    eps = 1e-9
    return eps < t < 1 - eps and eps < u < 1 - eps


def segment_min_distance(a, b, c, d):
    infos = (
        point_segment_distance(a, c, d) + (0,),
        point_segment_distance(b, c, d) + (0,),
        point_segment_distance(c, a, b) + (1,),
        point_segment_distance(d, a, b) + (1,),
    )
    if segments_intersect(a, b, c, d):
        return 0.0
    return min(x[0] for x in infos)


def obb_corners(o):
    cx, cy = o["x"], o["y"]
    w, h = o["w"], o["h"]
    a = math.radians(o.get("rot", 0.0))
    c, s = math.cos(a), math.sin(a)
    local = ((-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2))
    return [(cx + x * c - y * s, cy + x * s + y * c) for x, y in local]


def point_in_obb(p, o, margin=0.0):
    a = math.radians(o.get("rot", 0.0))
    c, s = math.cos(-a), math.sin(-a)
    dx, dy = p[0] - o["x"], p[1] - o["y"]
    lx = dx * c - dy * s
    ly = dx * s + dy * c
    return abs(lx) <= o["w"] / 2 + margin and abs(ly) <= o["h"] / 2 + margin


def segment_obb_gap(a, b, o):
    """线段到旋转矩形边界的最小间隙；线段在矩形内部（或穿越）返回 0。"""
    pts = obb_corners(o)
    edges = [(pts[i], pts[(i + 1) % 4]) for i in range(4)]
    if point_in_obb(a, o) or point_in_obb(b, o):
        return 0.0
    # 与边界相交判定（点在矩形内的情形上面已覆盖）
    for c, d in edges:
        if segments_intersect(a, b, c, d):
            return 0.0
    return min(segment_min_distance(a, b, c, d) for c, d in edges)


def circle_obb_overlap(c, r, o):
    """圆与 OBB 是否重叠（相交或内切）。

    把圆心变换到 OBB 局部坐标，求矩形（含边界）上离圆心最近的点，
    比较其距离与半径。注意不能直接用 point_in_obb(margin=r)：
    该判定沿局部轴各扩 r，相当于一个更大的矩形，会在角点附近误判。
    """
    a = math.radians(o.get("rot", 0.0))
    cw, sw = math.cos(-a), math.sin(-a)
    dx, dy = c[0] - o["x"], c[1] - o["y"]
    lx, ly = dx * cw - dy * sw, dx * sw + dy * cw
    hx, hy = o["w"] / 2.0, o["h"] / 2.0
    qx = max(abs(lx) - hx, 0.0)
    qy = max(abs(ly) - hy, 0.0)
    return qx * qx + qy * qy < r * r - 1e-12


def circle_overlap(c1, r1, c2, r2):
    """返回两圆重叠深度（负值表示最小间隙）。"""
    d = dist(c1, c2)
    return r1 + r2 - d


def arc_polyline(c, r, a0, sweep_cw, steps=None):
    """按顺时针 sweep_cw 把圆弧离散为折线。"""
    if steps is None:
        steps = max(8, int(math.degrees(sweep_cw) / 3.0))
    pts = []
    for i in range(steps + 1):
        a = a0 + sweep_cw * i / steps
        pts.append(polar(c, r, a))
    return pts


def polyline_min_distance_to_obb(poly, o):
    best = float("inf")
    for i in range(len(poly) - 1):
        best = min(best, segment_obb_gap(poly[i], poly[i + 1], o))
        if best == 0.0:
            return 0.0
    return best


def polyline_min_distance_to_polyline(p1, p2):
    best = float("inf")
    for i in range(len(p1) - 1):
        a, b = p1[i], p1[i + 1]
        for j in range(len(p2) - 1):
            best = min(best, segment_min_distance(a, b, p2[j], p2[j + 1]))
            if best == 0.0:
                return 0.0
    return best
