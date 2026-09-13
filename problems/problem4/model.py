"""问题四定向源保证发现所需的三角格几何模型。"""

from __future__ import annotations

import math

import numpy as np


def directional_lattice_points(
    arena_radius_m: float,
    reception_radius_m: float,
    *,
    spacing_m: float = 1_000.0,
    rotation_deg: float = 0.0,
    offset: tuple[float, float] = (0.0, 0.0),
) -> tuple[tuple[float, float], ...]:
    """生成有限三角格测站。

    无限三角格把平面剖分为边长 ``spacing_m`` 的等边三角形。当边长不超过最低
    接收半径时，三角形内任一点到三个顶点均不超过最低接收半径，且位于三顶点凸包
    内。因此任意朝向的闭半平面至少包含一个可接收测站。保留半径 ``R+s`` 内节点
    即足以覆盖半径 ``R`` 的目标圆域。
    """

    if spacing_m <= 0.0 or spacing_m > reception_radius_m + 1e-9:
        raise ValueError("directional lattice spacing must lie in (0, reception radius]")
    vertical = spacing_m * math.sqrt(3.0) / 2.0
    extent = arena_radius_m + spacing_m
    row_limit = math.ceil((extent + abs(offset[1])) / vertical) + 1
    column_limit = math.ceil((extent + abs(offset[0])) / spacing_m) + 2
    raw: list[tuple[float, float]] = []
    for row in range(-row_limit, row_limit + 1):
        y = row * vertical + offset[1]
        shift = 0.5 if row % 2 else 0.0
        for column in range(-column_limit, column_limit + 1):
            x = (column + shift) * spacing_m + offset[0]
            raw.append((x, y))

    angle = math.radians(rotation_deg)
    cosine, sine = math.cos(angle), math.sin(angle)
    rotated = [
        (cosine * x - sine * y, sine * x + cosine * y)
        for x, y in raw
    ]
    # 任何覆盖目标圆内位置的三角形顶点距原点至多 R+s；外侧节点可以安全删除。
    selected = [point for point in rotated if math.hypot(*point) <= extent + 1e-7]
    return tuple(sorted(selected, key=lambda point: (point[1], point[0])))


def directional_ring_mesh_points(
    arena_radius_m: float,
    reception_radius_m: float,
    *,
    sectors: int = 12,
    rotation_deg: float = 0.0,
    inner_radius_m: float | None = None,
) -> tuple[tuple[float, float], ...]:
    """生成中心点、内环和外接正多边形组成的短路线保证网格。

    对本题 ``R=1800,r=1000``，12扇区外环半径为 ``R/cos(15°)``，其正
    十二边形恰好外切目标圆。内环半径为1000m并与外环错开15°。中心扇形与两环
    间的所有三角形最长边均不超过1000m，因此保留与三角格相同的方向保证，却只需
    25个点且所有点都靠近目标圆域。
    """

    if sectors < 3:
        raise ValueError("ring mesh requires at least three sectors")
    half_step = math.pi / sectors
    outer_radius = arena_radius_m / math.cos(half_step)
    inner_radius = reception_radius_m if inner_radius_m is None else inner_radius_m
    if inner_radius <= 0.0:
        raise ValueError("inner ring radius must be positive")
    outer_edge = 2.0 * outer_radius * math.sin(half_step)
    cross_edge = math.sqrt(
        inner_radius**2 + outer_radius**2
        - 2.0 * inner_radius * outer_radius * math.cos(half_step)
    )
    if max(inner_radius, outer_edge, cross_edge) > reception_radius_m + 1e-7:
        raise ValueError("ring mesh triangles exceed guaranteed reception radius")
    rotation = math.radians(rotation_deg)
    step = 2.0 * math.pi / sectors
    inner = [
        (inner_radius * math.cos(rotation + index * step),
         inner_radius * math.sin(rotation + index * step))
        for index in range(sectors)
    ]
    outer = [
        (outer_radius * math.cos(rotation + (index + 0.5) * step),
         outer_radius * math.sin(rotation + (index + 0.5) * step))
        for index in range(sectors)
    ]
    return ((0.0, 0.0), *inner, *outer)


def directionally_guaranteed(
    source_position: tuple[float, float],
    stations: tuple[tuple[float, float], ...],
    reception_radius_m: float,
) -> bool:
    """验证给定位置是否被半径内测站的凸包包含。"""

    vectors = np.asarray(stations, dtype=float) - np.asarray(source_position, dtype=float)
    distances = np.linalg.norm(vectors, axis=1)
    nearby = vectors[distances <= reception_radius_m + 1e-7]
    if not len(nearby):
        return False
    if np.any(np.linalg.norm(nearby, axis=1) <= 1e-7):
        return True
    angles = np.sort(np.arctan2(nearby[:, 1], nearby[:, 0]))
    gaps = np.diff(np.r_[angles, angles[0] + 2.0 * math.pi])
    # 所有向量不被任何开半平面严格分开，等价于原点属于其凸包。
    return bool(float(np.max(gaps)) <= math.pi + 1e-9)


def validate_directional_lattice(
    stations: tuple[tuple[float, float], ...],
    arena_radius_m: float,
    reception_radius_m: float,
    sample_step_m: float = 50.0,
) -> dict[str, float | int | bool]:
    """用密集圆域网格审计有限截取没有破坏解析保证。"""

    coordinates = np.arange(-arena_radius_m, arena_radius_m + sample_step_m * 0.5, sample_step_m)
    tested = failed = 0
    for x in coordinates:
        for y in coordinates:
            if math.hypot(float(x), float(y)) > arena_radius_m + 1e-9:
                continue
            tested += 1
            failed += int(not directionally_guaranteed((float(x), float(y)), stations, reception_radius_m))
    return {
        "valid": failed == 0,
        "station_count": len(stations),
        "sample_count": tested,
        "failed_sample_count": failed,
        "sample_step_m": sample_step_m,
    }
