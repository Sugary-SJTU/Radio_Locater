"""问题 1、2 共用的二维凸几何算法。

角度均沿用题目约定：正东为 0°、逆时针为正。凸多边形顶点使用逆时针顺序。
圆域通过高边数正多边形近似，使示向度扇形约束可以统一按半平面裁剪处理。
"""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, hypot, pi, radians, sin
from typing import Iterable, Sequence

import numpy as np
from numpy.typing import NDArray

FloatArray = NDArray[np.float64]


@dataclass(frozen=True, slots=True)
class DiameterResult:
    """凸多边形直径及其一对端点。"""

    distance: float
    first_index: int
    second_index: int
    first: FloatArray
    second: FloatArray


def cross_2d(first: FloatArray, second: FloatArray) -> float:
    """返回二维向量叉积的标量值。"""

    return float(first[0] * second[1] - first[1] * second[0])


def bearing_deg(origin: Sequence[float], target: Sequence[float]) -> float:
    """计算从 origin 指向 target 的题目制方位角，结果归一化至 [0, 360)。"""

    dx = float(target[0]) - float(origin[0])
    dy = float(target[1]) - float(origin[1])
    return float(np.degrees(np.arctan2(dy, dx)) % 360.0)


def regular_polygon(radius: float, vertex_count: int) -> FloatArray:
    """生成以原点为中心、逆时针排列的正多边形。"""

    if radius <= 0 or vertex_count < 3:
        raise ValueError("radius must be positive and vertex_count at least 3")
    angles = np.linspace(0.0, 2.0 * pi, vertex_count, endpoint=False)
    return np.column_stack((radius * np.cos(angles), radius * np.sin(angles)))


def _segment_line_intersection(
    start: FloatArray,
    end: FloatArray,
    start_side: float,
    end_side: float,
) -> FloatArray:
    """计算线段与裁剪直线的交点；side 是端点的带符号半平面值。"""

    denominator = start_side - end_side
    if abs(denominator) < 1e-15:
        return start.copy()
    ratio = start_side / denominator
    return start + ratio * (end - start)


def clip_polygon_left(
    polygon: FloatArray,
    line_point: Sequence[float],
    line_direction: Sequence[float],
    tolerance: float = 1e-9,
) -> FloatArray:
    """用有向直线左侧的闭半平面裁剪凸多边形。

    点 X 位于左半平面当且仅当 ``direction × (X - line_point) >= 0``。
    使用 Sutherland--Hodgman 算法，输出仍保持逆时针顺序。
    """

    polygon = np.asarray(polygon, dtype=float)
    if len(polygon) == 0:
        return np.empty((0, 2), dtype=float)
    anchor = np.asarray(line_point, dtype=float)
    direction = np.asarray(line_direction, dtype=float)
    if hypot(float(direction[0]), float(direction[1])) == 0:
        raise ValueError("line_direction must be non-zero")

    output: list[FloatArray] = []
    previous = polygon[-1]
    previous_side = cross_2d(direction, previous - anchor)
    previous_inside = previous_side >= -tolerance
    for current in polygon:
        current_side = cross_2d(direction, current - anchor)
        current_inside = current_side >= -tolerance
        if current_inside != previous_inside:
            output.append(
                _segment_line_intersection(
                    previous,
                    current,
                    previous_side,
                    current_side,
                )
            )
        if current_inside:
            output.append(current.copy())
        previous = current
        previous_side = current_side
        previous_inside = current_inside

    if not output:
        return np.empty((0, 2), dtype=float)
    result = np.asarray(output, dtype=float)
    # 连续裁剪可能产生相邻重复点；删除它们可避免旋转卡壳遇到零长度边。
    keep = np.ones(len(result), dtype=bool)
    if len(result) > 1:
        keep[1:] = np.linalg.norm(np.diff(result, axis=0), axis=1) > tolerance
        result = result[keep]
        if len(result) > 1 and np.linalg.norm(result[0] - result[-1]) <= tolerance:
            result = result[:-1]
    return result


def intersect_bearing_wedges(
    stations: Iterable[Sequence[float]],
    bearings_deg: Iterable[float],
    error_deg: float,
    arena_radius: float,
    arena_vertices: int = 720,
    tolerance: float = 1e-9,
) -> FloatArray:
    """求多个示向度误差扇形与目标圆域近似多边形的交集。

    每个观测生成下边界 ``theta-error`` 和上边界 ``theta+error``。可行点须位于
    下边界左侧、上边界右侧；后者等价于反向上边界的左侧。
    """

    station_list = list(stations)
    bearing_list = list(bearings_deg)
    if len(station_list) != len(bearing_list):
        raise ValueError("stations and bearings_deg must have equal lengths")
    if not station_list:
        raise ValueError("at least one measurement is required")
    if not 0.0 <= error_deg < 90.0:
        raise ValueError("error_deg must lie in [0, 90)")

    polygon = regular_polygon(arena_radius, arena_vertices)
    for station, bearing in zip(station_list, bearing_list, strict=True):
        anchor = np.asarray(station, dtype=float)
        lower = radians(float(bearing) - error_deg)
        upper = radians(float(bearing) + error_deg)
        lower_direction = np.array([cos(lower), sin(lower)], dtype=float)
        upper_direction = np.array([cos(upper), sin(upper)], dtype=float)
        polygon = clip_polygon_left(
            polygon,
            anchor,
            lower_direction,
            tolerance,
        )
        polygon = clip_polygon_left(
            polygon,
            anchor,
            -upper_direction,
            tolerance,
        )
        if len(polygon) == 0:
            break
    return polygon


def polygon_area(polygon: FloatArray) -> float:
    """用鞋带公式计算多边形面积；空集和退化图形面积为 0。"""

    polygon = np.asarray(polygon, dtype=float)
    if len(polygon) < 3:
        return 0.0
    shifted = np.roll(polygon, -1, axis=0)
    cross_sum = np.sum(
        polygon[:, 0] * shifted[:, 1] - shifted[:, 0] * polygon[:, 1]
    )
    return abs(float(cross_sum)) / 2.0


def convex_hull(points: FloatArray) -> FloatArray:
    """用 Andrew 单调链算法返回点集的逆时针凸包。

    重复点会被删除；一个点或两个点的退化输入直接按字典序返回。此函数用于把
    离散搜索得到的近优点集合概括成便于输出和绘图的边界。
    """

    unique = np.unique(np.asarray(points, dtype=float), axis=0)
    if len(unique) <= 2:
        return unique
    ordered = unique[np.lexsort((unique[:, 1], unique[:, 0]))]

    def half_hull(sequence: FloatArray) -> list[FloatArray]:
        half: list[FloatArray] = []
        for point in sequence:
            while (
                len(half) >= 2
                and cross_2d(half[-1] - half[-2], point - half[-1]) <= 1e-12
            ):
                half.pop()
            half.append(point)
        return half

    lower = half_hull(ordered)
    upper = half_hull(ordered[::-1])
    return np.asarray(lower[:-1] + upper[:-1], dtype=float)


def brute_force_diameter(polygon: FloatArray) -> DiameterResult:
    """以 O(m²) 枚举所有顶点对，主要用于校验旋转卡壳实现。"""

    polygon = np.asarray(polygon, dtype=float)
    if len(polygon) == 0:
        raise ValueError("diameter is undefined for an empty polygon")
    if len(polygon) == 1:
        return DiameterResult(0.0, 0, 0, polygon[0].copy(), polygon[0].copy())
    best_squared = -1.0
    best_pair = (0, 1)
    for first_index in range(len(polygon)):
        differences = polygon[first_index + 1 :] - polygon[first_index]
        if not len(differences):
            continue
        squared = np.einsum("ij,ij->i", differences, differences)
        local_index = int(np.argmax(squared))
        if float(squared[local_index]) > best_squared:
            best_squared = float(squared[local_index])
            best_pair = (first_index, first_index + 1 + local_index)
    first_index, second_index = best_pair
    return DiameterResult(
        best_squared**0.5,
        first_index,
        second_index,
        polygon[first_index].copy(),
        polygon[second_index].copy(),
    )


def rotating_calipers_diameter(
    polygon: FloatArray,
    tolerance: float = 1e-12,
) -> DiameterResult:
    """用旋转卡壳在线性时间内求逆时针凸多边形直径。

    对每条边移动对踵点，直到三角形面积不再增加；检查边的两个端点与当前对踵点
    的距离。输入必须是按边界顺序排列的凸多边形。
    """

    polygon = np.asarray(polygon, dtype=float)
    count = len(polygon)
    if count <= 2:
        return brute_force_diameter(polygon)

    best_squared = -1.0
    best_pair = (0, 1)
    opposite = 1
    for first_index in range(count):
        next_index = (first_index + 1) % count
        edge = polygon[next_index] - polygon[first_index]
        while True:
            next_opposite = (opposite + 1) % count
            current_area = abs(
                cross_2d(edge, polygon[opposite] - polygon[first_index])
            )
            next_area = abs(
                cross_2d(edge, polygon[next_opposite] - polygon[first_index])
            )
            if next_area > current_area + tolerance:
                opposite = next_opposite
            else:
                break
        for endpoint in (first_index, next_index):
            difference = polygon[endpoint] - polygon[opposite]
            squared = float(np.dot(difference, difference))
            if squared > best_squared:
                best_squared = squared
                best_pair = (endpoint, opposite)

    first_index, second_index = best_pair
    return DiameterResult(
        best_squared**0.5,
        first_index,
        second_index,
        polygon[first_index].copy(),
        polygon[second_index].copy(),
    )


def diameter_circle_coverage(
    polygon: FloatArray,
    diameter: DiameterResult | None = None,
    tolerance: float = 1e-9,
) -> tuple[bool, FloatArray, float, float]:
    """判断以一对直径端点为直径的圆是否覆盖凸多边形。

    返回 ``(是否覆盖, 圆心, 半径, 最大超出量)``。凸函数到圆心的最大距离在凸
    多边形顶点处取得，因此只需检查所有顶点。
    """

    polygon = np.asarray(polygon, dtype=float)
    if diameter is None:
        diameter = rotating_calipers_diameter(polygon)
    center = (diameter.first + diameter.second) / 2.0
    radius = diameter.distance / 2.0
    distances = np.linalg.norm(polygon - center, axis=1)
    excess = float(np.max(distances) - radius)
    return excess <= tolerance, center, radius, max(0.0, excess)
