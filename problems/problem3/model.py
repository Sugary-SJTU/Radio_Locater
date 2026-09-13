"""问题 3 的全向干扰源搜索与清除策略模型。

整体采用两阶段：

1. 发现阶段：在 7 个能覆盖半径 1800 m 目标圆域的检测点依次扫描尚未发现的频道。
   对已经发现但只有一条示向度的源，如果后续检测点能够形成较好的交会角并且仍在
   接收范围内，则顺手补测一次，从而用很低的切换/检测成本换取第二阶段更短的行车
   路径。
2. 定位清除阶段：对多条示向度源直接用最小二乘交会点清除；对仍只有一条示向度的
   源，在示向度射线两侧生成对称第二点，并把它们作为“顺路前置点”插入 TSP 路径，
   避免为每个源单独往返。

本模块不发送 HTTP 请求，便于离线测试与路径成本评估。
"""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, hypot, radians, sin, sqrt

import numpy as np
from numpy.typing import NDArray

from config.constants import (
    ARENA_RADIUS_M,
    CLEARANCE_RADIUS_M,
    RECEPTION_RADIUS_MAX_M,
    RECEPTION_RADIUS_MIN_M,
)
from radio_locator.geometry import (
    bearing_deg,
    intersect_bearing_wedges,
    minimum_enclosing_circle,
)

FloatArray = NDArray[np.float64]

# ---------------------------------------------------------------------------
# 可调算法参数。它们不是题面常量，建议在演练测试中扫描调优。
# ---------------------------------------------------------------------------
# 单示向度源第二点的横向偏移。偏移越小，第二阶段绕路越少；但过小会使两条
# 示向度的交会基线过短。默认取 300 m。
SECOND_POINT_LATERAL_M: float = 300.0
# 顺手补测第二示向度时，候选点到可能源区间中点的最大距离。
EXTRA_BEARING_MAX_DISTANCE_M: float = 1000.0
# 顺手补测时，两条示向度中心线夹角正弦的最小值。越小越容易触发补测。
EXTRA_BEARING_MIN_SINE: float = 0.5
# 扫描阶段顺路清除的额外绕路上限（米）。
OPPORTUNISTIC_CLEAR_DETOUR_M: float = 250.0


@dataclass(frozen=True, slots=True)
class DetectedSource:
    """发现阶段记录的一次有效示向度。"""

    channel: int
    station: tuple[float, float]
    bearing_deg: float


@dataclass(frozen=True, slots=True)
class ClearTask:
    """一个待清除源在第二阶段中的路径与测量信息。"""

    channel: int
    clear_point: tuple[float, float]
    second_left: tuple[float, float] | None
    second_right: tuple[float, float] | None
    stations: tuple[tuple[float, float], ...]
    bearings: tuple[float, ...]


def _distance(first: tuple[float, float], second: tuple[float, float]) -> float:
    """两点欧氏距离。"""

    return hypot(float(first[0]) - float(second[0]), float(first[1]) - float(second[1]))


def discovery_centers() -> list[tuple[float, float]]:
    """返回能覆盖半径 1800 m 圆域的 7 个检测点。"""

    outer_radius = ARENA_RADIUS_M * cos(radians(30.0)) - sqrt(
        RECEPTION_RADIUS_MIN_M**2
        - (ARENA_RADIUS_M * sin(radians(30.0))) ** 2
    )
    centers = [(0.0, 0.0)]
    for index in range(6):
        angle = radians(index * 60.0)
        centers.append((outer_radius * cos(angle), outer_radius * sin(angle)))
    return centers


def _unit_vector(angle_deg: float) -> FloatArray:
    """由题目制方位角生成单位方向向量。"""

    angle = radians(angle_deg)
    return np.asarray([cos(angle), sin(angle)], dtype=float)


def _ray_disk_interval(
    origin: tuple[float, float],
    direction: FloatArray,
    disk_radius: float = ARENA_RADIUS_M,
) -> tuple[float, float] | None:
    """计算从 origin 沿 direction 前进时位于目标圆域内的距离区间。"""

    start = np.asarray(origin, dtype=float)
    coefficient_b = 2.0 * float(np.dot(start, direction))
    coefficient_c = float(np.dot(start, start)) - disk_radius * disk_radius
    discriminant = coefficient_b * coefficient_b - 4.0 * coefficient_c
    if discriminant < 0.0:
        return None
    root = sqrt(discriminant)
    first = (-coefficient_b - root) / 2.0
    second = (-coefficient_b + root) / 2.0
    lower = max(0.0, min(first, second))
    upper = max(first, second)
    if upper < 0.0:
        return None
    return lower, upper


def _possible_distance_interval(
    source: DetectedSource,
) -> tuple[float, float]:
    """利用首次检测成功和目标圆域约束，估计源到检测点的距离区间。"""

    direction = _unit_vector(source.bearing_deg)
    disk_interval = _ray_disk_interval(source.station, direction)
    if disk_interval is None:
        return 0.0, RECEPTION_RADIUS_MAX_M
    lower, upper = disk_interval
    lower = max(0.0, lower)
    upper = min(RECEPTION_RADIUS_MAX_M, upper)
    if upper < lower:
        return 0.0, RECEPTION_RADIUS_MAX_M
    return lower, upper


def _interval_endpoints(
    source: DetectedSource,
) -> tuple[tuple[float, float], tuple[float, float]]:
    """返回源可能所在射线区间在两个方向误差角为零时的端点。"""

    direction = _unit_vector(source.bearing_deg)
    lower, upper = _possible_distance_interval(source)
    station = np.asarray(source.station, dtype=float)
    start = station + lower * direction
    end = station + upper * direction
    return (float(start[0]), float(start[1])), (float(end[0]), float(end[1]))


def should_add_second_bearing(
    source: DetectedSource,
    candidate_center: tuple[float, float],
    max_distance_m: float = EXTRA_BEARING_MAX_DISTANCE_M,
    min_sine_angle: float = EXTRA_BEARING_MIN_SINE,
) -> bool:
    """判断后续检测点是否值得顺手补测一次示向度。

    由于补测失败只会损失一次 6 秒动作，这里采用较宽松的条件：候选点到射线区间
    中点不超过 ``max_distance_m``，且两条示向度中心线的夹角正弦不低于
    ``min_sine_angle``。若实际源恰好在区间远端导致补测 ``no_signal``，程序会保留
    原来的单示向度记录，不会影响正确性。
    """

    if candidate_center == source.station:
        return False
    direction = _unit_vector(source.bearing_deg)
    lower, upper = _possible_distance_interval(source)
    midpoint = np.asarray(source.station, dtype=float) + (
        (lower + upper) / 2.0
    ) * direction
    candidate = np.asarray(candidate_center, dtype=float)
    vector_to_candidate = candidate - midpoint
    denominator = float(np.linalg.norm(vector_to_candidate))
    if denominator < 1e-9:
        return False
    if denominator > max_distance_m:
        return False
    cross = direction[0] * vector_to_candidate[1] - direction[1] * vector_to_candidate[0]
    sine_angle = abs(float(cross)) / denominator
    return sine_angle >= min_sine_angle


def build_single_task(source: DetectedSource) -> ClearTask:
    """为只有一条示向度的源生成顺路补测候选点和预计清除点。"""

    direction = _unit_vector(source.bearing_deg)
    perpendicular = np.asarray([-direction[1], direction[0]], dtype=float)
    station = np.asarray(source.station, dtype=float)
    lower, upper = _possible_distance_interval(source)
    midpoint = (lower + upper) / 2.0
    half_width = (upper - lower) / 2.0
    clear = station + midpoint * direction

    available_lateral = sqrt(max(0.0, RECEPTION_RADIUS_MIN_M**2 - half_width**2))
    lateral = min(SECOND_POINT_LATERAL_M, max(20.0, available_lateral))
    second_left = tuple(float(value) for value in (clear + lateral * perpendicular))
    second_right = tuple(float(value) for value in (clear - lateral * perpendicular))
    return ClearTask(
        channel=source.channel,
        clear_point=(float(clear[0]), float(clear[1])),
        second_left=second_left,
        second_right=second_right,
        stations=(source.station,),
        bearings=(source.bearing_deg,),
    )


def build_multi_task(
    channel: int,
    measurements: list[DetectedSource],
) -> ClearTask:
    """把同一频道的多条示向度合成一个可直接交会清除的任务。"""

    stations = tuple(measurement.station for measurement in measurements)
    bearings = tuple(measurement.bearing_deg for measurement in measurements)
    clear_point = robust_bearing_intersection(list(stations), list(bearings))
    return ClearTask(
        channel=channel,
        clear_point=clear_point,
        second_left=None,
        second_right=None,
        stations=stations,
        bearings=bearings,
    )


def choose_second_point(
    task: ClearTask,
    previous: tuple[float, float],
) -> tuple[float, float] | None:
    """选择插入代价较小的一侧；多示向度任务没有第二点，返回 ``None``。"""

    if task.second_left is None or task.second_right is None:
        return None
    left_cost = _distance(previous, task.second_left) + _distance(
        task.second_left,
        task.clear_point,
    )
    right_cost = _distance(previous, task.second_right) + _distance(
        task.second_right,
        task.clear_point,
    )
    return task.second_left if left_cost <= right_cost else task.second_right


def _route_cost(
    order: list[ClearTask],
    start: tuple[float, float],
) -> float:
    """计算给定任务顺序的总移动距离。"""

    total = 0.0
    previous = start
    for task in order:
        second = choose_second_point(task, previous)
        if second is None:
            total += _distance(previous, task.clear_point)
        else:
            total += _distance(previous, second) + _distance(second, task.clear_point)
        previous = task.clear_point
    return total


def _two_opt(
    order: list[ClearTask],
    start: tuple[float, float],
) -> list[ClearTask]:
    """对任务顺序做 2-opt 局部搜索。"""

    current = list(order)
    improved = True
    best_cost = _route_cost(current, start)
    while improved:
        improved = False
        for first in range(len(current) - 1):
            for second in range(first + 1, len(current)):
                candidate = (
                    current[:first]
                    + current[first : second + 1][::-1]
                    + current[second + 1 :]
                )
                candidate_cost = _route_cost(candidate, start)
                if candidate_cost < best_cost - 1e-9:
                    current = candidate
                    best_cost = candidate_cost
                    improved = True
    return current


def order_clear_tasks(
    tasks: list[ClearTask],
    start: tuple[float, float],
) -> list[ClearTask]:
    """对多个贪心起点做最近邻构造，再用 2-opt 选出最短清除顺序。"""

    if not tasks:
        return []

    def _greedy(seed: ClearTask | None) -> list[ClearTask]:
        remaining = set(tasks)
        order: list[ClearTask] = []
        cursor = start
        if seed is not None:
            order.append(seed)
            remaining.remove(seed)
            cursor = seed.clear_point
        while remaining:
            next_task = min(
                remaining,
                key=lambda task: _distance(cursor, task.clear_point),
            )
            order.append(next_task)
            remaining.remove(next_task)
            cursor = next_task.clear_point
        return _two_opt(order, start)

    best_order = _greedy(None)
    best_cost = _route_cost(best_order, start)
    for seed in tasks:
        candidate = _greedy(seed)
        candidate_cost = _route_cost(candidate, start)
        if candidate_cost < best_cost - 1e-9:
            best_order = candidate
            best_cost = candidate_cost
    return best_order


def intersect_bearing_lines(
    stations: list[tuple[float, float]],
    bearings_deg: list[float],
) -> tuple[float, float]:
    """用最小二乘求多条示向度中心线的交点，作为真实源位置的估计。"""

    matrix = np.zeros((2, 2), dtype=float)
    vector = np.zeros(2, dtype=float)
    for station, bearing in zip(stations, bearings_deg, strict=True):
        direction = _unit_vector(bearing)
        projector = np.eye(2, dtype=float) - np.outer(direction, direction)
        point = np.asarray(station, dtype=float)
        matrix += projector
        vector += projector @ point
    if abs(np.linalg.det(matrix)) < 1e-12:
        estimate = np.mean(np.asarray(stations, dtype=float), axis=0)
    else:
        estimate = np.linalg.solve(matrix, vector)
    return float(estimate[0]), float(estimate[1])


def robust_bearing_intersection(
    stations: list[tuple[float, float]],
    bearings_deg: list[float],
    error_deg: float = 1.0,
) -> tuple[float, float]:
    """用示向度误差扇区交会的面积质心估计源位置。

    相比最小二乘中心线交点，该估计始终位于定位多边形内部，并且不会因两条示向度
    几乎平行而跑到目标区域之外，从而让清除失败后的就地补测更可靠。
    """

    try:
        polygon = intersect_bearing_wedges(
            stations,
            bearings_deg,
            error_deg,
            ARENA_RADIUS_M,
            720,
        )
    except ValueError:
        polygon = np.empty((0, 2), dtype=float)

    if len(polygon) < 3:
        return intersect_bearing_lines(stations, bearings_deg)

    shifted = np.roll(polygon, -1, axis=0)
    cross = (
        polygon[:, 0] * shifted[:, 1]
        - shifted[:, 0] * polygon[:, 1]
    )
    area = 0.5 * float(np.sum(cross))
    if abs(area) < 1e-9:
        return intersect_bearing_lines(stations, bearings_deg)
    centroid_x = float(
        np.sum((polygon[:, 0] + shifted[:, 0]) * cross)
    ) / (6.0 * area)
    centroid_y = float(
        np.sum((polygon[:, 1] + shifted[:, 1]) * cross)
    ) / (6.0 * area)
    return centroid_x, centroid_y


def clear_point_from_measurements(
    stations: list[tuple[float, float]],
    bearings_deg: list[float],
    error_deg: float = 1.0,
) -> tuple[float, float]:
    """优先使用定位区域最小包围圆圆心，否则退回定位区域面积质心。"""

    try:
        polygon = intersect_bearing_wedges(
            stations,
            bearings_deg,
            error_deg,
            ARENA_RADIUS_M,
            720,
        )
    except ValueError:
        polygon = np.empty((0, 2), dtype=float)
    if len(polygon) >= 1:
        circle = minimum_enclosing_circle(polygon)
        if circle.radius <= CLEARANCE_RADIUS_M + 1e-9:
            return float(circle.center[0]), float(circle.center[1])
    return robust_bearing_intersection(stations, bearings_deg)


def localization_circle(
    stations: list[tuple[float, float]],
    bearings_deg: list[float],
    error_deg: float = 1.0,
) -> tuple[tuple[float, float], float] | None:
    """返回当前定位区域的最小包围圆圆心和半径；区域为空时返回 ``None``。"""

    try:
        polygon = intersect_bearing_wedges(
            stations,
            bearings_deg,
            error_deg,
            ARENA_RADIUS_M,
            720,
        )
    except ValueError:
        return None
    if len(polygon) == 0:
        return None
    circle = minimum_enclosing_circle(polygon)
    return (
        (float(circle.center[0]), float(circle.center[1])),
        float(circle.radius),
    )


def bearing_from(station: tuple[float, float], target: tuple[float, float]) -> float:
    """返回从 station 指向 target 的方位角。"""

    return bearing_deg(station, target)
