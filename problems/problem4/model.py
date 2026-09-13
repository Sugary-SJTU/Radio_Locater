"""问题 4 的定向/全向混合源发现点与路径规划。

发现阶段使用三角格点，而不是问题 3 的正六边形。格点间距不超过 1000 m 时，对
任意源位置 G 和任意发射方向 u，源所在三角形顶点中至少有一个位于源前方 180°
覆盖范围内，因此不会因定向源背面朝外而漏检。

定位与清除任务直接复用问题 3 的 ``ClearTask``、``build_single_task``、
``build_multi_task`` 和 ``order_clear_tasks``。
"""

from __future__ import annotations

from math import ceil, hypot, sqrt

from config.constants import ARENA_RADIUS_M, RECEPTION_RADIUS_MIN_M
from problems.problem4.model import ClearTask, DetectedSource, order_clear_tasks


LATTICE_SPACING_M: float = 1000.0


def _point_distance(
    first: tuple[float, float],
    second: tuple[float, float],
) -> float:
    return hypot(first[0] - second[0], first[1] - second[1])


def directional_discovery_centers() -> list[tuple[float, float]]:
    """生成覆盖目标区域所需的多方向三角格点。"""

    spacing = LATTICE_SPACING_M
    limit = ARENA_RADIUS_M + RECEPTION_RADIUS_MIN_M
    max_index = int(ceil(limit / spacing)) + 1
    points: list[tuple[float, float]] = []
    for first in range(-max_index, max_index + 1):
        for second in range(-max_index, max_index + 1):
            x = first * spacing + (second % 2) * spacing * 0.5
            y = second * spacing * sqrt(3.0) / 2.0
            if x * x + y * y <= limit * limit:
                points.append((float(x), float(y)))
    if (0.0, 0.0) not in points:
        points.append((0.0, 0.0))
    return points


def _route_length(
    points: list[tuple[float, float]],
    start: tuple[float, float],
) -> float:
    if not points:
        return 0.0
    total = _point_distance(start, points[0])
    for first, second in zip(points, points[1:]):
        total += _point_distance(first, second)
    return total


def _two_opt_points(
    points: list[tuple[float, float]],
    start: tuple[float, float],
) -> list[tuple[float, float]]:
    """对发现点顺序做最近邻 + 2-opt。"""

    remaining = set(points)
    order: list[tuple[float, float]] = []
    cursor = start
    while remaining:
        next_point = min(remaining, key=lambda point: _point_distance(cursor, point))
        order.append(next_point)
        remaining.remove(next_point)
        cursor = next_point

    improved = True
    while improved and len(order) >= 3:
        improved = False
        baseline = _route_length(order, start)
        for first in range(len(order) - 1):
            for second in range(first + 2, len(order) + 1):
                candidate = (
                    order[:first]
                    + list(reversed(order[first:second]))
                    + order[second:]
                )
                if _route_length(candidate, start) + 1e-9 < baseline:
                    order = candidate
                    baseline = _route_length(order, start)
                    improved = True
    return order


def ordered_discovery_centers(
    start: tuple[float, float],
) -> list[tuple[float, float]]:
    """返回按当前起点优化后的多方向发现点顺序。"""

    return _two_opt_points(directional_discovery_centers(), start)


__all__ = [
    "ClearTask",
    "DetectedSource",
    "LATTICE_SPACING_M",
    "directional_discovery_centers",
    "ordered_discovery_centers",
    "order_clear_tasks",
]


