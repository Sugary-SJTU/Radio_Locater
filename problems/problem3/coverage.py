"""问题 3 的正多边形保证覆盖、参数搜索和剩余路径优化。

连续覆盖判据与题目要求的解析式一致；离散 `U_j` 只承担在线状态记录，不能替代
解析覆盖证明。参数搜索同时报告高分辨率笛卡尔网格复核结果。
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from itertools import pairwise
from math import cos, hypot, pi, radians, sin, sqrt

import numpy as np
from numpy.typing import NDArray

from config.constants import CHANNEL_SWITCH_TIME_S, MEASUREMENT_TIME_S, ROBOT_SPEED_MPS
from problems.problem3.config import Problem3Settings

FloatArray = NDArray[np.float64]


@dataclass(frozen=True, slots=True)
class CoverageMetrics:
    """一组扫描点对目标圆域的解析与数值覆盖指标。"""

    analytic_max_distance_m: float | None
    numerical_max_distance_m: float
    coverage_ratio: float
    worst_point: tuple[float, float]
    achieved_margin_m: float
    required_margin_m: float
    valid: bool


@dataclass(frozen=True, slots=True)
class PolygonPlan:
    """一个正多边形扫描方案及基础时间估计。"""

    sides: int
    radius_m: float
    rotation_deg: float
    scan_origin: bool
    points: tuple[tuple[float, float], ...]
    route_length_m: float
    measure_count: int
    switch_count: int
    estimated_base_time_s: float
    coverage: CoverageMetrics

    def as_dict(self) -> dict[str, object]:
        """转为可写入 JSON 的普通字典。"""

        return asdict(self)


def regular_polygon_points(
    sides: int,
    radius_m: float,
    rotation_deg: float = 0.0,
    origin: tuple[float, float] = (0.0, 0.0),
    scan_origin: bool = False,
) -> tuple[tuple[float, float], ...]:
    """按 `P_m=O+rho(cos(phi+2πm/n),sin(...))` 生成扫描点。"""

    if sides < 3 or radius_m <= 0.0:
        raise ValueError("sides must be >=3 and radius_m positive")
    phi0 = radians(rotation_deg)
    vertices = tuple(
        (
            origin[0] + radius_m * cos(phi0 + 2.0 * pi * index / sides),
            origin[1] + radius_m * sin(phi0 + 2.0 * pi * index / sides),
        )
        for index in range(sides)
    )
    return ((float(origin[0]), float(origin[1])), *vertices) if scan_origin else vertices


def analytic_polygon_max_distance(
    sides: int,
    radius_m: float,
    arena_radius_m: float,
) -> float:
    """计算不扫描原点时题目给定的连续最坏覆盖距离公式。"""

    boundary_gap = sqrt(
        arena_radius_m**2
        + radius_m**2
        - 2.0 * arena_radius_m * radius_m * cos(pi / sides)
    )
    return max(radius_m, boundary_gap)


def arena_grid(radius_m: float, step_m: float) -> FloatArray:
    """生成目标圆域内的高分辨率笛卡尔验证网格。"""

    coordinates = np.arange(-radius_m, radius_m + step_m * 0.5, step_m)
    x_grid, y_grid = np.meshgrid(coordinates, coordinates)
    points = np.column_stack((x_grid.ravel(), y_grid.ravel()))
    return points[np.linalg.norm(points, axis=1) <= radius_m + 1e-9]


def validate_coverage(
    points: tuple[tuple[float, float], ...],
    sides: int,
    radius_m: float,
    scan_origin: bool,
    arena_radius_m: float,
    guaranteed_radius_m: float,
    robustness_margin_m: float,
    grid_step_m: float,
) -> CoverageMetrics:
    """用解析式和密网格共同验证1000 m保证覆盖。"""

    grid = arena_grid(arena_radius_m, grid_step_m)
    scan_points = np.asarray(points, dtype=float)
    nearest = np.min(
        np.linalg.norm(grid[:, None, :] - scan_points[None, :, :], axis=2),
        axis=1,
    )
    worst_index = int(np.argmax(nearest))
    numerical_max = float(nearest[worst_index])
    threshold = guaranteed_radius_m - robustness_margin_m
    coverage_ratio = float(np.mean(nearest <= threshold + 1e-9))
    if scan_origin:
        # 最近顶点夹角最坏为pi/n。中心与顶点等距处r=a/(2cos(pi/n))；
        # 其前最近距离为r，其后顶点距离平方为凸二次式，最大值在端点。
        crossing = min(arena_radius_m, radius_m / (2.0 * cos(pi / sides)))
        boundary = sqrt(arena_radius_m**2 + radius_m**2
                        - 2.0 * arena_radius_m * radius_m * cos(pi / sides))
        analytic = max(crossing, min(arena_radius_m, boundary))
    else:
        analytic = analytic_polygon_max_distance(sides, radius_m, arena_radius_m)
    decisive_max = analytic
    valid = decisive_max <= threshold + 1e-9 and coverage_ratio >= 1.0 - 1e-12
    return CoverageMetrics(
        analytic_max_distance_m=analytic,
        numerical_max_distance_m=numerical_max,
        coverage_ratio=coverage_ratio,
        worst_point=(float(grid[worst_index, 0]), float(grid[worst_index, 1])),
        achieved_margin_m=guaranteed_radius_m - decisive_max,
        required_margin_m=robustness_margin_m,
        valid=bool(valid),
    )


def route_length(
    points: tuple[tuple[float, float], ...],
    start: tuple[float, float] = (0.0, 0.0),
) -> float:
    """计算从当前位置依次访问所有点且不返航的路线长度。"""

    if not points:
        return 0.0
    return sum(
        hypot(second[0] - first[0], second[1] - first[1])
        for first, second in pairwise((start, *points))
    )


def evaluate_polygon_plan(
    settings: Problem3Settings,
    sides: int,
    radius_m: float,
    rotation_deg: float,
    scan_origin: bool,
) -> PolygonPlan:
    """计算单个覆盖参数组合的几何指标和 `T_base`。"""

    points = regular_polygon_points(
        sides,
        radius_m,
        rotation_deg,
        settings.start_position,
        scan_origin,
    )
    coverage = validate_coverage(
        points,
        sides,
        radius_m,
        scan_origin,
        settings.arena_radius_m,
        settings.guaranteed_radius_m,
        settings.robustness_margin_m,
        settings.coverage_validation_step_m,
    )
    length = route_length(points, settings.start_position)
    measures = len(points) * len(settings.channels)
    switches = len(points) * max(len(settings.channels) - 1, 0)
    base_time = (
        length / ROBOT_SPEED_MPS
        + measures * MEASUREMENT_TIME_S
        + switches * CHANNEL_SWITCH_TIME_S
    )
    return PolygonPlan(
        sides,
        radius_m,
        rotation_deg,
        scan_origin,
        points,
        length,
        measures,
        switches,
        base_time,
        coverage,
    )


def search_polygon_plans(
    settings: Problem3Settings,
) -> tuple[PolygonPlan, tuple[PolygonPlan, ...]]:
    """比较6--9边、多半径、多旋转角和是否扫描原点，返回最短可行方案。"""

    if not settings.optimize_polygon:
        fixed = evaluate_polygon_plan(
            settings,
            settings.polygon_sides,
            settings.polygon_radius_m,
            settings.polygon_rotation_deg,
            settings.scan_origin,
        )
        if not fixed.coverage.valid:
            raise ValueError("user-specified polygon does not satisfy guaranteed coverage")
        return fixed, (fixed,)

    plans = tuple(
        evaluate_polygon_plan(settings, sides, radius, rotation, scan_origin)
        for sides in settings.polygon_side_candidates
        for radius in settings.polygon_radius_candidates_m
        for rotation in settings.polygon_rotation_candidates_deg
        for scan_origin in settings.scan_origin_candidates
    )
    feasible = [plan for plan in plans if plan.coverage.valid]
    if not feasible:
        raise ValueError("no polygon parameter combination satisfies coverage margin")
    best = min(feasible, key=lambda plan: plan.estimated_base_time_s)
    return best, plans


def insertion_extra_length(
    previous: tuple[float, float],
    inserted: tuple[float, float],
    following: tuple[float, float],
) -> float:
    """计算在线任务插入保底路径的额外路程 `ΔL(Q)`。"""

    return (
        hypot(inserted[0] - previous[0], inserted[1] - previous[1])
        + hypot(following[0] - inserted[0], following[1] - inserted[1])
        - hypot(following[0] - previous[0], following[1] - previous[1])
    )


def optimize_remaining_route(
    current: tuple[float, float],
    remaining: list[tuple[float, float]],
) -> list[tuple[float, float]]:
    """以最近邻初始化并用2-opt缩短不返航的剩余保底路线。"""

    pool = remaining.copy()
    route: list[tuple[float, float]] = []
    cursor = current
    while pool:
        next_point = min(pool, key=lambda point: math_distance(cursor, point))
        pool.remove(next_point)
        route.append(next_point)
        cursor = next_point
    improved = True
    while improved and len(route) >= 3:
        improved = False
        baseline = route_length(tuple(route), current)
        for first in range(len(route) - 1):
            for second in range(first + 2, len(route) + 1):
                candidate = route[:first] + list(reversed(route[first:second])) + route[second:]
                candidate_length = route_length(tuple(candidate), current)
                if candidate_length + 1e-9 < baseline:
                    route = candidate
                    baseline = candidate_length
                    improved = True
    return route


def optimize_remaining_route_multistart(
    current: tuple[float, float],
    remaining: list[tuple[float, float]],
) -> list[tuple[float, float]]:
    """以每个候选首点构造开放路线，再交替执行 2-opt 与单点重插入。

    该函数只优化实际移动距离，不改变覆盖点、定位点或清除判据。节点较多时，
    多起点可降低单次最近邻初始化落入较差局部最优的风险。
    """

    if len(remaining) <= 2:
        return optimize_remaining_route(current, remaining)

    def greedy_from(first_index: int) -> list[tuple[float, float]]:
        pool = [
            point for index, point in enumerate(remaining) if index != first_index
        ]
        route = [remaining[first_index]]
        cursor = route[0]
        while pool:
            next_point = min(pool, key=lambda point: math_distance(cursor, point))
            pool.remove(next_point)
            route.append(next_point)
            cursor = next_point
        return route

    def improve(route: list[tuple[float, float]]) -> list[tuple[float, float]]:
        best = route
        while True:
            best_length = route_length(tuple(best), current)
            candidate_best = best
            candidate_length = best_length

            for first in range(len(best) - 1):
                for second in range(first + 2, len(best) + 1):
                    candidate = (
                        best[:first]
                        + list(reversed(best[first:second]))
                        + best[second:]
                    )
                    length = route_length(tuple(candidate), current)
                    if length + 1e-9 < candidate_length:
                        candidate_best = candidate
                        candidate_length = length

            for old_index, point in enumerate(best):
                shortened = best[:old_index] + best[old_index + 1:]
                for new_index in range(len(best)):
                    candidate = shortened.copy()
                    candidate.insert(new_index, point)
                    length = route_length(tuple(candidate), current)
                    if length + 1e-9 < candidate_length:
                        candidate_best = candidate
                        candidate_length = length

            if candidate_length + 1e-9 >= best_length:
                return best
            best = candidate_best

    candidates = [improve(greedy_from(index)) for index in range(len(remaining))]
    return min(candidates, key=lambda route: route_length(tuple(route), current))


def held_karp_open_path(
    start: tuple[float, float],
    points: list[tuple[float, float]],
) -> list[int]:
    """返回从 ``start`` 出发、访问全部点且不返航的精确最短顺序。

    返回值是 ``points`` 的索引序列。问题中待清除频道最多16个，因此
    ``O(K^2 2^K)`` 的 Held--Karp 动态规划可以直接用于末端清除排序。
    """

    count = len(points)
    if count <= 1:
        return list(range(count))
    costs: dict[tuple[int, int], float] = {}
    parents: dict[tuple[int, int], int] = {}
    for last, point in enumerate(points):
        costs[(1 << last, last)] = math_distance(start, point)

    for size in range(2, count + 1):
        for mask in range(1, 1 << count):
            if mask.bit_count() != size:
                continue
            for last in range(count):
                if not mask & (1 << last):
                    continue
                previous_mask = mask ^ (1 << last)
                choices = (
                    (
                        costs[(previous_mask, previous)]
                        + math_distance(points[previous], points[last]),
                        previous,
                    )
                    for previous in range(count)
                    if previous_mask & (1 << previous)
                )
                best_cost, best_previous = min(choices)
                costs[(mask, last)] = best_cost
                parents[(mask, last)] = best_previous

    full_mask = (1 << count) - 1
    last = min(range(count), key=lambda index: costs[(full_mask, index)])
    reversed_order: list[int] = []
    mask = full_mask
    while mask:
        reversed_order.append(last)
        previous_mask = mask ^ (1 << last)
        if not previous_mask:
            break
        last = parents[(mask, last)]
        mask = previous_mask
    return list(reversed(reversed_order))


def math_distance(first: tuple[float, float], second: tuple[float, float]) -> float:
    """无数组分配地计算二维欧氏距离。"""

    return hypot(second[0] - first[0], second[1] - first[1])
