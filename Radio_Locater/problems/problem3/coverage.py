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
    analytic = None
    if not scan_origin:
        analytic = analytic_polygon_max_distance(sides, radius_m, arena_radius_m)
    # 不扫描原点时连续解析式是最终证明。扫描原点没有题面给定闭式，网格节点的
    # 最大值还需加上半个网格单元的对角线，才是对单元内部任意点的连续上界。
    decisive_max = (
        analytic
        if analytic is not None
        else numerical_max + grid_step_m / sqrt(2.0)
    )
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
    # 用2-opt优化后的不返航路线估计基础移动成本，而不是按正多边形顺序机械累加。
    ordered_points = optimize_remaining_route(settings.start_position, list(points))
    length = route_length(tuple(ordered_points), settings.start_position)
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


def math_distance(first: tuple[float, float], second: tuple[float, float]) -> float:
    """无数组分配地计算二维欧氏距离。"""

    return hypot(second[0] - first[0], second[1] - first[1])
