"""问题 2 的第二检测点粗筛与局部细化算法。

实现论文第 6 节的直径增益路线：先按可移动性、最远接收范围、检测概率及交会角
质量筛选候选点，再在高质量区域计算期望对数直径缩减，最后在 5% 近优集合中选择
离第一个检测点最近的位置。
"""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, radians, sin

import numpy as np
from numpy.typing import NDArray

from problems.problem1.model import BearingMeasurement
from problems.problem2.config import (
    ANGLE_ERROR_DEG,
    ARENA_POLYGON_VERTICES,
    BEARING_ERROR_SAMPLES_DEG,
    COARSE_CANDIDATE_STEP_M,
    DIAMETER_FLOOR_M,
    DOMAIN_RADIUS_M,
    GUARANTEED_RECEPTION_RADIUS_M,
    MAX_RECEPTION_RADIUS_M,
    MIN_DETECTION_PROBABILITY,
    MIN_GEOMETRY_SCORE,
    MIN_SECOND_POINT_DISTANCE_M,
    NEAR_OPTIMAL_RELATIVE_GAP,
    POSTERIOR_GRID_STEP_M,
    REFINE_RADIUS_M,
    REFINE_STEP_M,
)
from radio_locator.geometry import (
    bearing_deg,
    clip_polygon_left,
    convex_hull,
    intersect_bearing_wedges,
    rotating_calipers_diameter,
)

FloatArray = NDArray[np.float64]


@dataclass(frozen=True, slots=True)
class PosteriorGrid:
    """首次检测后干扰源位置的离散后验。"""

    points: FloatArray
    weights: FloatArray
    distances_from_first: FloatArray
    first_region: FloatArray


@dataclass(frozen=True, slots=True)
class CandidateScore:
    """一个第二检测点的粗筛指标和细化目标值。"""

    point: tuple[float, float]
    reachable: bool
    detection_probability: float
    geometry_score: float
    feasible: bool
    proxy_score: float
    expected_log_diameter_gain: float | None = None


@dataclass(frozen=True, slots=True)
class SelectionResult:
    """第二检测点搜索的全部可复现结果。"""

    first_measurement: BearingMeasurement
    posterior: PosteriorGrid
    coarse_scores: tuple[CandidateScore, ...]
    coarse_refinement_centers: tuple[CandidateScore, ...]
    fine_scores: tuple[CandidateScore, ...]
    excellent_scores: tuple[CandidateScore, ...]
    excellent_region_hulls: tuple[FloatArray, ...]
    maximum_score: CandidateScore
    selected: CandidateScore


@dataclass(frozen=True, slots=True)
class SecondRegionDiameterSample:
    """固定 M2 后，一个干扰源网格假设和测向误差对应的 S2 直径。"""

    source_point: tuple[float, float]
    source_posterior_weight: float
    conditional_detection_probability: float
    bearing_error_deg: float
    second_bearing_deg: float
    diameter_m: float
    integration_weight: float


def reception_probability(distance_m: float | FloatArray) -> float | FloatArray:
    """按论文式 (17) 计算收到信号的概率 p(d)。

    假定未知有效接收半径在 [1000, 1500] m 上服从均匀分布。
    """

    distance = np.asarray(distance_m, dtype=float)
    transition_width = (
        MAX_RECEPTION_RADIUS_M - GUARANTEED_RECEPTION_RADIUS_M
    )
    probability = np.where(
        distance <= GUARANTEED_RECEPTION_RADIUS_M,
        1.0,
        np.where(
            distance < MAX_RECEPTION_RADIUS_M,
            (MAX_RECEPTION_RADIUS_M - distance) / transition_width,
            0.0,
        ),
    )
    if np.ndim(distance_m) == 0:
        return float(probability)
    return probability


def _disk_grid(radius: float, step: float) -> FloatArray:
    """生成圆域内的规则网格点。"""

    coordinates = np.arange(-radius, radius + step * 0.5, step)
    x_grid, y_grid = np.meshgrid(coordinates, coordinates)
    points = np.column_stack((x_grid.ravel(), y_grid.ravel()))
    return points[np.linalg.norm(points, axis=1) <= radius + 1e-9]


def _points_in_convex_polygon(
    points: FloatArray,
    polygon: FloatArray,
) -> NDArray[np.bool_]:
    """判断多个点是否位于逆时针凸多边形内部或边界上。"""

    mask = np.ones(len(points), dtype=bool)
    for start, end in zip(polygon, np.roll(polygon, -1, axis=0), strict=True):
        edge = end - start
        sides = edge[0] * (points[:, 1] - start[1]) - edge[1] * (
            points[:, 0] - start[0]
        )
        mask &= sides >= -1e-8
    return mask


def build_posterior_grid(
    first_measurement: BearingMeasurement,
    grid_step_m: float = POSTERIOR_GRID_STEP_M,
) -> PosteriorGrid:
    """在首次误差扇形中建立按 p(d1) 加权的离散后验。"""

    first_region = intersect_bearing_wedges(
        [first_measurement.station],
        [first_measurement.bearing_deg],
        ANGLE_ERROR_DEG,
        DOMAIN_RADIUS_M,
        ARENA_POLYGON_VERTICES,
    )
    grid = _disk_grid(DOMAIN_RADIUS_M, grid_step_m)
    inside = _points_in_convex_polygon(grid, first_region)
    points = grid[inside]
    first = np.asarray(first_measurement.station, dtype=float)
    distances = np.linalg.norm(points - first, axis=1)
    likelihood = np.asarray(reception_probability(distances), dtype=float)
    # X 与检测点完全重合时方向向量为零，示向度没有定义。该单点在连续区域中为
    # 零测度，密网格偶然命中时必须排除，避免后续生成伪造的 0 m 定位直径。
    positive = (likelihood > 0.0) & (distances > 1e-9)
    points = points[positive]
    distances = distances[positive]
    likelihood = likelihood[positive]
    if not len(points):
        raise ValueError("first measurement produces no posterior grid points")
    weights = likelihood / np.sum(likelihood)
    return PosteriorGrid(points, weights, distances, first_region)


def evaluate_candidate(
    point: tuple[float, float],
    first_measurement: BearingMeasurement,
    posterior: PosteriorGrid,
) -> CandidateScore:
    """计算论文粗筛中的可接收性、检测概率和交会角质量。"""

    candidate = np.asarray(point, dtype=float)
    first = np.asarray(first_measurement.station, dtype=float)
    moved_far_enough = (
        np.linalg.norm(candidate - first) > MIN_SECOND_POINT_DISTANCE_M
    )
    second_distances = np.linalg.norm(posterior.points - candidate, axis=1)
    reachable = bool(np.min(second_distances) <= MAX_RECEPTION_RADIUS_M)

    joint_probability = np.asarray(
        reception_probability(
            np.maximum(posterior.distances_from_first, second_distances)
        ),
        dtype=float,
    )
    first_probability = np.asarray(
        reception_probability(posterior.distances_from_first),
        dtype=float,
    )
    conditional = np.divide(
        joint_probability,
        first_probability,
        out=np.zeros_like(joint_probability),
        where=first_probability > 0.0,
    )
    detection_probability = float(np.dot(posterior.weights, conditional))

    vector_first = first - posterior.points
    vector_second = candidate - posterior.points
    denominator = np.linalg.norm(vector_first, axis=1) * np.linalg.norm(
        vector_second,
        axis=1,
    )
    cross = np.abs(
        vector_first[:, 0] * vector_second[:, 1]
        - vector_first[:, 1] * vector_second[:, 0]
    )
    sine_angle = np.divide(
        cross,
        denominator,
        out=np.zeros_like(cross),
        where=denominator > 1e-12,
    )
    if detection_probability > 0.0:
        geometry_score = float(
            np.dot(posterior.weights * conditional, sine_angle)
            / detection_probability
        )
    else:
        geometry_score = 0.0

    feasible = bool(
        moved_far_enough
        and reachable
        and detection_probability >= MIN_DETECTION_PROBABILITY
        and geometry_score >= MIN_GEOMETRY_SCORE
    )
    proxy_score = detection_probability * geometry_score
    return CandidateScore(
        point=(float(point[0]), float(point[1])),
        reachable=reachable,
        detection_probability=detection_probability,
        geometry_score=geometry_score,
        feasible=feasible,
        proxy_score=proxy_score,
    )


def _add_wedge_to_polygon(
    polygon: FloatArray,
    station: tuple[float, float],
    measured_bearing_deg: float,
) -> FloatArray:
    """在已有定位区域上追加一次 ±1° 示向度半平面约束。"""

    anchor = np.asarray(station, dtype=float)
    lower = radians(measured_bearing_deg - ANGLE_ERROR_DEG)
    upper = radians(measured_bearing_deg + ANGLE_ERROR_DEG)
    lower_direction = np.array([cos(lower), sin(lower)])
    upper_direction = np.array([cos(upper), sin(upper)])
    clipped = clip_polygon_left(polygon, anchor, lower_direction)
    return clip_polygon_left(clipped, anchor, -upper_direction)


def expected_diameter_gain(
    candidate: CandidateScore,
    first_measurement: BearingMeasurement,
    posterior: PosteriorGrid,
) -> float:
    """近似论文式 (37) 的期望对数直径缩减。"""

    if not candidate.feasible:
        return float("-inf")
    # 对首次扇形内的全部离散位置 X_k 求和，不使用预设真值或位置抽样。
    points = posterior.points
    weights = posterior.weights
    first_distances = posterior.distances_from_first
    first_diameter = rotating_calipers_diameter(posterior.first_region).distance
    candidate_point = np.asarray(candidate.point, dtype=float)
    second_distances = np.linalg.norm(points - candidate_point, axis=1)
    joint = np.asarray(
        reception_probability(np.maximum(first_distances, second_distances)),
        dtype=float,
    )
    first_probability = np.asarray(reception_probability(first_distances), dtype=float)
    conditional = np.divide(
        joint,
        first_probability,
        out=np.zeros_like(joint),
        where=first_probability > 0.0,
    )

    expected_gain = 0.0
    error_weight = 1.0 / len(BEARING_ERROR_SAMPLES_DEG)
    for hypothesis, weight, detect_probability in zip(
        points,
        weights,
        conditional,
        strict=True,
    ):
        true_bearing = bearing_deg(candidate.point, hypothesis)
        for error in BEARING_ERROR_SAMPLES_DEG:
            second_region = _add_wedge_to_polygon(
                posterior.first_region,
                candidate.point,
                true_bearing + error,
            )
            if len(second_region) == 0:
                continue
            second_diameter = rotating_calipers_diameter(second_region).distance
            gain = np.log2(
                first_diameter / max(second_diameter, DIAMETER_FLOOR_M)
            )
            expected_gain += float(weight * detect_probability * error_weight * gain)
    return expected_gain


def _refinement_grid(coarse_centers: list[CandidateScore]) -> FloatArray:
    """生成论文式 (53) 中各粗网格近优分量峰值附近的细网格。"""

    offsets = np.arange(-REFINE_RADIUS_M, REFINE_RADIUS_M + 1e-9, REFINE_STEP_M)
    local_x, local_y = np.meshgrid(offsets, offsets)
    local_offsets = np.column_stack((local_x.ravel(), local_y.ravel()))
    blocks = [local_offsets + np.asarray(score.point) for score in coarse_centers]
    points = np.unique(np.vstack(blocks), axis=0)
    return points[np.linalg.norm(points, axis=1) <= DOMAIN_RADIUS_M + 1e-9]


def _connected_score_components(
    scores: list[CandidateScore],
    grid_step_m: float,
) -> list[list[CandidateScore]]:
    """按规则网格八邻域划分候选点连通分量。"""

    if not scores:
        return []
    points = np.asarray([score.point for score in scores], dtype=float)
    neighbour_distance = grid_step_m * np.sqrt(2.0) + 1e-8
    unvisited = set(range(len(points)))
    components: list[list[CandidateScore]] = []
    while unvisited:
        seed = unvisited.pop()
        component_indices = [seed]
        frontier = [seed]
        while frontier:
            current = frontier.pop()
            neighbours = [
                index
                for index in unvisited
                if np.linalg.norm(points[index] - points[current])
                <= neighbour_distance
            ]
            for index in neighbours:
                unvisited.remove(index)
                component_indices.append(index)
                frontier.append(index)
        components.append([scores[index] for index in component_indices])
    return components


def _build_excellent_region_hulls(
    scores: list[CandidateScore],
) -> tuple[FloatArray, ...]:
    """把 5% 近优网格点连成若干区域，并返回各区域的凸包边界。

    近优集合在一般输入下仍可能不连通。这里按八邻域（含少量浮点
    容差）划分连通分量；每个分量的凸包用于绘图和输出坐标范围。凸包只概括离散
    网格范围，不把边界宣称为连续优化的精确等值线。
    """

    if not scores:
        return ()
    points = np.asarray([score.point for score in scores], dtype=float)
    neighbour_distance = REFINE_STEP_M * np.sqrt(2.0) + 1e-8
    unvisited = set(range(len(points)))
    components: list[FloatArray] = []
    while unvisited:
        seed = unvisited.pop()
        component = [seed]
        frontier = [seed]
        while frontier:
            current = frontier.pop()
            neighbours = [
                index
                for index in unvisited
                if np.linalg.norm(points[index] - points[current])
                <= neighbour_distance
            ]
            for index in neighbours:
                unvisited.remove(index)
                component.append(index)
                frontier.append(index)
        components.append(convex_hull(points[component]))
    return tuple(components)


def select_second_point(
    first_measurement: BearingMeasurement,
) -> SelectionResult:
    """运行全域粗筛、局部细化和近优距离择优，返回第二检测点。"""

    posterior = build_posterior_grid(first_measurement)
    coarse_points = _disk_grid(DOMAIN_RADIUS_M, COARSE_CANDIDATE_STEP_M)
    screened_coarse = tuple(
        evaluate_candidate(tuple(point), first_measurement, posterior)
        for point in coarse_points
    )
    feasible_coarse = [score for score in screened_coarse if score.feasible]
    if not feasible_coarse:
        raise ValueError("no candidate passes the coarse screening thresholds")
    # 对照论文式 (52)：粗筛通过后直接计算所选目标 JD，而不是用 Pdet*Gγ
    # 代理排序。未通过粗筛的点保持目标值为空，便于结果表区分筛选与优化阶段。
    coarse_with_gain: list[CandidateScore] = []
    for score in screened_coarse:
        gain = (
            expected_diameter_gain(score, first_measurement, posterior)
            if score.feasible
            else None
        )
        coarse_with_gain.append(
            CandidateScore(
                point=score.point,
                reachable=score.reachable,
                detection_probability=score.detection_probability,
                geometry_score=score.geometry_score,
                feasible=score.feasible,
                proxy_score=score.proxy_score,
                expected_log_diameter_gain=gain,
            )
        )
    coarse_scores = tuple(coarse_with_gain)
    coarse_best_gain = max(
        score.expected_log_diameter_gain or 0.0
        for score in coarse_scores
        if score.feasible
    )
    coarse_near_optimal = [
        score
        for score in coarse_scores
        if score.feasible
        and (score.expected_log_diameter_gain or 0.0)
        >= (1.0 - NEAR_OPTIMAL_RELATIVE_GAP) * coarse_best_gain
    ]
    coarse_components = _connected_score_components(
        coarse_near_optimal,
        COARSE_CANDIDATE_STEP_M,
    )
    # 粗网格可能在扇形两侧产生近似对称的多个峰。若只细化唯一全局第一名，网格
    # 扰动就可能漏掉另一侧，并错误改变式 (59) 的最近点选择。因此每个粗近优
    # 连通分量保留一个目标函数峰值作为细化中心。
    coarse_centers = [
        max(
            component,
            key=lambda score: score.expected_log_diameter_gain
            or float("-inf"),
        )
        for component in coarse_components
    ]

    refined_points = _refinement_grid(coarse_centers)
    refined_proxy = [
        evaluate_candidate(tuple(point), first_measurement, posterior)
        for point in refined_points
    ]
    feasible_refined = [score for score in refined_proxy if score.feasible]
    feasible_refined.sort(key=lambda score: score.proxy_score, reverse=True)
    fine_scores: list[CandidateScore] = []
    # 论文的精筛目标应用于全部局部细网格可行点。这里不再按代理得分截断，避免
    # 某个真正的期望直径增益优良点在计算目标函数前被误删。
    for score in feasible_refined:
        gain = expected_diameter_gain(score, first_measurement, posterior)
        fine_scores.append(
            CandidateScore(
                point=score.point,
                reachable=score.reachable,
                detection_probability=score.detection_probability,
                geometry_score=score.geometry_score,
                feasible=score.feasible,
                proxy_score=score.proxy_score,
                expected_log_diameter_gain=gain,
            )
        )
    if not fine_scores:
        raise ValueError("no candidate remains after local refinement")

    maximum_score = max(
        fine_scores,
        key=lambda score: score.expected_log_diameter_gain or float("-inf"),
    )
    best_gain = maximum_score.expected_log_diameter_gain or 0.0
    near_optimal = [
        score
        for score in fine_scores
        if (score.expected_log_diameter_gain or 0.0)
        >= (1.0 - NEAR_OPTIMAL_RELATIVE_GAP) * best_gain
    ]
    first = np.asarray(first_measurement.station, dtype=float)
    selected = min(
        near_optimal,
        key=lambda score: np.linalg.norm(np.asarray(score.point) - first),
    )
    excellent_region_hulls = _build_excellent_region_hulls(near_optimal)
    return SelectionResult(
        first_measurement,
        posterior,
        coarse_scores,
        tuple(coarse_centers),
        tuple(fine_scores),
        tuple(near_optimal),
        excellent_region_hulls,
        maximum_score,
        selected,
    )


def sample_selected_second_region_diameters(
    result: SelectionResult,
    posterior: PosteriorGrid | None = None,
) -> tuple[SecondRegionDiameterSample, ...]:
    """对 S1 内全部网格位置及误差样本，计算选定 M2 形成的 S2 直径。

    `integration_weight` 等于论文式 (37) 中每项的 ``π_k q_2k / n_error``；
    因而该表既能检查单个几何结果，也能直接构造检测条件下的加权直径分布。
    """

    # 可传入更密的独立后验网格，用于选点完成后的分布验证。默认仍复用选点网格。
    posterior = result.posterior if posterior is None else posterior
    candidate = np.asarray(result.selected.point, dtype=float)
    second_distances = np.linalg.norm(posterior.points - candidate, axis=1)
    joint = np.asarray(
        reception_probability(
            np.maximum(posterior.distances_from_first, second_distances)
        ),
        dtype=float,
    )
    first_probability = np.asarray(
        reception_probability(posterior.distances_from_first), dtype=float
    )
    conditional = np.divide(
        joint,
        first_probability,
        out=np.zeros_like(joint),
        where=first_probability > 0.0,
    )
    error_weight = 1.0 / len(BEARING_ERROR_SAMPLES_DEG)
    samples: list[SecondRegionDiameterSample] = []
    for source, posterior_weight, detect_probability in zip(
        posterior.points,
        posterior.weights,
        conditional,
        strict=True,
    ):
        true_bearing = bearing_deg(result.selected.point, source)
        for error in BEARING_ERROR_SAMPLES_DEG:
            measured_bearing = (true_bearing + error) % 360.0
            second_region = _add_wedge_to_polygon(
                posterior.first_region,
                result.selected.point,
                measured_bearing,
            )
            diameter = (
                rotating_calipers_diameter(second_region).distance
                if len(second_region)
                else 0.0
            )
            samples.append(
                SecondRegionDiameterSample(
                    source_point=(float(source[0]), float(source[1])),
                    source_posterior_weight=float(posterior_weight),
                    conditional_detection_probability=float(detect_probability),
                    bearing_error_deg=float(error),
                    second_bearing_deg=float(measured_bearing),
                    diameter_m=float(diameter),
                    integration_weight=float(
                        posterior_weight * detect_probability * error_weight
                    ),
                )
            )
    return tuple(samples)
