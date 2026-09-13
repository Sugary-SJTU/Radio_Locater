"""问题 3 两种在线策略的集中配置。

题面物理常量仍保存在 ``config.constants``；本文件只定义路径搜索、粒子近似、MPC
规模及输出路径等算法参数，避免硬编码散落在策略实现中。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final

from config.constants import (
    ARENA_RADIUS_M,
    CHANNELS,
    CLEARANCE_RADIUS_M,
    INITIAL_CHANNEL,
    INITIAL_POSITION,
    RECEPTION_RADIUS_MIN_M,
)
from config.paths import (
    LOGS_DIR,
    PROBLEM3_FIGURES_DIR,
    PROBLEM3_TRAJECTORY_FIGURES_DIR,
    TABLES_DIR,
)

ROBUST_STRATEGY: Final[str] = "robust_polygon_rolling"
MPC_STRATEGY: Final[str] = "belief_mpc"
TOUR_STRATEGY: Final[str] = "integrated_bearing_tour"
DISTANCE_TOUR_STRATEGY: Final[str] = "distance_optimized_bearing_tour"
ROUTE_ALIGNED_TOUR_STRATEGY: Final[str] = "route_aligned_bearing_tour"
SAFE_CLEAR_TOUR_STRATEGY: Final[str] = "safe_clear_route_aligned_tour"
DYNAMIC_COVERAGE_TOUR_STRATEGY: Final[str] = "dynamic_coverage_route_aligned_tour"
SHARED_STRATEGY: Final[str] = "cooperative_bearing_tour"
STRATEGIES: Final[tuple[str, ...]] = (
    ROBUST_STRATEGY,
    MPC_STRATEGY,
    TOUR_STRATEGY,
    DISTANCE_TOUR_STRATEGY,
    ROUTE_ALIGNED_TOUR_STRATEGY,
    SAFE_CLEAR_TOUR_STRATEGY,
    DYNAMIC_COVERAGE_TOUR_STRATEGY,
    SHARED_STRATEGY,
)


@dataclass(frozen=True, slots=True)
class Problem3Settings:
    """一次问题 3 运行使用的全部算法参数。"""

    tour_polygon_sides: int = 6
    tour_polygon_radius_m: float = 1150.0
    tour_scan_origin: bool = True
    tour_target_radius_m: float = 150.0
    route_aligned_polygon_sides: int = 7
    route_aligned_polygon_radius_m: float = 1050.0
    tour_channel_order: str = "legacy"
    tour_endgame_mode: str = "probe"
    seed: int = 1
    channels: tuple[int, ...] = CHANNELS
    arena_radius_m: float = ARENA_RADIUS_M
    guaranteed_radius_m: float = RECEPTION_RADIUS_MIN_M
    # 保守滚动策略固定采用原点+正六边形；下方 polygon_* 参数仍供 belief_mpc
    # 的覆盖骨架参数搜索使用，确保两套策略可以独立演进。
    robust_polygon_sides: int = 6
    robust_polygon_radius_m: float = 1_200.0
    robust_polygon_rotation_deg: float = 0.0
    robust_scan_origin: bool = True
    robust_clear_radius_m: float = 19.8
    fallback_grid_step_m: float = 28.0
    scan_origin: bool = False
    polygon_sides: int = 7
    polygon_radius_m: float = 1_000.0
    polygon_rotation_deg: float = 0.0
    # 七边形rho=1000恰在连续覆盖边界上，因此默认余量为0；正式运行可提高此值。
    robustness_margin_m: float = 0.0
    optimize_polygon: bool = True
    polygon_side_candidates: tuple[int, ...] = (6, 7, 8, 9)
    polygon_radius_candidates_m: tuple[float, ...] = (
        750.0,
        800.0,
        850.0,
        900.0,
        950.0,
        1_000.0,
    )
    polygon_rotation_candidates_deg: tuple[float, ...] = (0.0, 7.5, 15.0, 22.5)
    scan_origin_candidates: tuple[bool, ...] = (False, True)
    coverage_grid_step_m: float = 40.0
    coverage_validation_step_m: float = 20.0
    localization_max_measurements: int = 6
    insertion_time_limit_s: float = 420.0
    particle_count_per_channel: int = 480
    existence_prior: float = 0.65
    direction_bin_deg: float = 5.0
    entropy_clear_threshold_bits: float = 0.2
    mean_information_gain_floor_bits: float = 0.05
    p0: float = 0.8
    g0: float = 0.5
    candidate_action_limit: int = 30
    coverage_lookahead_nodes: int = 4
    detected_candidate_count: int = 3
    horizon: int = 3
    beam_width: int = 20
    maximum_noncoverage_actions: int = 6
    start_position: tuple[float, float] = INITIAL_POSITION
    start_channel: int = INITIAL_CHANNEL
    clearance_radius_m: float = CLEARANCE_RADIUS_M


ACTION_LOG_DIR = LOGS_DIR / "problem3"
SUMMARY_DIR = TABLES_DIR / "problem3"
FORMAL_RESULTS_TABLE = SUMMARY_DIR / "formal_runs.csv"
# 后续问题 3 的论文插图保存至 pic3；机器狗运行回放保持原目录，避免破坏既有日志链接。
FIGURE_DIR = PROBLEM3_FIGURES_DIR
TRAJECTORY_FIGURE_DIR = PROBLEM3_TRAJECTORY_FIGURES_DIR
