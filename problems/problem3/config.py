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
from config.paths import FIGURES_DIR, LOGS_DIR, TABLES_DIR

ROBUST_STRATEGY: Final[str] = "robust_polygon_rolling"
MPC_STRATEGY: Final[str] = "belief_mpc"
STRATEGIES: Final[tuple[str, str]] = (ROBUST_STRATEGY, MPC_STRATEGY)


@dataclass(frozen=True, slots=True)
class Problem3Settings:
    """一次问题 3 运行使用的全部算法参数。"""

    seed: int = 1
    channels: tuple[int, ...] = CHANNELS
    arena_radius_m: float = ARENA_RADIUS_M
    guaranteed_radius_m: float = RECEPTION_RADIUS_MIN_M
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
    # 联合信念初始为“空间粗网格 × 固定半径离散值”；检测后按下列层级细化。
    coarse_grid_size: float = 150.0
    fine_grid_size: float = 30.0
    clearance_grid_size: float = 8.0
    radius_grid_size: float = 50.0
    probability_prune_threshold: float = 1e-7
    existence_threshold: float = 1e-6
    clear_probability: float = 0.99
    existence_prior: float = 0.65
    direction_bin_deg: float = 5.0
    entropy_clear_threshold_bits: float = 0.2
    mean_information_gain_floor_bits: float = 0.05
    p0: float = 0.8
    g0: float = 0.5
    candidate_action_limit: int = 30
    candidate_top_k: int = 24
    coverage_lookahead_nodes: int = 4
    detected_candidate_count: int = 3
    horizon: int = 3
    planning_horizon: int = 3
    beam_width: int = 20
    observation_scenario_limit: int = 5
    observation_merge_probability: float = 0.01
    replan_time_margin: float = 8.0
    efficiency_coverage_weight: float = 1.0
    efficiency_localization_weight: float = 1.0
    expected_clear_failure_probability: float = 0.01
    fallback_no_progress_steps: int = 5
    minimum_safe_remaining_time_s: float = 300.0
    ablation_variant: str = "E"
    random_seed: int | None = None
    maximum_noncoverage_actions: int = 6
    start_position: tuple[float, float] = INITIAL_POSITION
    start_channel: int = INITIAL_CHANNEL
    clearance_radius_m: float = CLEARANCE_RADIUS_M


ACTION_LOG_DIR = LOGS_DIR / "problem3"
SUMMARY_DIR = TABLES_DIR / "problem3"
FORMAL_RESULTS_TABLE = SUMMARY_DIR / "formal_runs.csv"
TRAJECTORY_FIGURE_DIR = FIGURES_DIR / "problem3"
