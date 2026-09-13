"""问题 4 的混合干扰源测试配置与结果路径。

与问题 3 共用模拟器和基础物理参数，额外暴露定向源的半覆盖角。后续算法状态和
策略超参数应与题面常量分开定义，避免人工检查时混淆。
"""

from dataclasses import dataclass
from typing import Final

from config.constants import (
    CHANNELS,
    CLEARANCE_RADIUS_M,
    DIRECTIONAL_HALF_ANGLE_DEG,
    INITIAL_CHANNEL,
    INITIAL_POSITION,
    SOURCE_COUNT_MAX,
    SOURCE_COUNT_MIN,
)
from config.paths import (
    LOGS_DIR,
    PROBLEM4_FIGURES_DIR,
    PROBLEM4_TRAJECTORY_FIGURES_DIR,
    TABLES_DIR,
)
from config.simulator import ARENA_ID, BASE_URL, HTTP_TIMEOUT_S, ROBOT_ID
from problems.problem3.config import Problem3Settings

# 问题 4 的干扰源总数和合法频道范围与问题 3 相同。
SOURCE_COUNT_RANGE: Final[range] = range(SOURCE_COUNT_MIN, SOURCE_COUNT_MAX + 1)
AVAILABLE_CHANNELS: Final[tuple[int, ...]] = CHANNELS
# 模拟器初始状态也与问题 3 相同。
START_POSITION: Final[tuple[float, float]] = INITIAL_POSITION
START_CHANNEL: Final[int] = INITIAL_CHANNEL
# 清除成功只取决于距离，与目标是全向源还是定向源无关。
TARGET_CLEARANCE_RADIUS_M: Final[float] = CLEARANCE_RADIUS_M
# 定向方向未知；检测点还必须落在定向方向左右各 90° 的覆盖区内才能收到信号。
SOURCE_COVERAGE_HALF_ANGLE_DEG: Final[float] = DIRECTIONAL_HALF_ANGLE_DEG

# 与问题 3 分开记录，避免两类测试的动作和正式结果互相覆盖。
ACTION_LOG_DIR = LOGS_DIR / "problem4"
SUMMARY_DIR = TABLES_DIR / "problem4"
ACTION_LOG = ACTION_LOG_DIR / "problem4_actions.jsonl"
FORMAL_TEST_TABLE = TABLES_DIR / "problem4_formal_tests.xlsx"
FIGURE_DIR = PROBLEM4_FIGURES_DIR
TRAJECTORY_FIGURE_DIR = PROBLEM4_TRAJECTORY_FIGURES_DIR

GUARANTEED_DIRECTIONAL_LATTICE: Final[str] = "guaranteed_directional_lattice"
OPTIMIZED_GUARANTEED_LATTICE: Final[str] = "optimized_guaranteed_lattice"
PARENT_FAST_STRATEGY: Final[str] = "problem4_fast"
LEGACY_OUTER_PROBE_FAST: Final[str] = "legacy_outer_probe_fast"


@dataclass(frozen=True, slots=True)
class Problem4Settings(Problem3Settings):
    """问题四算法参数；基础定位参数沿用问题三，定向发现参数单独列出。"""

    directional_mesh_layout: str = "concentric_ring"
    directional_grid_spacing_m: float = 1_000.0
    directional_grid_rotation_deg: float = 0.0
    directional_grid_offset_x_m: float = 0.0
    # 半行高平移可把有限圆域所需格点由31个降至27个，解析保证不变。
    directional_grid_offset_y_m: float = 433.0127018922193
    # 在12扇区几何约束内可取的更短内环；仍保持每个三角形最长边不超过1000m。
    directional_ring_inner_radius_m: float = 925.0
    opportunistic_bearing_limit: int = 5
    directional_clear_radius_m: float = 19.8
    route_clear_insertion_limit_m: float = 300.0
    route_probe_insertion_limit_m: float = 0.0

# 对外只暴露运行问题 4 必需的配置名称。
__all__ = [
    "ACTION_LOG",
    "ACTION_LOG_DIR",
    "ARENA_ID",
    "AVAILABLE_CHANNELS",
    "BASE_URL",
    "FIGURE_DIR",
    "FORMAL_TEST_TABLE",
    "GUARANTEED_DIRECTIONAL_LATTICE",
    "HTTP_TIMEOUT_S",
    "LEGACY_OUTER_PROBE_FAST",
    "OPTIMIZED_GUARANTEED_LATTICE",
    "PARENT_FAST_STRATEGY",
    "ROBOT_ID",
    "SOURCE_COUNT_RANGE",
    "SOURCE_COVERAGE_HALF_ANGLE_DEG",
    "START_CHANNEL",
    "START_POSITION",
    "SUMMARY_DIR",
    "TARGET_CLEARANCE_RADIUS_M",
    "TRAJECTORY_FIGURE_DIR",
    "Problem4Settings",
]
