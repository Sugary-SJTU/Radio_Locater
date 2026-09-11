"""问题 2 的第二检测点选择参数与结果路径。

输入将是首个检测点及其示向度；后续算法输出第二检测点的选择规则和候选区域。
当前文件只保存题面角度、区域范围和输出位置。
"""

from typing import Final

from config.constants import (
    ARENA_RADIUS_M,
    BEARING_ERROR_DEG,
    NEAR_DISTANCE_M,
    RECEPTION_RADIUS_MAX_M,
    RECEPTION_RADIUS_MIN_M,
)
from config.paths import FIGURES_DIR, TABLES_DIR

# 首次示向度同样带有 ±1° 误差，候选区域必须考虑该误差带。
ANGLE_ERROR_DEG: Final[float] = BEARING_ERROR_DEG
# 第二检测点可以在目标圆域内选择；若后续允许走出圆域，应在模型中单独说明。
DOMAIN_RADIUS_M: Final[float] = ARENA_RADIUS_M
# 角度统一使用“正东为 0°、逆时针为正、范围 [0°, 360°)”的题目约定。
FULL_TURN_DEG: Final[float] = 360.0
# 两条观测方向接近正交时通常具有更好的交会几何条件。
# 这里只记录几何基准角，不等同于已经确定第二检测点策略。
RIGHT_ANGLE_DEG: Final[float] = 90.0

# 论文式 (26)、(29) 的粗筛阈值。
MIN_DETECTION_PROBABILITY: Final[float] = 0.8
MIN_GEOMETRY_SCORE: Final[float] = 0.5
MIN_SECOND_POINT_DISTANCE_M: Final[float] = NEAR_DISTANCE_M
GUARANTEED_RECEPTION_RADIUS_M: Final[float] = RECEPTION_RADIUS_MIN_M
MAX_RECEPTION_RADIUS_M: Final[float] = RECEPTION_RADIUS_MAX_M

# 以下均为离散求解参数，并非题目给定值。粗网格负责覆盖全局，细网格只搜索
# 粗网格目标函数最优点附近；误差样本覆盖 [-1°, 1°]。
POSTERIOR_GRID_STEP_M: Final[float] = 45.0
# 固定 M2 后的直径分布使用更密的独立网格；它只影响验证表和分布图，不改变选点。
DIAMETER_SAMPLE_GRID_STEP_M: Final[float] = 20.0
COARSE_CANDIDATE_STEP_M: Final[float] = 150.0
REFINE_RADIUS_M: Final[float] = 240.0
REFINE_STEP_M: Final[float] = 40.0
BEARING_ERROR_SAMPLES_DEG: Final[tuple[float, ...]] = (-1.0, 0.0, 1.0)
NEAR_OPTIMAL_RELATIVE_GAP: Final[float] = 0.05
DIAMETER_FLOOR_M: Final[float] = 1.0
ARENA_POLYGON_VERTICES: Final[int] = 720

# 分别保存候选点数据和候选区域示意图；本文件不执行写入。
RESULT_TABLE = TABLES_DIR / "problem2_candidate_scores.csv"
REGION_SUMMARY = TABLES_DIR / "problem2_excellent_regions.json"
DIAMETER_SAMPLE_TABLE = TABLES_DIR / "problem2_selected_m2_diameters.csv"
SCREENING_FIGURE = FIGURES_DIR / "problem2_screening_principles.png"
RESULT_FIGURE = FIGURES_DIR / "problem2_candidate_region.png"
DIAMETER_HISTOGRAM = FIGURES_DIR / "problem2_selected_m2_diameter_histogram.png"
