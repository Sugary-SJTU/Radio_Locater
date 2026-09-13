"""问题 1 的交会定位区域计算参数与结果路径。

输入将是检测点坐标和对应示向度；后续算法输出定位多边形、区域直径及覆盖圆判定。
当前文件只整理误差边界、目标区域边界、数值容差和输出位置。
"""

from typing import Final

from config.constants import ARENA_RADIUS_M, BEARING_ERROR_DEG
from config.paths import FIGURES_DIR, TABLES_DIR

# 在题目 1 命名空间中使用更贴近该题语义的名称，数值仍来自公共题面常量。
# 每次测向形成“示向度 ± 1°”的扇形约束，多次约束相交得到定位区域。
ANGLE_ERROR_DEG: Final[float] = BEARING_ERROR_DEG
# 将定位区域限制在题目给定的半径 1800 m 圆域内。
DOMAIN_RADIUS_M: Final[float] = ARENA_RADIUS_M
# 数值计算容差，不是题面参数；几何算法实现后应通过测试确认是否需要调整。
GEOMETRY_TOLERANCE: Final[float] = 1e-9
# 用正多边形近似半径 1800 m 圆域。1440 条边对应 0.25° 的圆周分辨率，
# 兼顾绘图平滑度和半平面裁剪速度；这是数值参数，不是题面参数。
ARENA_POLYGON_VERTICES: Final[int] = 1_440

# 表格用于保存顶点、直径端点和覆盖判定；图片用于论文展示定位区域。
# 此处只声明路径，不会创建文件或父目录。
RESULT_TABLE = TABLES_DIR / "problem1_validation.csv"
INTERSECTION_FIGURE = FIGURES_DIR / "problem1_three_station_intersection.png"
CALIPERS_FIGURE = FIGURES_DIR / "problem1_rotating_calipers.png"
VALIDATION_FIGURE = FIGURES_DIR / "problem1_validation_cases.png"
COUNTEREXAMPLE_FIGURE = FIGURES_DIR / "problem1_diameter_circle_counterexample.png"
