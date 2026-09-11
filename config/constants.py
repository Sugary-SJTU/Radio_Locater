"""题面及附件给出的全局常量。

本文件只记录已知条件，不放置搜索步长、停止阈值等算法超参数。这样人工检查时
可以明确区分“题目规定值”和“后续建模假设”。距离单位统一为米，时间单位统一为秒。
"""

from typing import Final

# ``Final`` 向类型检查器和阅读者表明：以下名称是常量，不应在运行时重新赋值。
# Python 本身不会阻止常量被修改，因此调用代码仍需遵守这一约定。

# ---------------------------------------------------------------------------
# 目标区域与坐标
# ---------------------------------------------------------------------------
# 坐标系以圆域中心为原点，x 轴正向向东，y 轴正向向北。
ARENA_CENTER: Final[tuple[float, float]] = (0.0, 0.0)
# 所有干扰源都位于半径 1800 m 的圆形目标区域内。
ARENA_RADIUS_M: Final[float] = 1_800.0
# 机器狗允许走出目标区域，但提交坐标的每个分量绝对值不能超过此值。
MAX_COORDINATE_ABS_M: Final[float] = 2_000_000.0

# ---------------------------------------------------------------------------
# 干扰源
# ---------------------------------------------------------------------------
# 频道编号为闭区间 [1, 20] 内的整数；每个频道最多对应一个干扰源。
CHANNEL_MIN: Final[int] = 1
CHANNEL_MAX: Final[int] = 20
# 生成不可变频道序列，供问题 3、4 扫描全部频道时复用。
CHANNELS: Final[tuple[int, ...]] = tuple(range(CHANNEL_MIN, CHANNEL_MAX + 1))
# 每个案例中的干扰源总数未知，但题目给出了以下闭区间。
SOURCE_COUNT_MIN: Final[int] = 10
SOURCE_COUNT_MAX: Final[int] = 16
# 单个干扰源的实际接收半径不公开，只知道位于以下闭区间。
RECEPTION_RADIUS_MIN_M: Final[float] = 1_000.0
RECEPTION_RADIUS_MAX_M: Final[float] = 1_500.0
# 定向源覆盖定向方向左右各 90°，因此总覆盖角为 180°。
DIRECTIONAL_HALF_ANGLE_DEG: Final[float] = 90.0

# ---------------------------------------------------------------------------
# 测向与清除
# ---------------------------------------------------------------------------
# 示向度误差范围为 [-1°, 1°]；这里保存误差绝对值上界。
BEARING_ERROR_DEG: Final[float] = 1.0
# 位于信号覆盖范围且距离不超过 5 m 时，/measure 返回 near 而非示向度。
NEAR_DISTANCE_M: Final[float] = 5.0
# /clear 只判断距离，不受定向干扰源覆盖方向影响。
CLEARANCE_RADIUS_M: Final[float] = 20.0

# ---------------------------------------------------------------------------
# 动作耗时
# ---------------------------------------------------------------------------
# 移动耗时 = 相邻合法动作位置之间的欧氏距离 / 移动速度。
ROBOT_SPEED_MPS: Final[float] = 5.0
# 只有合法 /measure 改变当前频道时才会产生切换耗时。
CHANNEL_SWITCH_TIME_S: Final[float] = 1.0
# 无论检测结果为何，一次合法 /measure 的检测动作都耗时 5 s。
MEASUREMENT_TIME_S: Final[float] = 5.0
# /clear 先执行光学定位；发现目标后再执行激光清除。
OPTICAL_LOCALIZATION_TIME_S: Final[float] = 3.0
LASER_CLEARANCE_TIME_S: Final[float] = 2.0
# 两个合计值便于后续计算和核对模拟器的虚拟时间。
CLEARANCE_SUCCESS_TIME_S: Final[float] = 5.0
CLEARANCE_FAILURE_TIME_S: Final[float] = 3.0

# ---------------------------------------------------------------------------
# 测试限制与初始状态
# ---------------------------------------------------------------------------
# 测试窗口开放 25 min；/enter 成功后的程序运行时间最多为 20 min。
TEST_WINDOW_S: Final[int] = 1_500
MAX_REAL_DURATION_S: Final[int] = 1_200
# 虚拟世界活动时长最多为 100 h，即 360000 s。
MAX_VIRTUAL_DURATION_S: Final[int] = 360_000
# 每局成功调用 /enter 后都会重置为以下位置和测向机频道。
INITIAL_POSITION: Final[tuple[float, float]] = (0.0, 0.0)
INITIAL_CHANNEL: Final[int] = 1
