"""问题 4 的混合干扰源测试配置与结果路径。

与问题 3 共用模拟器和基础物理参数，额外暴露定向源的半覆盖角。后续算法状态和
策略超参数应与题面常量分开定义，避免人工检查时混淆。
"""

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
from config.paths import LOGS_DIR, TABLES_DIR
from config.simulator import ARENA_ID, BASE_URL, HTTP_TIMEOUT_S, ROBOT_ID

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
ACTION_LOG = LOGS_DIR / "problem4_actions.jsonl"
FORMAL_TEST_TABLE = TABLES_DIR / "problem4_formal_tests.xlsx"

# 对外只暴露运行问题 4 必需的配置名称。
__all__ = [
    "ACTION_LOG",
    "ARENA_ID",
    "AVAILABLE_CHANNELS",
    "BASE_URL",
    "FORMAL_TEST_TABLE",
    "HTTP_TIMEOUT_S",
    "ROBOT_ID",
    "SOURCE_COUNT_RANGE",
    "SOURCE_COVERAGE_HALF_ANGLE_DEG",
    "START_CHANNEL",
    "START_POSITION",
    "TARGET_CLEARANCE_RADIUS_M",
]
