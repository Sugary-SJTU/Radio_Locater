"""问题 3 的全向干扰源测试配置与结果路径。

配置来自题面、模拟器附件和环境变量。后续搜索策略自行维护已检测频道、定位区域
和已清除目标；这些动态状态不得写入本配置文件。
"""

from typing import Final

from config.constants import (
    CHANNELS,
    CLEARANCE_RADIUS_M,
    INITIAL_CHANNEL,
    INITIAL_POSITION,
    SOURCE_COUNT_MAX,
    SOURCE_COUNT_MIN,
)
from config.paths import LOGS_DIR, TABLES_DIR
from config.simulator import ARENA_ID, BASE_URL, HTTP_TIMEOUT_S, ROBOT_ID

# range 的右端点不包含在内，因此上界需要加 1；这里表示可能的数量 10 至 16。
SOURCE_COUNT_RANGE: Final[range] = range(SOURCE_COUNT_MIN, SOURCE_COUNT_MAX + 1)
# 使用 tuple 防止策略运行时意外增删合法频道。
AVAILABLE_CHANNELS: Final[tuple[int, ...]] = CHANNELS
# 每次 /enter 成功后，位置固定为原点，测向机固定为频道 1。
START_POSITION: Final[tuple[float, float]] = INITIAL_POSITION
START_CHANNEL: Final[int] = INITIAL_CHANNEL
# 估计位置与真实干扰源相距不超过 20 m 时，/clear 才能成功。
TARGET_CLEARANCE_RADIUS_M: Final[float] = CLEARANCE_RADIUS_M

# JSONL 适合逐动作追加一行 JSON，即使程序中途退出也能保留此前记录。
ACTION_LOG = LOGS_DIR / "problem3_actions.jsonl"
# 正式测试表应汇总案例编码、清除数量、平均定位清除时间和程序运行时间。
FORMAL_TEST_TABLE = TABLES_DIR / "problem3_formal_tests.xlsx"

# 显式列出可供问题 3 入口或后续策略模块使用的配置，避免通配导入泄露内部名称。
__all__ = [
    "ACTION_LOG",
    "ARENA_ID",
    "AVAILABLE_CHANNELS",
    "BASE_URL",
    "FORMAL_TEST_TABLE",
    "HTTP_TIMEOUT_S",
    "ROBOT_ID",
    "SOURCE_COUNT_RANGE",
    "START_CHANNEL",
    "START_POSITION",
    "TARGET_CLEARANCE_RADIUS_M",
]
