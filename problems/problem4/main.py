"""问题 4 的预留运行入口。

计划职责：连接模拟器，处理全向和定向干扰源的搜索、定位及清除，并记录测试结果。
当前阶段仅汇总配置和客户端参数容器，不会访问模拟器。
"""

from problems.problem4.config import (
    ACTION_LOG,
    AVAILABLE_CHANNELS,
    BASE_URL,
    FORMAL_TEST_TABLE,
    ROBOT_ID,
    SOURCE_COVERAGE_HALF_ANGLE_DEG,
    START_CHANNEL,
    START_POSITION,
)
from radio_locator.client import SimulatorClient

# 当前只装配问题配置与客户端类型，不会创建网络连接。
# 后续策略不能将 no_signal 直接解释为“该频道没有干扰源”，因为定向源可能未覆盖检测点。
__all__ = [
    "ACTION_LOG",
    "AVAILABLE_CHANNELS",
    "BASE_URL",
    "FORMAL_TEST_TABLE",
    "ROBOT_ID",
    "SOURCE_COVERAGE_HALF_ANGLE_DEG",
    "START_CHANNEL",
    "START_POSITION",
    "SimulatorClient",
]
