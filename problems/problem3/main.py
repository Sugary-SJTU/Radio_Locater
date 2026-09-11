"""问题 3 的预留运行入口。

计划职责：连接模拟器，执行全向干扰源的搜索、定位和清除策略，记录每次请求及
响应，并汇总正式测试结果。当前阶段仅汇总配置和客户端参数容器，不会访问模拟器。
"""

from problems.problem3.config import (
    ACTION_LOG,
    AVAILABLE_CHANNELS,
    BASE_URL,
    FORMAL_TEST_TABLE,
    ROBOT_ID,
    START_CHANNEL,
    START_POSITION,
)
from radio_locator.client import SimulatorClient

# 当前只装配问题配置和公共客户端类型，不实例化客户端，也不会连接模拟器。
# 后续 main() 应先校验 ROBOT_ID，再按 enter → 串行动作 → exit 的生命周期运行。
__all__ = [
    "ACTION_LOG",
    "AVAILABLE_CHANNELS",
    "BASE_URL",
    "FORMAL_TEST_TABLE",
    "ROBOT_ID",
    "START_CHANNEL",
    "START_POSITION",
    "SimulatorClient",
]
