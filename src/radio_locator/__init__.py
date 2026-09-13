"""各问题共用的无线电干扰源定位组件。

包内另含凸几何算法和本地模拟器；顶层只重导出稳定的真实端口客户端。
"""

# 在包顶层重新导出客户端类型，使调用方可写 ``from radio_locator import SimulatorClient``。
# 本地服务必须显式从命令行启动，导入包不会产生网络或文件系统副作用。
from radio_locator.client import SimulatorClient

# ``from radio_locator import *`` 时只公开稳定接口，不暴露内部模块名称。
__all__ = ["SimulatorClient"]
