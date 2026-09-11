"""各问题共用的无线电干扰源定位组件。

当前仅公开模拟器连接参数容器；几何工具、接口通信与搜索策略将在对应算法实现后加入。
"""

# 在包顶层重新导出客户端类型，使调用方可写 ``from radio_locator import SimulatorClient``。
# 通信方法尚未实现，所以目前不会因导入包而产生网络或文件系统副作用。
from radio_locator.client import SimulatorClient

# ``from radio_locator import *`` 时只公开稳定接口，不暴露内部模块名称。
__all__ = ["SimulatorClient"]
