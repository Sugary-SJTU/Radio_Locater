"""问题 3、4 共用的模拟器客户端参数容器。

本模块不发出 HTTP 请求。后续实现须在此处统一处理串行请求、``request_id`` 幂等、
连接重试、HTTP 状态时机和业务字段 ``accepted``，避免各题重复实现通信细节。
"""

from dataclasses import dataclass

from config.simulator import ARENA_ID, BASE_URL, HTTP_TIMEOUT_S, ROBOT_ID


# frozen=True 防止运行过程中意外改写连接身份；slots=True 减少实例属性并阻止拼错属性名。
@dataclass(frozen=True, slots=True)
class SimulatorClient:
    """保存模拟器连接所需的静态参数。

    属性:
        base_url: 模拟器本机服务地址。
        robot_id: 当前登录模拟器所使用的参赛队号。
        arena_id: 协议规定的目标区域标识，固定为 ``default``。
        timeout_s: 单次 HTTP 请求的现实时间超时。
    """

    # 默认值统一来自 config.simulator，同时允许测试时显式传入替代地址或虚拟队号。
    base_url: str = BASE_URL  # 例如 http://127.0.0.1:2026，不含末尾端点路径。
    robot_id: str = ROBOT_ID  # 必须与模拟器当前登录的参赛队号逐字节一致。
    arena_id: str = ARENA_ID  # 当前协议固定为 "default"，不应由策略自行生成。
    timeout_s: float = HTTP_TIMEOUT_S  # 现实 HTTP 等待时间，不是模拟器虚拟耗时。
