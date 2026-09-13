"""问题 3、4 使用的模拟器 HTTP 接口配置。

参赛队号不要提交到仓库；运行前通过 ``CUMCM_ROBOT_ID`` 环境变量设置。
本文件只定义连接参数、端点和响应状态码，不发送任何网络请求。
"""

import os
from typing import Final

# 可修改项从环境变量读取，避免将真实参赛队号或本机端口写进仓库。
# 未设置地址时，使用附件规定的本机默认服务地址。
BASE_URL: Final[str] = os.getenv("CUMCM_BASE_URL", "http://127.0.0.1:2026")
# 空字符串明确表示尚未配置队号；后续发送请求前必须对此进行校验。
ROBOT_ID: Final[str] = os.getenv("CUMCM_ROBOT_ID", "")
# 协议规定 arena_id 必须逐字节等于 ASCII 字符串 "default"。
ARENA_ID: Final[str] = "default"
# 环境变量读取结果为字符串，因此显式转换为 float。
HTTP_TIMEOUT_S: Final[float] = float(os.getenv("CUMCM_HTTP_TIMEOUT_S", "5"))
# 协议要求请求体为无 BOM 的 UTF-8 JSON 对象。
CONTENT_TYPE: Final[str] = "application/json"

# 以下端点必须精确匹配：不能添加尾随斜杠或查询参数，并且统一使用 POST。
ENTER_ENDPOINT: Final[str] = "/enter"
MEASURE_ENDPOINT: Final[str] = "/measure"
CLEAR_ENDPOINT: Final[str] = "/clear"
EXIT_ENDPOINT: Final[str] = "/exit"

# /measure 在 accepted=true 时可能返回的三种业务结果。
MEASURE_NO_SIGNAL: Final[str] = "no_signal"
MEASURE_NEAR: Final[str] = "near"
MEASURE_DIRECTION: Final[str] = "direction"
# /clear 在 accepted=true 时可能返回的两种业务结果。
CLEAR_SUCCESS: Final[str] = "success"
CLEAR_NO_TARGET: Final[str] = "no_target_in_range"
# 主动 /exit 成功后的固定原因；超时或中止通常会直接关闭接口。
EXIT_USER: Final[str] = "user_exit"




