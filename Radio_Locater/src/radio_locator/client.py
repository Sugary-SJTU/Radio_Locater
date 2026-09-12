"""问题 3、4 共用的模拟器 HTTP 客户端。

除保存连接参数外，本模块实现四类 POST 动作：``/enter``、``/measure``、
``/clear`` 和 ``/exit``。协议要求动作必须串行，且网络重试时必须复用同一个
``request_id``；本模块在单次动作内用同一个请求体自动重试，从而满足幂等要求。
"""

from __future__ import annotations

import json
import time
import uuid
from dataclasses import dataclass
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from config.simulator import ARENA_ID, BASE_URL, HTTP_TIMEOUT_S, ROBOT_ID


class SimulatorError(RuntimeError):
    """模拟器请求失败或返回无法继续处理的响应。"""


class SimulatorRejected(SimulatorError):
    """``accepted=false`` 或 HTTP 错误导致本次动作未被接受。"""


class SimulatorNetworkError(SimulatorError):
    """网络层失败，可按协议复用相同 ``request_id`` 重试。"""


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
    retries: int = 3  # 网络层失败时的最大尝试次数，新动作和重试均使用同一请求体。

    # ------------------------------------------------------------------
    # 内部工具
    # ------------------------------------------------------------------
    def _endpoint_url(self, path: str) -> str:
        """拼接精确端点路径；base_url 不应带尾随斜杠。"""

        return f"{self.base_url.rstrip('/')}{path}"

    @staticmethod
    def _request_id() -> str:
        """生成单次动作的幂等键；长度和字符集均满足协议要求。"""

        return f"robot-{uuid.uuid4().hex}"

    def _base_payload(self, request_id: str) -> dict[str, str]:
        """构造每条指令公共的 arena_id、robot_id 和 request_id 字段。"""

        return {
            "arena_id": self.arena_id,
            "robot_id": self.robot_id,
            "request_id": request_id,
        }

    def _post(self, path: str, payload: dict[str, Any]) -> dict[str, Any]:
        """发送一次 POST；不在此处重试，网络异常会抛出专用异常。"""

        request = Request(
            self._endpoint_url(path),
            data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urlopen(request, timeout=self.timeout_s) as response:
                status = int(response.status)
                raw = response.read()
        except HTTPError as exc:
            status = int(exc.code)
            raw = exc.read()
        except (URLError, TimeoutError, ConnectionError, OSError) as exc:
            raise SimulatorNetworkError(str(exc)) from exc

        try:
            body = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise SimulatorError(f"模拟器返回了非 JSON 响应：{raw!r}") from exc

        if status != 200:
            raise SimulatorRejected(f"HTTP {status}：{body}")
        if not body.get("accepted", False):
            raise SimulatorRejected(f"accepted=false：{body}")
        return body

    def _call(self, path: str, payload: dict[str, Any]) -> dict[str, Any]:
        """串行执行一个动作；网络失败时复用同一 request_id 重试。"""

        last_error: Exception | None = None
        for attempt in range(1, self.retries + 1):
            try:
                return self._post(path, payload)
            except SimulatorNetworkError as exc:
                last_error = exc
                time.sleep(0.05 * (2**attempt))
        assert last_error is not None
        raise last_error

    # ------------------------------------------------------------------
    # 四个协议动作
    # ------------------------------------------------------------------
    def enter(self) -> dict[str, Any]:
        """进入目标区域并开始计时。"""

        request_id = self._request_id()
        return self._call("/enter", self._base_payload(request_id))

    def measure(
        self,
        position: tuple[float, float],
        channel: int,
    ) -> dict[str, Any]:
        """移动到指定位置并检测指定频道。"""

        request_id = self._request_id()
        payload = self._base_payload(request_id)
        payload["position"] = {"x": float(position[0]), "y": float(position[1])}
        payload["channel"] = int(channel)
        return self._call("/measure", payload)

    def clear(
        self,
        position: tuple[float, float],
        channel: int,
    ) -> dict[str, Any]:
        """移动到指定位置并尝试清除指定频道的干扰源。"""

        request_id = self._request_id()
        payload = self._base_payload(request_id)
        payload["position"] = {"x": float(position[0]), "y": float(position[1])}
        payload["channel"] = int(channel)
        return self._call("/clear", payload)

    def exit(self) -> dict[str, Any]:
        """主动结束测试。"""

        request_id = self._request_id()
        return self._call("/exit", self._base_payload(request_id))
