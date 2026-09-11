"""问题 3、4 共用的附件 HTTP 客户端。

客户端只使用附件规定的四个 POST 端点。每个新动作分配新 request_id；网络超时等
传输歧义不会自动重发，调用方只能显式重试保存的完全相同请求，防止动作执行两次。
"""

from __future__ import annotations

import json
import socket
from dataclasses import dataclass, field
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from config.simulator import ARENA_ID, BASE_URL, HTTP_TIMEOUT_S, ROBOT_ID


class SimulatorClientError(RuntimeError):
    """模拟器连接、HTTP状态或业务拒绝错误。"""


class AmbiguousActionError(SimulatorClientError):
    """请求可能已经执行但响应未收到，禁止自动创建新 ID 重试。"""


@dataclass(frozen=True, slots=True)
class ActionExchange:
    """一次请求及原始响应，供策略日志逐字保存。"""

    path: str
    request: dict[str, Any]
    status: int
    response: dict[str, Any]


@dataclass(slots=True)
class SimulatorClient:
    """串行调用 `/enter`、`/measure`、`/clear`、`/exit` 的客户端。"""

    base_url: str = BASE_URL
    robot_id: str = ROBOT_ID
    arena_id: str = ARENA_ID
    timeout_s: float = HTTP_TIMEOUT_S
    request_prefix: str = "p3"
    _counter: int = field(default=0, init=False, repr=False)
    _last_ambiguous: tuple[str, dict[str, Any]] | None = field(
        default=None, init=False, repr=False
    )

    def check_connection(self) -> None:
        """在 `/enter` 前验证主机端口可连接，不以伪数据降级。"""

        from urllib.parse import urlsplit

        parsed = urlsplit(self.base_url)
        if parsed.scheme != "http" or not parsed.hostname or parsed.port is None:
            raise SimulatorClientError("base_url must be http://host:port")
        try:
            with socket.create_connection(
                (parsed.hostname, parsed.port), timeout=self.timeout_s
            ):
                return
        except OSError as error:
            raise SimulatorClientError(
                f"cannot connect to simulator at {self.base_url}"
            ) from error

    def _new_id(self, action: str) -> str:
        """生成当前客户端实例内唯一且可读的 request_id。"""

        self._counter += 1
        return f"{self.request_prefix}-{action}-{self._counter}"

    def _base(self, request_id: str) -> dict[str, Any]:
        """构造四类请求共有的身份字段。"""

        return {
            "arena_id": self.arena_id,
            "robot_id": self.robot_id,
            "request_id": request_id,
        }

    def _post(self, path: str, payload: dict[str, Any]) -> ActionExchange:
        """发送一次动作；传输失败时保存原动作但绝不自动重发。"""

        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode(
            "utf-8"
        )
        request = Request(
            self.base_url + path,
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urlopen(request, timeout=self.timeout_s) as http_response:
                status = http_response.status
                raw = http_response.read()
        except HTTPError as error:
            status = error.code
            raw = error.read()
        except (URLError, TimeoutError, socket.timeout, ConnectionError) as error:
            self._last_ambiguous = (path, payload.copy())
            raise AmbiguousActionError(
                "transport failed; retry only with retry_last_ambiguous()"
            ) from error
        try:
            response = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise SimulatorClientError("simulator returned invalid UTF-8 JSON") from error
        exchange = ActionExchange(path, payload.copy(), status, response)
        if status != 200:
            raise SimulatorClientError(f"simulator returned HTTP {status}: {response}")
        if response.get("accepted") is not True:
            raise SimulatorClientError(f"simulator rejected {path}: {response}")
        self._last_ambiguous = None
        return exchange

    def retry_last_ambiguous(self) -> ActionExchange:
        """以完全相同的路径、请求体和 request_id 显式重试歧义动作。"""

        if self._last_ambiguous is None:
            raise SimulatorClientError("there is no ambiguous action to retry")
        path, payload = self._last_ambiguous
        return self._post(path, payload)

    def enter(self) -> ActionExchange:
        """进入测试并读取附件返回的现实/虚拟时限。"""

        return self._post("/enter", self._base(self._new_id("enter")))

    def measure(
        self, position: tuple[float, float], channel: int
    ) -> ActionExchange:
        """移动到指定位置并检测唯一指定频道。"""

        payload = self._base(self._new_id("measure"))
        payload.update(
            {
                "position": {"x": float(position[0]), "y": float(position[1])},
                "channel": int(channel),
            }
        )
        return self._post("/measure", payload)

    def clear(self, position: tuple[float, float], channel: int) -> ActionExchange:
        """移动到指定位置并尝试清除目标频道；不会改变测向频道。"""

        payload = self._base(self._new_id("clear"))
        payload.update(
            {
                "position": {"x": float(position[0]), "y": float(position[1])},
                "channel": int(channel),
            }
        )
        return self._post("/clear", payload)

    def exit(self) -> ActionExchange:
        """主动结束当前测试。"""

        return self._post("/exit", self._base(self._new_id("exit")))
