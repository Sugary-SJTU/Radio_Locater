"""复现附件 1、2 的 enter→measure→measure→clear→measure→exit 示例。

先启动本地模拟器，再运行本文件。该脚本只验证接口和计时语义，不是搜索策略。
"""

from __future__ import annotations

import argparse
import json
from urllib.request import Request, urlopen


def post(base_url: str, path: str, payload: dict[str, object]) -> dict[str, object]:
    """以附件规定的无 BOM UTF-8 application/json 发送一次 POST。"""

    request = Request(
        base_url.rstrip("/") + path,
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urlopen(request, timeout=5) as response:
        result = json.loads(response.read().decode("utf-8"))
    print(path, result)
    return result


def main() -> None:
    """按附件表 2/第 10 节的坐标、频道及 request_id 顺序调用接口。"""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default="http://127.0.0.1:2026")
    parser.add_argument("--robot-id", default="demo")
    arguments = parser.parse_args()

    def base(request_id: str) -> dict[str, object]:
        return {
            "arena_id": "default",
            "robot_id": arguments.robot_id,
            "request_id": request_id,
        }

    def action(
        request_id: str,
        x_m: float,
        y_m: float,
        channel: int,
    ) -> dict[str, object]:
        payload = base(request_id)
        payload.update(
            {"position": {"x": x_m, "y": y_m}, "channel": channel}
        )
        return payload

    post(arguments.base_url, "/enter", base("enter-1"))
    post(arguments.base_url, "/measure", action("measure-1", 300, 400, 1))
    post(arguments.base_url, "/measure", action("measure-2", 300, 400, 2))
    post(arguments.base_url, "/clear", action("clear-1", 300, 0, 3))
    post(arguments.base_url, "/measure", action("measure-3", 300, 0, 2))
    post(arguments.base_url, "/exit", base("exit-1"))


if __name__ == "__main__":
    main()

