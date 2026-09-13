"""兼容题目示例的根运行入口；实际参数解析统一位于 radio_locator.cli。"""

import sys
import time
from pathlib import Path

# 优先使用本 checkout 的源码，避免虚拟环境中旧的 editable 安装覆盖当前版本。
sys.path.insert(0, str(Path(__file__).resolve().parent / "src"))

from radio_locator.cli import main


def run() -> None:
    """执行统一命令行入口，并输出本次进程的实际墙钟运行时间。"""

    started_at = time.perf_counter()
    try:
        main()
    finally:
        elapsed_s = time.perf_counter() - started_at
        print(f"\n程序实际运行时间：{elapsed_s:.2f} s", flush=True)


if __name__ == "__main__":
    run()
