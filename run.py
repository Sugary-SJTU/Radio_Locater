"""兼容题目示例的根运行入口；实际参数解析统一位于 radio_locator.cli。"""

import sys
from pathlib import Path

# 优先使用本 checkout 的源码，避免虚拟环境中旧的 editable 安装覆盖当前版本。
sys.path.insert(0, str(Path(__file__).resolve().parent / "src"))

from radio_locator.cli import main


if __name__ == "__main__":
    main()
