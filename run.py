#!/usr/bin/env python3
"""Linux 兼容入口；实际参数解析统一位于 radio_locator.cli。"""

import sys
from pathlib import Path

SOURCE_DIR = Path(__file__).resolve().parent / "src"
if str(SOURCE_DIR) not in sys.path:
    sys.path.insert(0, str(SOURCE_DIR))

from radio_locator.cli import main


if __name__ == "__main__":
    main()
