#!/usr/bin/env python3
"""Linux 项目根入口。

本文件只把 ``python main.py`` 转发到统一命令行入口，不包含模型、配置或业务逻辑。
安装环境后也可以直接运行 ``radio-locator``，两种方式行为一致。
"""

import sys
from pathlib import Path

# Linux 下允许直接执行 ``python main.py`` 或 ``./main.py``，无需预先设置
# PYTHONPATH；可编辑安装后的 console script 仍走同一个 cli.main。
SOURCE_DIR = Path(__file__).resolve().parent / "src"
if str(SOURCE_DIR) not in sys.path:
    sys.path.insert(0, str(SOURCE_DIR))

from radio_locator.cli import main


# 直接执行 ``python main.py`` 时调用入口；被测试或其他模块导入时不会自动运行。
if __name__ == "__main__":
    main()
