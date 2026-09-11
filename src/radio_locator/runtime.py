"""运行时环境的小型跨平台适配层。"""

from __future__ import annotations

import os
import tempfile
from pathlib import Path


def prepare_matplotlib_config() -> None:
    """准备无界面绘图运行时，避免 Windows 缓存目录和 GUI 后端问题。"""

    if "MPLCONFIGDIR" not in os.environ:
        cache_dir = Path(tempfile.gettempdir()) / "radio_locator_matplotlib"
        cache_dir.mkdir(parents=True, exist_ok=True)
        os.environ["MPLCONFIGDIR"] = str(cache_dir)

    # 本项目只生成 PNG 文件；Agg 在 Windows 无桌面会话、CI 和普通终端下均稳定可用。
    import matplotlib

    matplotlib.use("Agg")
