"""Linux 无桌面会话下的 Matplotlib 运行时适配。"""

from __future__ import annotations

import os
import tempfile
from pathlib import Path


def prepare_matplotlib_config() -> None:
    """使用用户隔离的临时缓存和Agg后端，兼容终端、SSH及CI运行。"""

    if "MPLCONFIGDIR" not in os.environ:
        # 受限Linux环境中的 ~/.config 可能只读。加入uid可避免多用户共同使用一个
        # Matplotlib字体缓存；不要修改HOME或依赖启动目录。
        user_id = os.getuid() if hasattr(os, "getuid") else 0
        cache_dir = (
            Path(tempfile.gettempdir()) / f"radio_locator_matplotlib_{user_id}"
        )
        cache_dir.mkdir(parents=True, exist_ok=True)
        os.environ["MPLCONFIGDIR"] = str(cache_dir)

    # 项目只写PNG，不启动X11/Wayland窗口；在导入pyplot之前固定无界面后端。
    os.environ.setdefault("MPLBACKEND", "Agg")
    import matplotlib

    matplotlib.use("Agg", force=True)
