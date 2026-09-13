"""问题 3、4 本地模拟器的运行参数。

物理量直接复用题面常量；本文件只保存本地服务、幂等缓存和可复现案例的默认值。
"""

from typing import Final

DEFAULT_HOST: Final[str] = "127.0.0.1"
DEFAULT_PORT: Final[int] = 2026
DEFAULT_ROBOT_ID: Final[str] = "demo"
DEFAULT_SEED: Final[int] = 2026
MAX_REQUEST_BODY_BYTES: Final[int] = 65_536
MAX_JSON_DEPTH: Final[int] = 16
MAX_IDEMPOTENCY_RECORDS: Final[int] = 10_000

