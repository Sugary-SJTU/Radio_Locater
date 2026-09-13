"""与附件 1、2 HTTP+JSON 协议兼容的本地问题 3、4 模拟器。

本模块包含可复现场景生成、物理与虚拟时间状态机、严格请求校验和本机 HTTP 服务。
场景真值只用于本地演练；通过四个正式接口不能读取干扰源数量、位置或接收半径。
"""

from __future__ import annotations

import hashlib
import json
import math
import threading
import time
import unicodedata
from dataclasses import asdict, dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable, Final

import numpy as np

from config.constants import (
    ARENA_RADIUS_M,
    BEARING_ERROR_DEG,
    CHANNEL_MAX,
    CHANNEL_MIN,
    CHANNEL_SWITCH_TIME_S,
    CLEARANCE_FAILURE_TIME_S,
    CLEARANCE_RADIUS_M,
    CLEARANCE_SUCCESS_TIME_S,
    DIRECTIONAL_HALF_ANGLE_DEG,
    INITIAL_CHANNEL,
    INITIAL_POSITION,
    MAX_COORDINATE_ABS_M,
    MAX_REAL_DURATION_S,
    MAX_VIRTUAL_DURATION_S,
    MEASUREMENT_TIME_S,
    NEAR_DISTANCE_M,
    RECEPTION_RADIUS_MAX_M,
    RECEPTION_RADIUS_MIN_M,
    ROBOT_SPEED_MPS,
    SOURCE_COUNT_MAX,
    SOURCE_COUNT_MIN,
)
from config.local_simulator import (
    MAX_IDEMPOTENCY_RECORDS,
    MAX_JSON_DEPTH,
    MAX_REQUEST_BODY_BYTES,
)

ENDPOINTS: Final[frozenset[str]] = frozenset(
    {"/enter", "/measure", "/clear", "/exit"}
)
BASE_FIELDS: Final[frozenset[str]] = frozenset(
    {"arena_id", "robot_id", "request_id"}
)
ACTION_FIELDS: Final[frozenset[str]] = BASE_FIELDS | {"position", "channel"}


class ProtocolError(ValueError):
    """表示应返回 HTTP 400 的 JSON 结构或字段类型错误。"""


class DuplicateKeyError(ProtocolError):
    """表示 JSON 对象中出现附件禁止的重复键。"""


@dataclass(slots=True)
class InterferenceSource:
    """一个本地案例中的干扰源真值。"""

    channel: int
    x_m: float
    y_m: float
    reception_radius_m: float
    source_type: str
    direction_deg: float | None
    cleared: bool = False

    @property
    def position(self) -> tuple[float, float]:
        """返回便于距离计算的二维坐标。"""

        return self.x_m, self.y_m


@dataclass(slots=True)
class SimulatorScenario:
    """由题号和随机种子唯一确定的干扰源案例。"""

    problem: int
    seed: int
    sources: list[InterferenceSource]

    @classmethod
    def generate(cls, problem: int, seed: int) -> "SimulatorScenario":
        """在目标圆域内均匀生成 10--16 个频道互异的干扰源。"""

        if problem not in (3, 4):
            raise ValueError("problem must be 3 or 4")
        rng = np.random.default_rng(seed)
        count = int(rng.integers(SOURCE_COUNT_MIN, SOURCE_COUNT_MAX + 1))
        channels = sorted(
            int(value)
            for value in rng.choice(
                np.arange(CHANNEL_MIN, CHANNEL_MAX + 1),
                size=count,
                replace=False,
            )
        )
        if problem == 4:
            directional_count = int(rng.integers(1, count))
            flags = np.array(
                [True] * directional_count + [False] * (count - directional_count)
            )
            rng.shuffle(flags)
        else:
            flags = np.zeros(count, dtype=bool)

        sources: list[InterferenceSource] = []
        for channel, directional in zip(channels, flags, strict=True):
            # sqrt(U) 使面积密度在圆域内均匀，不能直接令半径服从均匀分布。
            radius = ARENA_RADIUS_M * math.sqrt(float(rng.random()))
            angle = 2.0 * math.pi * float(rng.random())
            sources.append(
                InterferenceSource(
                    channel=channel,
                    x_m=radius * math.cos(angle),
                    y_m=radius * math.sin(angle),
                    reception_radius_m=float(
                        rng.uniform(
                            RECEPTION_RADIUS_MIN_M,
                            RECEPTION_RADIUS_MAX_M,
                        )
                    ),
                    source_type="directional" if directional else "omnidirectional",
                    direction_deg=(float(rng.uniform(0.0, 360.0)) if directional else None),
                )
            )
        return cls(problem=problem, seed=seed, sources=sources)

    def write_truth(self, output: Path) -> None:
        """写出本地案例真值，供固定种子的人工复核。"""

        output.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "file_function": "本地模拟器案例真值；定位策略不得读取此文件。",
            "problem": self.problem,
            "seed": self.seed,
            "source_count": len(self.sources),
            "sources": [asdict(source) for source in self.sources],
        }
        output.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )


@dataclass(frozen=True, slots=True)
class CachedResponse:
    """一次已接受动作的幂等指纹及其首次完整响应。"""

    fingerprint: str
    status: int
    response: dict[str, Any]


def _bearing_deg(origin: tuple[float, float], target: tuple[float, float]) -> float:
    """按题面角度制计算 origin 指向 target 的方位角。"""

    return math.degrees(
        math.atan2(target[1] - origin[1], target[0] - origin[0])
    ) % 360.0


def _angular_difference_deg(first: float, second: float) -> float:
    """返回两个方向的最小绝对夹角，范围为 [0, 180]。"""

    return abs((first - second + 180.0) % 360.0 - 180.0)


def _finite_number(value: Any) -> bool:
    """判断值是否为有限 JSON number，并排除属于 int 子类的 bool。"""

    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _identifier_valid(value: Any, maximum_bytes: int) -> bool:
    """检查标识符类型、UTF-8长度及控制/不可见格式字符。"""

    if not isinstance(value, str):
        return False
    length = len(value.encode("utf-8"))
    if not 1 <= length <= maximum_bytes:
        return False
    return not any(unicodedata.category(character) in {"Cc", "Cf"} for character in value)


def _json_depth(value: Any, current: int = 1) -> int:
    """计算 JSON 容器嵌套深度。"""

    if isinstance(value, dict):
        return max([current, *(_json_depth(item, current + 1) for item in value.values())])
    if isinstance(value, list):
        return max([current, *(_json_depth(item, current + 1) for item in value)])
    return current


def _unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    """供 json.loads 使用，在构造字典前拒绝重复键。"""

    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise DuplicateKeyError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def decode_request_json(raw: bytes) -> dict[str, Any]:
    """按附件限制解析无 BOM UTF-8 JSON 对象。"""

    if raw.startswith(b"\xef\xbb\xbf"):
        raise ProtocolError("UTF-8 BOM is not allowed")
    try:
        text = raw.decode("utf-8", errors="strict")
        value = json.loads(
            text,
            object_pairs_hook=_unique_object,
            parse_constant=lambda token: (_ for _ in ()).throw(
                ProtocolError(f"non-finite JSON number: {token}")
            ),
        )
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ProtocolError("request body is not valid UTF-8 JSON") from error
    if not isinstance(value, dict):
        raise ProtocolError("request body must be a JSON object")
    if _json_depth(value) > MAX_JSON_DEPTH:
        raise ProtocolError("JSON nesting exceeds 16 levels")
    return value


class SimulatorEngine:
    """执行附件物理规则、状态转换、计时和 request_id 幂等语义。"""

    def __init__(
        self,
        scenario: SimulatorScenario,
        robot_id: str,
        timestamp_ms: Callable[[], int] | None = None,
        monotonic_s: Callable[[], float] | None = None,
    ) -> None:
        if not _identifier_valid(robot_id, 64):
            raise ValueError("robot_id must be 1--64 UTF-8 bytes without control characters")
        self.scenario = scenario
        self.robot_id = robot_id
        self.entered = False
        self.exited = False
        self.position = INITIAL_POSITION
        self.channel = INITIAL_CHANNEL
        self.virtual_time_s = 0.0
        self._timestamp_ms = timestamp_ms or (lambda: time.time_ns() // 1_000_000)
        self._monotonic_s = monotonic_s or time.monotonic
        self._entered_at_s: float | None = None
        self._idempotency: dict[str, CachedResponse] = {}
        self.action_lock = threading.Lock()

    def _base_response(self, accepted: bool, current_time: bool) -> dict[str, Any]:
        """构造所有 HTTP/业务响应共有的三个字段。"""

        return {
            "accepted": accepted,
            "real_timestamp_ms": self._timestamp_ms(),
            "virtual_time_s": self._virtual_time_value() if current_time else 0,
        }

    def _virtual_time_value(self) -> int | float:
        """按附件要求最多保留六位小数并删除无意义零。"""

        value = round(self.virtual_time_s, 6)
        return int(value) if value.is_integer() else value

    def error_response(self, status: int) -> tuple[int, dict[str, Any]]:
        """构造未执行请求的最小错误响应。"""

        return status, self._base_response(False, current_time=False)

    def _validate_schema(self, path: str, payload: dict[str, Any]) -> bool:
        """验证字段类型；返回是否含未知字段，未知字段属于业务拒绝。"""

        expected = BASE_FIELDS if path in {"/enter", "/exit"} else ACTION_FIELDS
        missing = expected - payload.keys()
        if missing:
            raise ProtocolError(f"missing required fields: {sorted(missing)}")
        if not isinstance(payload["arena_id"], str):
            raise ProtocolError("arena_id must be a string")
        if not _identifier_valid(payload["robot_id"], 64):
            raise ProtocolError("invalid robot_id")
        if not _identifier_valid(payload["request_id"], 128):
            raise ProtocolError("invalid request_id")
        unknown = bool(payload.keys() - expected)
        if path in {"/measure", "/clear"}:
            position = payload["position"]
            if not isinstance(position, dict):
                raise ProtocolError("position must be an object")
            if not {"x", "y"}.issubset(position):
                raise ProtocolError("position requires x and y")
            unknown = unknown or bool(position.keys() - {"x", "y"})
            for coordinate in (position["x"], position["y"]):
                if not _finite_number(coordinate) or abs(float(coordinate)) > MAX_COORDINATE_ABS_M:
                    raise ProtocolError("coordinate must be finite and within ±2000000")
            channel = payload["channel"]
            if not _finite_number(channel) or not float(channel).is_integer():
                raise ProtocolError("channel must be an integer-valued number")
            if not CHANNEL_MIN <= int(channel) <= CHANNEL_MAX:
                raise ProtocolError("channel must lie in 1..20")
        return unknown

    @staticmethod
    def _fingerprint(path: str, payload: dict[str, Any]) -> str:
        """生成与 JSON 空白及键顺序无关的动作指纹。"""

        canonical = json.dumps(payload, sort_keys=True, ensure_ascii=False, separators=(",", ":"))
        return f"{path}\n{canonical}"

    def process(self, path: str, payload: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        """处理一个已经成功解码的请求，返回 HTTP 状态和 JSON 响应。"""

        try:
            unknown = self._validate_schema(path, payload)
        except ProtocolError:
            return self.error_response(400)
        if unknown:
            return self.error_response(200)

        request_id = payload["request_id"]
        fingerprint = self._fingerprint(path, payload)
        cached = self._idempotency.get(request_id)
        if cached is not None:
            if cached.fingerprint != fingerprint:
                return self.error_response(409)
            return cached.status, dict(cached.response)
        if payload["arena_id"] != "default" or payload["robot_id"] != self.robot_id:
            return self.error_response(200)
        if len(self._idempotency) >= MAX_IDEMPOTENCY_RECORDS:
            return self.error_response(429)

        if path == "/enter":
            if self.entered or self.exited:
                return self.error_response(200)
            self.entered = True
            self._entered_at_s = self._monotonic_s()
            self.position = INITIAL_POSITION
            self.channel = INITIAL_CHANNEL
            response = self._base_response(True, current_time=True)
            response.update(
                {
                    "max_virtual_duration_s": MAX_VIRTUAL_DURATION_S,
                    "max_real_duration_s": MAX_REAL_DURATION_S,
                    "remaining_real_duration_s": MAX_REAL_DURATION_S,
                }
            )
        elif not self.entered or self.exited:
            return self.error_response(200)
        elif self._timed_out():
            self.exited = True
            return self.error_response(200)
        elif path == "/measure":
            response = self._measure(payload)
        elif path == "/clear":
            response = self._clear(payload)
        else:
            response = self._base_response(True, current_time=True)
            response["exit_reason"] = "user_exit"
            self.exited = True

        cached_response = CachedResponse(fingerprint, 200, dict(response))
        self._idempotency[request_id] = cached_response
        return 200, response

    def _timed_out(self) -> bool:
        """判断程序现实时间或虚拟时间是否已在新动作到达前耗尽。"""

        real_expired = (
            self._entered_at_s is not None
            and self._monotonic_s() - self._entered_at_s >= MAX_REAL_DURATION_S
        )
        virtual_expired = self.virtual_time_s >= MAX_VIRTUAL_DURATION_S
        return real_expired or virtual_expired

    def _movement_time(self, position: tuple[float, float]) -> float:
        """计算并落实从上一合法动作位置到新位置的移动耗时。"""

        distance = math.dist(self.position, position)
        self.position = position
        return distance / ROBOT_SPEED_MPS

    def _source_on_channel(self, channel: int) -> InterferenceSource | None:
        """返回指定频道尚未清除的干扰源。"""

        return next(
            (
                source
                for source in self.scenario.sources
                if source.channel == channel and not source.cleared
            ),
            None,
        )

    def _covered(self, source: InterferenceSource, position: tuple[float, float]) -> bool:
        """判断检测点是否位于全向或定向源的有效覆盖范围。"""

        distance = math.dist(source.position, position)
        if distance > source.reception_radius_m + 1e-9:
            return False
        # 定向扇形的顶点属于覆盖集；重合时方位角本身没有定义，不能用 atan2(0,0)
        # 得到的任意 0° 再做方向排除。
        if distance <= 1e-9:
            return True
        if source.source_type == "omnidirectional":
            return True
        direction_to_detector = _bearing_deg(source.position, position)
        return _angular_difference_deg(direction_to_detector, source.direction_deg or 0.0) <= DIRECTIONAL_HALF_ANGLE_DEG + 1e-9

    def _fixed_bearing_error(self, channel: int, position: tuple[float, float]) -> float:
        """按种子、频道和地点确定固定误差，保证同地点重复检测不改变。"""

        material = (
            f"{self.scenario.seed}|{channel}|{position[0]:.12g}|{position[1]:.12g}"
        ).encode("ascii")
        integer = int.from_bytes(hashlib.blake2b(material, digest_size=8).digest(), "big")
        unit = integer / (2**64 - 1)
        return (2.0 * unit - 1.0) * BEARING_ERROR_DEG

    def _measure(self, payload: dict[str, Any]) -> dict[str, Any]:
        """执行移动、可能的频道切换和五秒检测。"""

        position = (float(payload["position"]["x"]), float(payload["position"]["y"]))
        channel = int(payload["channel"])
        switch_time = CHANNEL_SWITCH_TIME_S if channel != self.channel else 0.0
        self.virtual_time_s += self._movement_time(position) + switch_time + MEASUREMENT_TIME_S
        self.channel = channel
        source = self._source_on_channel(channel)
        response = self._base_response(True, current_time=True)
        if source is None or not self._covered(source, position):
            response["measure_result"] = "no_signal"
            return response
        distance = math.dist(source.position, position)
        if distance <= NEAR_DISTANCE_M + 1e-9:
            response["measure_result"] = "near"
            return response
        measured = (
            _bearing_deg(position, source.position)
            + self._fixed_bearing_error(channel, position)
        ) % 360.0
        rounded = round(measured, 2) % 360.0
        response.update({"measure_result": "direction", "svd_deg": rounded})
        return response

    def _clear(self, payload: dict[str, Any]) -> dict[str, Any]:
        """执行移动和光学清除；不切换或改变测向机频道。"""

        position = (float(payload["position"]["x"]), float(payload["position"]["y"]))
        channel = int(payload["channel"])
        source = self._source_on_channel(channel)
        success = source is not None and math.dist(source.position, position) <= CLEARANCE_RADIUS_M + 1e-9
        action_time = CLEARANCE_SUCCESS_TIME_S if success else CLEARANCE_FAILURE_TIME_S
        self.virtual_time_s += self._movement_time(position) + action_time
        if success and source is not None:
            source.cleared = True
        response = self._base_response(True, current_time=True)
        response["clear_result"] = "success" if success else "no_target_in_range"
        return response


class SimulatorHTTPServer(ThreadingHTTPServer):
    """携带 SimulatorEngine 的本机多线程 HTTP 服务。"""

    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address: tuple[str, int], engine: SimulatorEngine) -> None:
        self.engine = engine
        super().__init__(address, SimulatorRequestHandler)


class SimulatorRequestHandler(BaseHTTPRequestHandler):
    """将附件规定的 HTTP 细节适配到纯状态机。"""

    server: SimulatorHTTPServer
    protocol_version = "HTTP/1.1"

    def log_message(self, format_string: str, *arguments: object) -> None:
        """关闭逐请求刷屏；完整动作已由机器狗客户端写入 JSONL。"""

        return None

    def _send_json(self, status: int, payload: dict[str, Any]) -> None:
        """发送无 BOM UTF-8 JSON 响应。"""

        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _protocol_error(self, status: int) -> None:
        """发送至少包含三个公共字段的协议错误响应。"""

        actual_status, response = self.server.engine.error_response(status)
        self._send_json(actual_status, response)

    @staticmethod
    def _content_type_valid(value: str | None) -> bool:
        """只接受 application/json 及唯一可选参数 charset=utf-8。"""

        if value is None:
            return False
        parts = [part.strip() for part in value.split(";")]
        if parts[0].lower() != "application/json":
            return False
        if len(parts) == 1:
            return True
        return len(parts) == 2 and parts[1].replace(" ", "").lower() == "charset=utf-8"

    def do_POST(self) -> None:
        """严格处理四个 POST 路径。"""

        if self.path not in ENDPOINTS:
            self._protocol_error(404)
            return
        if not self._content_type_valid(self.headers.get("Content-Type")):
            self._protocol_error(415)
            return
        encoding = self.headers.get("Content-Encoding")
        if encoding is not None and encoding.strip().lower() != "identity":
            self._protocol_error(415)
            return
        try:
            content_length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            self._protocol_error(400)
            return
        if content_length < 0:
            self._protocol_error(400)
            return
        if content_length > MAX_REQUEST_BODY_BYTES:
            self._protocol_error(413)
            return
        raw = self.rfile.read(content_length)
        try:
            payload = decode_request_json(raw)
        except ProtocolError:
            self._protocol_error(400)
            return
        if not self.server.engine.action_lock.acquire(blocking=False):
            self._protocol_error(409)
            return
        try:
            try:
                status, response = self.server.engine.process(self.path, payload)
            except Exception:
                # 不向客户端泄露本地真值或栈信息；附件规定可形成响应的内部错误
                # 使用 HTTP 500 和三个公共业务字段。
                status, response = self.server.engine.error_response(500)
        finally:
            self.server.engine.action_lock.release()
        self._send_json(status, response)

    def _wrong_method(self) -> None:
        """已知精确路径返回 405，其余路径返回 404。"""

        self._protocol_error(405 if self.path in ENDPOINTS else 404)

    do_GET = _wrong_method
    do_HEAD = _wrong_method
    do_PUT = _wrong_method
    do_DELETE = _wrong_method
    do_PATCH = _wrong_method
    do_OPTIONS = _wrong_method
    do_TRACE = _wrong_method


def run_local_simulator(
    problem: int,
    seed: int,
    robot_id: str,
    host: str,
    port: int,
    truth_output: Path,
) -> None:
    """生成案例、保存真值并持续运行本地 HTTP 服务。"""

    scenario = SimulatorScenario.generate(problem, seed)
    scenario.write_truth(truth_output)
    engine = SimulatorEngine(scenario, robot_id)
    server = SimulatorHTTPServer((host, port), engine)
    print(
        f"本地问题 {problem} 模拟器：http://{host}:{server.server_port}；"
        f"seed={seed}；robot_id={robot_id}；源数量={len(scenario.sources)}"
    )
    print(f"案例真值：{truth_output}")
    print("按 Ctrl+C 停止服务。")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n模拟器已停止。")
    finally:
        server.server_close()
