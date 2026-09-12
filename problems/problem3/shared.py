"""问题 3 两种策略共享的频道状态、覆盖集合、定位任务、执行器和日志。

本模块是策略与附件客户端之间的唯一状态更新层，确保每个真实返回只更新对应频道，
并统一统计移动、切频、检测和清除耗时。
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Literal

import numpy as np
from numpy.typing import NDArray

from config.constants import (
    BEARING_ERROR_DEG,
    CHANNEL_SWITCH_TIME_S,
    CLEARANCE_FAILURE_TIME_S,
    CLEARANCE_SUCCESS_TIME_S,
    MEASUREMENT_TIME_S,
    ROBOT_SPEED_MPS,
)
from problems.problem1.model import BearingMeasurement
from problems.problem2.config import ARENA_POLYGON_VERTICES
from problems.problem2.model import (
    build_posterior_grid,
    evaluate_candidate,
    expected_diameter_gain,
)
from problems.problem3.belief import ChannelBelief
from problems.problem3.config import Problem3Settings
from problems.problem3.coverage import arena_grid, math_distance
from radio_locator.client import ActionExchange, SimulatorClient
from radio_locator.geometry import (
    MinimumEnclosingCircle,
    intersect_bearing_wedges,
    minimum_enclosing_circle,
)

FloatArray = NDArray[np.float64]
ChannelStatus = Literal["unknown", "detected", "localized", "cleared", "absent"]


@dataclass(slots=True)
class ChannelTrack:
    """单频道从全域搜索到清除的全部在线状态。"""

    status: ChannelStatus = "unknown"
    measurements: list[BearingMeasurement] = field(default_factory=list)
    region: FloatArray | None = None
    clear_circle: MinimumEnclosingCircle | None = None
    first_detected_time_s: float | None = None
    localized_time_s: float | None = None
    cleared_time_s: float | None = None
    last_detection_probability: float = 0.0


@dataclass(slots=True)
class RunCounters:
    """问题 3 汇总表所需的动作与距离计数器。"""

    movement_distance_m: float = 0.0
    measure_count: int = 0
    switch_count: int = 0
    clear_success_count: int = 0
    clear_failure_count: int = 0


@dataclass(frozen=True, slots=True)
class PlannedAction:
    """MPC和滚动策略共用的有限候选动作。"""

    kind: Literal["measure", "clear", "batch_measure"]
    position: tuple[float, float]
    channels: tuple[int, ...]
    reason: str
    information_gain_bits: float = 0.0
    q_value_s: float | None = None
    estimated_remaining_time_s: float | None = None


class JsonlRunLogger:
    """逐步追加JSONL；程序异常时已完成动作仍保留在磁盘。"""

    def __init__(self, path: Path) -> None:
        self.path = path
        path.parent.mkdir(parents=True, exist_ok=True)
        self._file = path.open("w", encoding="utf-8")

    def write(self, record: dict[str, Any]) -> None:
        """写入并立即刷新一个动作记录。"""

        self._file.write(json.dumps(record, ensure_ascii=False) + "\n")
        self._file.flush()

    def close(self) -> None:
        """关闭日志文件。"""

        self._file.close()


class Problem3State:
    """20频道状态、离散保证覆盖集合和联合粒子信念。"""

    def __init__(self, settings: Problem3Settings) -> None:
        self.settings = settings
        self.position = settings.start_position
        self.current_channel = settings.start_channel
        self.virtual_time_s = 0.0
        self.remaining_real_duration_s = 0
        self.tracks = {channel: ChannelTrack() for channel in settings.channels}
        self.coverage_grid = arena_grid(
            settings.arena_radius_m, settings.coverage_grid_step_m
        )
        self.uncovered = {
            channel: np.ones(len(self.coverage_grid), dtype=bool)
            for channel in settings.channels
        }
        self.beliefs = {
            channel: ChannelBelief.initialize(settings, channel)
            for channel in settings.channels
        }
        self.counters = RunCounters()

    def status_snapshot(self) -> dict[str, ChannelStatus]:
        """返回适合日志序列化的频道状态。"""

        return {str(channel): track.status for channel, track in self.tracks.items()}

    def unresolved_channels(self) -> list[int]:
        """返回尚未证明absent或成功cleared的频道。"""

        return [
            channel
            for channel, track in self.tracks.items()
            if track.status not in {"cleared", "absent"}
        ]

    def scan_needed(self, channel: int, position: tuple[float, float]) -> bool:
        """检测点的1000 m圆是否能严格减少该unknown频道的 `U_j`。"""

        if self.tracks[channel].status != "unknown":
            return False
        distances = np.linalg.norm(
            self.coverage_grid - np.asarray(position, dtype=float), axis=1
        )
        return bool(
            np.any(
                self.uncovered[channel]
                & (distances <= self.settings.guaranteed_radius_m + 1e-9)
            )
        )

    def apply_no_signal(self, channel: int, position: tuple[float, float]) -> None:
        """以保证半径排除 `Omega(M,1000)`，只在 `U_j` 空时标 absent。"""

        self.beliefs[channel].update(position, "no_signal")
        track = self.tracks[channel]
        if track.status != "unknown":
            return
        distances = np.linalg.norm(
            self.coverage_grid - np.asarray(position, dtype=float), axis=1
        )
        self.uncovered[channel] &= distances > self.settings.guaranteed_radius_m + 1e-9
        if not np.any(self.uncovered[channel]):
            track.status = "absent"

    def apply_direction(
        self,
        channel: int,
        position: tuple[float, float],
        bearing_deg: float,
    ) -> None:
        """更新单频道粒子、示向度半平面交和20 m最小覆盖圆条件。"""

        belief = self.beliefs[channel]
        belief.update(position, "direction", bearing_deg)
        track = self.tracks[channel]
        track.last_detection_probability = belief.existence_probability
        track.measurements.append(BearingMeasurement(position, bearing_deg))
        if track.status == "unknown":
            track.status = "detected"
            track.first_detected_time_s = self.virtual_time_s
        stations = [measurement.station for measurement in track.measurements]
        bearings = [measurement.bearing_deg for measurement in track.measurements]
        track.region = intersect_bearing_wedges(
            stations,
            bearings,
            BEARING_ERROR_DEG,
            self.settings.arena_radius_m,
            ARENA_POLYGON_VERTICES,
        )
        if len(track.region):
            track.clear_circle = minimum_enclosing_circle(track.region)
            if track.clear_circle.radius <= self.settings.clearance_radius_m + 1e-9:
                track.status = "localized"
                if track.localized_time_s is None:
                    track.localized_time_s = self.virtual_time_s

    def apply_near(self, channel: int, position: tuple[float, float]) -> None:
        """记录near观测并把当前位置直接设为清除任务点。"""

        self.beliefs[channel].update(position, "near")
        track = self.tracks[channel]
        if track.first_detected_time_s is None:
            track.first_detected_time_s = self.virtual_time_s
        track.status = "localized"
        track.localized_time_s = self.virtual_time_s
        track.clear_circle = MinimumEnclosingCircle(
            np.asarray(position, dtype=float), 0.0
        )

    def apply_clear(self, channel: int, success: bool) -> None:
        """成功清除终结频道；失败则保留detected状态继续定位。"""

        track = self.tracks[channel]
        if success:
            track.status = "cleared"
            track.cleared_time_s = self.virtual_time_s
        elif track.status == "localized":
            track.status = "detected"

    def coverage_fraction(self, channel: int) -> float:
        """返回该频道已完成1000 m保证扫描的离散网格比例。"""

        return float(1.0 - np.mean(self.uncovered[channel]))

    def all_resolved(self) -> bool:
        """判断所有频道是否均为cleared或由覆盖证明absent。"""

        return all(
            track.status in {"cleared", "absent"} for track in self.tracks.values()
        )


class Problem3Executor:
    """串行执行真实接口动作、更新状态并记录完整耗时分解。"""

    def __init__(
        self,
        client: SimulatorClient,
        state: Problem3State,
        logger: JsonlRunLogger,
        strategy_name: str,
    ) -> None:
        self.client = client
        self.state = state
        self.logger = logger
        self.strategy_name = strategy_name

    def enter(self) -> None:
        """连接检查后进入；缺字段或拒绝直接抛错。"""

        self.client.check_connection()
        exchange = self.client.enter()
        response = exchange.response
        for field_name in (
            "virtual_time_s",
            "remaining_real_duration_s",
            "max_virtual_duration_s",
            "max_real_duration_s",
        ):
            if field_name not in response:
                raise RuntimeError(f"/enter response missing {field_name}")
        self.state.position = self.state.settings.start_position
        self.state.current_channel = self.state.settings.start_channel
        self.state.virtual_time_s = float(response["virtual_time_s"])
        if abs(self.state.virtual_time_s) > 1e-9:
            raise RuntimeError("/enter did not initialize virtual time to zero")
        self.state.remaining_real_duration_s = int(response["remaining_real_duration_s"])
        self._log_exchange(exchange, "enter", (), "进入测试", 0.0, 0.0, 0.0)

    def measure(
        self,
        position: tuple[float, float],
        channel: int,
        reason: str,
        information_gain_bits: float = 0.0,
        q_value_s: float | None = None,
        estimated_remaining_time_s: float | None = None,
    ) -> str:
        """执行一次measure并只更新目标频道；near会立即调用clear。"""

        old_position = self.state.position
        old_virtual_time = self.state.virtual_time_s
        move_distance = math_distance(old_position, position)
        move_time = move_distance / ROBOT_SPEED_MPS
        switch_time = (
            CHANNEL_SWITCH_TIME_S
            if channel != self.state.current_channel
            else 0.0
        )
        exchange = self.client.measure(position, channel)
        self.state.counters.movement_distance_m += move_distance
        self.state.counters.measure_count += 1
        self.state.counters.switch_count += int(switch_time > 0.0)
        self.state.position = position
        self.state.current_channel = channel
        self.state.virtual_time_s = float(exchange.response["virtual_time_s"])
        expected_elapsed = move_time + switch_time + MEASUREMENT_TIME_S
        if not math.isclose(
            self.state.virtual_time_s - old_virtual_time,
            expected_elapsed,
            abs_tol=2e-5,
        ):
            raise RuntimeError("/measure virtual time violates movement/switch/5s rule")
        result = str(exchange.response["measure_result"])
        if result == "no_signal":
            self.state.apply_no_signal(channel, position)
        elif result == "direction":
            self.state.apply_direction(
                channel, position, float(exchange.response["svd_deg"])
            )
        elif result == "near":
            self.state.apply_near(channel, position)
        else:
            raise RuntimeError(f"unknown measure_result: {result}")
        self._log_exchange(
            exchange,
            "measure",
            (channel,),
            reason,
            move_time,
            switch_time,
            MEASUREMENT_TIME_S,
            information_gain_bits,
            q_value_s,
            estimated_remaining_time_s,
        )
        if result == "near":
            self.clear(position, channel, "near触发当前位置立即清除")
        return result

    def clear(
        self,
        position: tuple[float, float],
        channel: int,
        reason: str,
        q_value_s: float | None = None,
        estimated_remaining_time_s: float | None = None,
    ) -> bool:
        """执行clear；保持当前测向频道不变。"""

        old_position = self.state.position
        old_virtual_time = self.state.virtual_time_s
        move_distance = math_distance(old_position, position)
        move_time = move_distance / ROBOT_SPEED_MPS
        old_channel = self.state.current_channel
        exchange = self.client.clear(position, channel)
        self.state.counters.movement_distance_m += move_distance
        self.state.position = position
        self.state.virtual_time_s = float(exchange.response["virtual_time_s"])
        clear_result = exchange.response["clear_result"]
        if clear_result not in {"success", "no_target_in_range"}:
            raise RuntimeError(f"unknown clear_result: {clear_result}")
        success = clear_result == "success"
        action_time = CLEARANCE_SUCCESS_TIME_S if success else CLEARANCE_FAILURE_TIME_S
        if not math.isclose(
            self.state.virtual_time_s - old_virtual_time,
            move_time + action_time,
            abs_tol=2e-5,
        ):
            raise RuntimeError("/clear virtual time violates movement/3-or-5s rule")
        if success:
            self.state.counters.clear_success_count += 1
        else:
            self.state.counters.clear_failure_count += 1
        self.state.apply_clear(channel, success)
        if self.state.current_channel != old_channel:
            raise RuntimeError("/clear illegally changed local receiver channel")
        self._log_exchange(
            exchange,
            "clear",
            (channel,),
            reason,
            move_time,
            0.0,
            action_time,
            0.0,
            q_value_s,
            estimated_remaining_time_s,
        )
        return success

    def exit(self) -> None:
        """主动退出并记录最终原始响应。"""

        exchange = self.client.exit()
        self.state.virtual_time_s = float(exchange.response["virtual_time_s"])
        self._log_exchange(exchange, "exit", (), "策略结束", 0.0, 0.0, 0.0)

    def _log_exchange(
        self,
        exchange: ActionExchange,
        action_type: str,
        channels: tuple[int, ...],
        reason: str,
        move_time_s: float,
        switch_time_s: float,
        operation_time_s: float,
        information_gain_bits: float = 0.0,
        q_value_s: float | None = None,
        estimated_remaining_time_s: float | None = None,
    ) -> None:
        """记录共享要求列出的动作、信念、Q值和状态快照。"""

        target_position = exchange.request.get("position")
        channel = channels[0] if channels else None
        belief = self.state.beliefs.get(channel) if channel is not None else None
        self.logger.write(
            {
                "strategy": self.strategy_name,
                "virtual_time_s": self.state.virtual_time_s,
                "position": {"x": self.state.position[0], "y": self.state.position[1]},
                "current_channel": self.state.current_channel,
                "action_type": action_type,
                "target_position": target_position,
                "target_channels": list(channels),
                "time_breakdown_s": {
                    "movement": move_time_s,
                    "channel_switch": switch_time_s,
                    "measure_or_clear": operation_time_s,
                    "total": move_time_s + switch_time_s + operation_time_s,
                },
                "raw_request": exchange.request,
                "raw_response": exchange.response,
                "channel_status": self.state.status_snapshot(),
                "detection_probability": (
                    belief.existence_probability if belief is not None else None
                ),
                "information_gain_bits": information_gain_bits,
                "q_value_s": q_value_s,
                "estimated_remaining_time_s": estimated_remaining_time_s,
                "reason": reason,
            }
        )


def choose_localization_point(
    state: Problem3State,
    channel: int,
) -> tuple[tuple[float, float], str]:
    """复用问题2后验/直径增益，为已检测频道选择下一检测点。"""

    track = state.tracks[channel]
    if not track.measurements:
        raise ValueError("localization requires at least one direction measurement")
    if len(track.measurements) == 1:
        candidates = localization_candidate_points(state, channel, 1)
        if candidates:
            return candidates[0]
        first = track.measurements[0]
        posterior = build_posterior_grid(first)
        support_circle = minimum_enclosing_circle(posterior.points)
        return (
            (float(support_circle.center[0]), float(support_circle.center[1])),
            "问题2候选未通过粗筛，回退到首次后验最小覆盖圆中心",
        )
    if track.clear_circle is None:
        raise RuntimeError("localized channel has no current region circle")
    return (
        (float(track.clear_circle.center[0]), float(track.clear_circle.center[1])),
        f"在当前定位区域最小覆盖圆中心复测，rho={track.clear_circle.radius:.2f} m",
    )


def localization_candidate_points(
    state: Problem3State,
    channel: int,
    count: int,
) -> list[tuple[tuple[float, float], str]]:
    """复用问题2候选评价，返回首次测向后的前K个定位点。"""

    track = state.tracks[channel]
    if count <= 0 or not track.measurements:
        return []
    if len(track.measurements) == 1:
        first = track.measurements[0]
        try:
            posterior = build_posterior_grid(first)
        except ValueError:
            # 示向楔形很窄时，固定笛卡尔网格可能恰好没有节点落入其中。
            # 此时使用解析保底点 M1+750u±600v；在±1°误差和1500 m
            # 首次可能距离下，两点到真实源的最坏距离仍小于1000 m。
            direction = math.radians(first.bearing_deg)
            unit = np.array([math.cos(direction), math.sin(direction)])
            perpendicular = np.array([-math.sin(direction), math.cos(direction)])
            station = np.asarray(first.station, dtype=float)
            fallback = [
                station + 750.0 * unit + sign * 600.0 * perpendicular
                for sign in (-1.0, 1.0)
            ]
            return [
                (
                    (float(point[0]), float(point[1])),
                    "首次后验网格为空，采用750u±600v保证复测点",
                )
                for point in fallback[:count]
            ]
        support_circle = minimum_enclosing_circle(posterior.points)
        direction = math.radians(first.bearing_deg)
        perpendicular = np.array([-math.sin(direction), math.cos(direction)])
        max_offset = math.sqrt(
            max(
                state.settings.guaranteed_radius_m**2
                - support_circle.radius**2,
                0.0,
            )
        )
        fractions = np.linspace(0.65, 0.95, max(2, count))
        candidates = [
            support_circle.center + sign * fraction * max_offset * perpendicular
            for fraction in fractions
            for sign in (-1.0, 1.0)
        ]
        scored: list[tuple[float, tuple[float, float]]] = []
        for candidate in candidates:
            point = (float(candidate[0]), float(candidate[1]))
            score = evaluate_candidate(point, first, posterior)
            if score.feasible:
                gain = expected_diameter_gain(score, first, posterior)
                scored.append((gain, point))
        if scored:
            scored.sort(key=lambda item: item[0], reverse=True)
            return [
                (point, f"问题2后验期望直径增益前K候选，gain={gain:.3f} bit")
                for gain, point in scored[:count]
            ]
        return []
    if track.clear_circle is None:
        return []
    center = (float(track.clear_circle.center[0]), float(track.clear_circle.center[1]))
    return [(center, f"当前定位区域最小覆盖圆中心，rho={track.clear_circle.radius:.2f} m")]


def _point_to_segment_distance(
    point: np.ndarray,
    first: np.ndarray,
    second: np.ndarray,
) -> float:
    """计算点到闭线段的欧氏距离。"""

    direction = second - first
    squared_length = float(np.dot(direction, direction))
    if squared_length <= 1e-18:
        return float(np.linalg.norm(point - first))
    ratio = float(np.dot(point - first, direction) / squared_length)
    ratio = min(max(ratio, 0.0), 1.0)
    return float(np.linalg.norm(point - (first + ratio * direction)))


def _point_in_convex_polygon(point: np.ndarray, polygon: FloatArray) -> bool:
    """判断点是否位于逆时针凸多边形内或边界上。"""

    if len(polygon) < 3:
        return False
    edges = np.roll(polygon, -1, axis=0) - polygon
    offsets = point - polygon
    crosses = edges[:, 0] * offsets[:, 1] - edges[:, 1] * offsets[:, 0]
    return bool(np.all(crosses >= -1e-9) or np.all(crosses <= 1e-9))


def fallback_clearance_grid(
    region: FloatArray,
    step_m: float,
    clearance_radius_m: float,
    current_position: tuple[float, float],
) -> list[tuple[float, float]]:
    """生成覆盖定位区域的方格中心，并按蛇形开放路线排序。

    网格中心从外接矩形左下方半个步长处开始。仅保留位于多边形内，或到
    多边形边界不超过清除半径的中心；真实位置所在网格单元的中心必被保留。
    """

    polygon = np.asarray(region, dtype=float)
    if not len(polygon):
        return []
    half = step_m / 2.0
    x_values = np.arange(
        float(np.min(polygon[:, 0])) - half,
        float(np.max(polygon[:, 0])) + half + step_m * 0.5,
        step_m,
    )
    y_values = np.arange(
        float(np.min(polygon[:, 1])) - half,
        float(np.max(polygon[:, 1])) + half + step_m * 0.5,
        step_m,
    )
    rows: list[list[tuple[float, float]]] = []
    for row_index, y_value in enumerate(y_values):
        row: list[tuple[float, float]] = []
        for x_value in x_values:
            point = np.asarray((x_value, y_value), dtype=float)
            inside = _point_in_convex_polygon(point, polygon)
            boundary_distance = min(
                _point_to_segment_distance(
                    point, polygon[index], polygon[(index + 1) % len(polygon)]
                )
                for index in range(len(polygon))
            )
            if inside or boundary_distance <= clearance_radius_m + 1e-9:
                row.append((float(x_value), float(y_value)))
        if row_index % 2:
            row.reverse()
        if row:
            rows.append(row)
    route = [point for row in rows for point in row]
    if route and math_distance(current_position, route[-1]) < math_distance(
        current_position, route[0]
    ):
        route.reverse()
    return route


def clear_with_fallback_grid(
    executor: Problem3Executor,
    channel: int,
    reason: str,
) -> None:
    """用边长28 m方格覆盖剩余定位区域，直到该频道清除成功。"""

    state = executor.state
    track = state.tracks[channel]
    if track.region is None or not len(track.region):
        return
    route = fallback_clearance_grid(
        track.region,
        state.settings.fallback_grid_step_m,
        state.settings.clearance_radius_m,
        state.position,
    )
    for index, point in enumerate(route, start=1):
        if executor.clear(
            point,
            channel,
            f"{reason}；28m网格兜底第{index}/{len(route)}点",
        ):
            return


def localize_channel(
    executor: Problem3Executor,
    channel: int,
    insertion_reason: str,
    *,
    clear_when_ready: bool,
    clear_radius_m: float,
    use_grid_fallback: bool,
) -> None:
    """执行有限次测向；可立即清除、延期清除或转入28 m网格兜底。"""

    state = executor.state
    for _ in range(state.settings.localization_max_measurements):
        track = state.tracks[channel]
        if track.status == "cleared":
            return
        if (
            track.clear_circle is not None
            and track.clear_circle.radius <= clear_radius_m + 1e-9
        ):
            track.status = "localized"
            if not clear_when_ready:
                return
            center = (
                float(track.clear_circle.center[0]),
                float(track.clear_circle.center[1]),
            )
            executor.clear(
                center,
                channel,
                f"{insertion_reason}；最小覆盖圆rho={track.clear_circle.radius:.2f}"
                f"<={clear_radius_m:.1f}",
            )
            if state.tracks[channel].status == "cleared":
                return
            break
        point, reason = choose_localization_point(state, channel)
        information_gain = state.beliefs[channel].information_gain_bits(point)
        executor.measure(
            point,
            channel,
            f"{insertion_reason}；{reason}",
            information_gain_bits=information_gain,
        )
    track = state.tracks[channel]
    if (
        track.status != "cleared"
        and track.clear_circle is not None
        and track.clear_circle.radius <= clear_radius_m + 1e-9
    ):
        track.status = "localized"
        if not clear_when_ready:
            return
        center = (
            float(track.clear_circle.center[0]),
            float(track.clear_circle.center[1]),
        )
        if executor.clear(
            center,
            channel,
            f"{insertion_reason}；最终测向后最小覆盖圆"
            f"rho={track.clear_circle.radius:.2f}<={clear_radius_m:.1f}",
        ):
            return
    if use_grid_fallback and state.tracks[channel].status != "cleared":
        clear_with_fallback_grid(executor, channel, f"{insertion_reason}；常规定位达到上限")


def localize_and_clear_channel(
    executor: Problem3Executor,
    channel: int,
    insertion_reason: str,
) -> None:
    """兼容MPC原行为：最多测向6次，并在20 m最小覆盖圆中心清除。"""

    localize_channel(
        executor,
        channel,
        insertion_reason,
        clear_when_ready=True,
        clear_radius_m=executor.state.settings.clearance_radius_m,
        use_grid_fallback=False,
    )


def summarize_state(
    state: Problem3State,
    strategy: str,
    coverage_plan: dict[str, Any],
    original_log_names: list[str] | None = None,
) -> dict[str, Any]:
    """生成题目要求的清除、时间、动作、频道和覆盖汇总。"""

    cleared = state.counters.clear_success_count
    all_resolved = state.all_resolved()
    reached_source_upper_bound = cleared >= 16
    ratio = 1.0 if all_resolved or reached_source_upper_bound else cleared / 16.0
    localization_durations = [
        track.cleared_time_s - track.first_detected_time_s
        for track in state.tracks.values()
        if track.cleared_time_s is not None and track.first_detected_time_s is not None
    ]
    return {
        "strategy": strategy,
        "all_channels_resolved": all_resolved,
        "cleared_count": cleared,
        "clearance_ratio": ratio,
        "clearance_ratio_denominator": (
            "inferred completion because all channels resolved or 16 sources cleared"
            if all_resolved or reached_source_upper_bound
            else "conservative upper bound 16"
        ),
        "total_virtual_time_s": state.virtual_time_s,
        "average_localize_clear_time_s": (
            float(np.mean(localization_durations)) if localization_durations else None
        ),
        "total_movement_distance_m": state.counters.movement_distance_m,
        "measure_count": state.counters.measure_count,
        "switch_count": state.counters.switch_count,
        "clear_success_count": state.counters.clear_success_count,
        "clear_failure_count": state.counters.clear_failure_count,
        "channels": {
            str(channel): {
                "status": track.status,
                "first_detected_time_s": track.first_detected_time_s,
                "localized_time_s": track.localized_time_s,
                "cleared_time_s": track.cleared_time_s,
                "guaranteed_coverage_fraction": state.coverage_fraction(channel),
            }
            for channel, track in state.tracks.items()
        },
        "coverage_plan": coverage_plan,
        "official_log_original_names": original_log_names or [],
    }
