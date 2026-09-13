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
from problems.problem4.belief import ChannelBelief, calibrate_existence_count
from problems.problem4.config import Problem4Settings
from problems.problem4.coverage import arena_grid, math_distance
from radio_locator.client import ActionExchange, SimulatorClient
from radio_locator.geometry import (
    MinimumEnclosingCircle,
    intersect_bearing_wedges,
    minimum_enclosing_circle,
)

FloatArray = NDArray[np.float64]
ChannelStatus = Literal["unknown", "detected", "localized", "cleared", "absent"]


@dataclass(slots=True)
class ChannelState:
    """单频道唯一状态对象。

    参数：``channel_id``、固定半径联合信念和覆盖掩码由 :class:`problem4State`
    初始化。所有观测、连续几何、任务和状态转换只写入本对象；兼容属性 ``tracks``、
    ``beliefs``、``uncovered`` 仅是对本对象字段的只读映射视图。
    """

    channel_id: int
    joint_belief: ChannelBelief
    coverage_remaining: NDArray[np.bool_]
    status: ChannelStatus = "unknown"
    observation_history: list[BearingMeasurement] = field(default_factory=list)
    support_region: FloatArray | None = None
    credible_region: FloatArray | None = None
    bearing_regions: list[FloatArray] = field(default_factory=list)
    localization_center: tuple[float, float] | None = None
    localization_radius: float | None = None
    pending_tasks: set[str] = field(default_factory=set)
    last_observation: str | None = None
    clear_exclusion_circles: list[tuple[tuple[float, float], float]] = field(default_factory=list)
    first_detected_time_s: float | None = None
    localized_time_s: float | None = None
    cleared_time_s: float | None = None
    last_detection_probability: float = 0.0
    last_measurement_time_s: float | None = None
    last_clear_failure_time_s: float | None = None
    measured_positions: set[tuple[float, float]] = field(default_factory=set)

    @property
    def existence_probability(self) -> float:
        """返回统一联合信念的存在边缘概率。"""

        return self.joint_belief.existence_probability

    @property
    def measurements(self) -> list[BearingMeasurement]:
        """兼容旧定位辅助函数的观测历史别名。"""

        return self.observation_history

    @property
    def region(self) -> FloatArray | None:
        """兼容旧几何调用的严格support区域别名。"""

        return self.support_region

    @region.setter
    def region(self, value: FloatArray | None) -> None:
        self.support_region = value

    @property
    def clear_circle(self) -> MinimumEnclosingCircle | None:
        if self.localization_center is None or self.localization_radius is None:
            return None
        return MinimumEnclosingCircle(np.asarray(self.localization_center), self.localization_radius)

    @clear_circle.setter
    def clear_circle(self, value: MinimumEnclosingCircle | None) -> None:
        if value is None:
            self.localization_center = None; self.localization_radius = None
        else:
            self.localization_center = (float(value.center[0]), float(value.center[1]))
            self.localization_radius = float(value.radius)


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
    immediate_time_s: float | None = None
    coverage_gain: float = 0.0
    localization_gain: float = 0.0
    efficiency: float = 0.0
    lower_bound_s: float | None = None
    upper_bound_s: float | None = None


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


class problem4State:
    """20频道状态、离散保证覆盖集合和联合粒子信念。"""

    def __init__(self, settings: Problem4Settings) -> None:
        self.settings = settings
        self.position = settings.start_position
        self.current_channel = settings.start_channel
        self.virtual_time_s = 0.0
        self.max_virtual_duration_s = 360_000.0
        self.remaining_real_duration_s = 0
        self.coverage_grid = arena_grid(
            settings.arena_radius_m, settings.coverage_grid_step_m
        )
        self.channel_states = {
            channel: ChannelState(
                channel,
                ChannelBelief.initialize(settings, channel),
                np.ones(len(self.coverage_grid), dtype=bool),
            )
            for channel in settings.channels
        }
        self.counters = RunCounters()
        self.no_progress_steps = 0

    @property
    def tracks(self) -> dict[int, ChannelState]:
        """旧调用兼容视图；不持有第二份频道状态。"""
        return self.channel_states

    @property
    def beliefs(self) -> dict[int, ChannelBelief]:
        """旧调用兼容视图；每项就是对应ChannelState.joint_belief。"""
        return {channel: item.joint_belief for channel, item in self.channel_states.items()}

    @property
    def uncovered(self) -> dict[int, NDArray[np.bool_]]:
        """旧调用兼容视图；每项就是对应ChannelState.coverage_remaining。"""
        return {channel: item.coverage_remaining for channel, item in self.channel_states.items()}

    def calibrate_channel_count(self) -> None:
        """用10～16总数约束温和校正存在概率，不改变确定性频道状态。"""

        calibrate_existence_count(
            self.beliefs,
            {channel: track.status for channel, track in self.tracks.items()},
        )

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
        if position in self.channel_states[channel].measured_positions:
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
        """兼容入口；真实更新统一委托给 :meth:`update_channel_state`。"""
        self.update_channel_state(channel, "measure", {"result": "no_signal", "position": position})

    def apply_direction(
        self,
        channel: int,
        position: tuple[float, float],
        bearing_deg: float,
    ) -> None:
        """兼容入口；真实更新统一委托给 :meth:`update_channel_state`。"""
        self.update_channel_state(channel, "measure", {"result": "direction", "position": position, "bearing_deg": bearing_deg})

    def apply_near(self, channel: int, position: tuple[float, float]) -> None:
        """兼容入口；真实更新统一委托给 :meth:`update_channel_state`。"""
        self.update_channel_state(channel, "measure", {"result": "near", "position": position})

    def apply_clear(self, channel: int, success: bool) -> None:
        """兼容入口；真实更新统一委托给 :meth:`update_channel_state`。"""
        self.update_channel_state(channel, "clear", {"success": success, "position": self.position})

    def update_channel_state(self, channel_id: int, action: str, observation: dict[str, Any]) -> None:
        """唯一频道状态更新入口。

        参数：频道、真实动作种类和附件返回的规范化观测。返回值为None。
        副作用：原子信念、严格support区域、可信区域、覆盖U、任务集合和状态只在对应
        ``ChannelState`` 中同步更新；任何非法的cleared后观测都会被拒绝。
        """
        item = self.channel_states[channel_id]
        if item.status == "cleared" and action != "clear":
            raise RuntimeError("cleared channel cannot receive a new measurement task")
        position = tuple(observation.get("position", self.position))
        if action == "measure":
            result = str(observation["result"])
            item.last_observation = result
            item.last_measurement_time_s = self.virtual_time_s
            item.measured_positions.add(position)
            if result == "no_signal":
                item.joint_belief.update(position, "no_signal")
                if item.status == "unknown":
                    distances = np.linalg.norm(self.coverage_grid - np.asarray(position), axis=1)
                    item.coverage_remaining &= distances > self.settings.guaranteed_radius_m + 1e-9
                    if not np.any(item.coverage_remaining):
                        item.status = "absent"; item.pending_tasks.clear()
                self.no_progress_steps += 1
            elif result == "direction":
                bearing = float(observation["bearing_deg"])
                item.joint_belief.update(position, "direction", bearing)
                item.joint_belief.force_exists()
                item.last_detection_probability = 1.0
                item.observation_history.append(BearingMeasurement(position, bearing))
                item.joint_belief.restrict_to_bearings(
                    [(measurement.station, measurement.bearing_deg) for measurement in item.observation_history]
                )
                if item.first_detected_time_s is None: item.first_detected_time_s = self.virtual_time_s
                item.status = "detected"
                stations=[m.station for m in item.observation_history]; bearings=[m.bearing_deg for m in item.observation_history]
                item.support_region = intersect_bearing_wedges(stations,bearings,BEARING_ERROR_DEG,self.settings.arena_radius_m,ARENA_POLYGON_VERTICES)
                item.bearing_regions.append(item.support_region.copy())
                item.credible_region = item.joint_belief.credible_points(0.99)
                self._assert_posterior_in_support(item)
                if len(item.support_region):
                    circle=minimum_enclosing_circle(item.support_region); item.clear_circle=circle
                    if circle.radius <= self.settings.clearance_radius_m + 1e-9:
                        item.status="localized"; item.localized_time_s=item.localized_time_s or self.virtual_time_s; item.pending_tasks={"clear"}
                    else: item.pending_tasks={"localize"}
                self.no_progress_steps=0
            elif result == "near":
                item.joint_belief.update(position,"near"); item.joint_belief.force_exists()
                item.last_detection_probability=1.0; item.first_detected_time_s=item.first_detected_time_s or self.virtual_time_s
                item.status="localized"; item.localized_time_s=self.virtual_time_s
                item.support_region=np.asarray([position],dtype=float); item.credible_region=np.asarray([position],dtype=float)
                item.clear_circle=MinimumEnclosingCircle(np.asarray(position,dtype=float),0.0); item.pending_tasks={"clear"}; self.no_progress_steps=0
            else: raise ValueError(f"unsupported measure observation {result}")
        elif action == "clear":
            success=bool(observation["success"]); item.last_observation="clear_success" if success else "clear_failure"
            item.joint_belief.force_exists()
            if success:
                item.status="cleared"; item.cleared_time_s=self.virtual_time_s; item.pending_tasks.clear()
            else:
                item.status="detected"; item.last_clear_failure_time_s=self.virtual_time_s
                item.clear_exclusion_circles.append((position,self.settings.clearance_radius_m))
                item.joint_belief.exclude_clear_circle(position,self.settings.clearance_radius_m)
                item.credible_region=item.joint_belief.credible_points(0.99); item.pending_tasks={"localize"}
            self.no_progress_steps=0 if success else self.no_progress_steps+1
        else: raise ValueError(f"unsupported channel action {action}")
        self.calibrate_channel_count(); self._assert_channel_consistency(item)

    def _assert_posterior_in_support(self, item: ChannelState) -> None:
        """验证direction后正权重离散点没有落到严格扇形区域外。"""
        if item.support_region is None or not len(item.support_region): return
        # 用每条历史测向直接复核，避免依赖多边形点包含库。
        for measurement in item.observation_history:
            vectors=item.joint_belief.points-np.asarray(measurement.station)
            angles=np.degrees(np.arctan2(vectors[:,1],vectors[:,0]))%360
            delta=np.abs((angles-measurement.bearing_deg+180)%360-180)
            invalid=item.joint_belief.exists & (item.joint_belief.weights>1e-12) & (delta>BEARING_ERROR_DEG+.02)
            if np.any(invalid): raise RuntimeError("posterior_support is outside support_region")

    def _assert_channel_consistency(self, item: ChannelState) -> None:
        """检查概率归一、状态转换和确定定位的几何判据。"""
        if not math.isclose(float(np.sum(item.joint_belief.weights)),1.0,abs_tol=1e-8): raise RuntimeError("joint belief is not normalized")
        if item.last_observation in {"direction","near","clear_failure"} and not math.isclose(item.existence_probability,1.0,abs_tol=1e-10): raise RuntimeError("confirmed channel must have P(Z=1)=1")
        if item.status=="localized" and (item.localization_radius is None or item.localization_radius>self.settings.clearance_radius_m+1e-9): raise RuntimeError("localized requires support MEC radius <=20")

    def coverage_fraction(self, channel: int) -> float:
        """返回该频道已完成1000 m保证扫描的离散网格比例。"""

        return float(1.0 - np.mean(self.uncovered[channel]))

    def all_resolved(self) -> bool:
        """判断所有频道是否均为cleared或由覆盖证明absent。"""

        return all(
            track.status in {"cleared", "absent"} for track in self.tracks.values()
        )

    def should_terminate(self) -> bool:
        """仅在全频道有确定结论或已达到题面16个源上限时提前终止。"""

        return self.all_resolved() or self.counters.clear_success_count >= 16


class problem4Executor:
    """串行执行真实接口动作、更新状态并记录完整耗时分解。"""

    def __init__(
        self,
        client: SimulatorClient,
        state: problem4State,
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
        self.state.max_virtual_duration_s = float(response["max_virtual_duration_s"])
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

    def log_planning(
        self,
        candidates: list[PlannedAction],
        selected: PlannedAction,
        reason: str,
    ) -> None:
        """在动作前记录完整候选表；此记录不调用接口、也不推进虚拟时间。"""

        def serialize(action: PlannedAction) -> dict[str, Any]:
            return {
                "kind": action.kind,
                "position": list(action.position),
                "channels": list(action.channels),
                "tau_s": action.immediate_time_s,
                "information_gain_bits": action.information_gain_bits,
                "coverage_gain": action.coverage_gain,
                "localization_gain": action.localization_gain,
                "efficiency": action.efficiency,
                "estimated_remaining_time_s": action.estimated_remaining_time_s,
                "q_value_s": action.q_value_s,
                "lower_bound_s": action.lower_bound_s,
                "upper_bound_s": action.upper_bound_s,
                "reason": action.reason,
            }

        self.logger.write(
            {
                "strategy": self.strategy_name,
                "record_type": "planning",
                "virtual_time_s": self.state.virtual_time_s,
                "position": {"x": self.state.position[0], "y": self.state.position[1]},
                "current_channel": self.state.current_channel,
                "candidates": [serialize(action) for action in candidates],
                "selected": serialize(selected),
                "selection_reason": reason,
            }
        )

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
                "belief_metrics": (
                    belief.geometry_metrics(self.state.settings.clear_probability)
                    if belief is not None else None
                ),
            }
        )


def choose_localization_point(
    state: problem4State,
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
    state: problem4State,
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
            # 问题2的规则网格可能恰好没有点落入很窄的±1°带；联合信念仍有
            # 固定R后验，使用其均值与MAP继续滚动，不能让在线策略中断。
            belief = state.beliefs[channel]
            fallback = [belief.representative_point(), belief.map_point()]
            return [
                (point, "问题2网格为空，回退到联合信念高后验位置")
                for point in dict.fromkeys(fallback)
            ][:count]
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


def localize_and_clear_channel(
    executor: problem4Executor,
    channel: int,
    insertion_reason: str,
) -> None:
    """滚动执行定位，且只在最小覆盖圆半径<=20 m或near时清除。"""

    state = executor.state
    attempted_points: set[tuple[float, float]] = set()
    for _ in range(state.settings.localization_max_measurements):
        track = state.tracks[channel]
        if track.status == "cleared":
            return
        if track.status == "localized" and track.clear_circle is not None:
            center = (
                float(track.clear_circle.center[0]),
                float(track.clear_circle.center[1]),
            )
            executor.clear(
                center,
                channel,
                f"{insertion_reason}；最小覆盖圆rho={track.clear_circle.radius:.2f}<=20",
            )
            return
        point, reason = choose_localization_point(state, channel)
        if point in attempted_points or point in track.measured_positions:
            # 问题2后验/直径增益可能反复给同一点；继续测只会空耗时间。
            return
        attempted_points.add(point)
        information_gain = state.beliefs[channel].information_gain_bits(point)
        executor.measure(
            point,
            channel,
            f"{insertion_reason}；{reason}",
            information_gain_bits=information_gain,
        )


def summarize_state(
    state: problem4State,
    strategy: str,
    coverage_plan: dict[str, Any],
    original_log_names: list[str] | None = None,
    known_source_total: int | None = None,
) -> dict[str, Any]:
    """生成题目要求的清除、时间、动作、频道和覆盖汇总。"""

    cleared = state.counters.clear_success_count
    all_resolved = state.all_resolved()
    reached_source_upper_bound = cleared >= 16
    if known_source_total is not None and not (
        10 <= known_source_total <= 16 and cleared <= known_source_total
    ):
        raise ValueError(
            "known source total must be in [10,16] and not below cleared count"
        )
    if known_source_total is not None:
        source_total = known_source_total
        total_basis = "post-run truth file"
    elif all_resolved:
        source_total = cleared
        total_basis = "inferred after every channel resolved"
    elif reached_source_upper_bound:
        source_total = 16
        total_basis = "problem upper bound reached"
    else:
        source_total = None
        total_basis = "unknown; conservative upper bound is 16"
    ratio = cleared / source_total if source_total else cleared / 16.0
    localization_durations = [
        track.cleared_time_s - track.first_detected_time_s
        for track in state.tracks.values()
        if track.cleared_time_s is not None and track.first_detected_time_s is not None
    ]
    time_breakdown = {
        "movement_s": state.counters.movement_distance_m / ROBOT_SPEED_MPS,
        "measurement_s": state.counters.measure_count * MEASUREMENT_TIME_S,
        "channel_switch_s": state.counters.switch_count * CHANNEL_SWITCH_TIME_S,
        "clear_s": (
            state.counters.clear_success_count * CLEARANCE_SUCCESS_TIME_S
            + state.counters.clear_failure_count * CLEARANCE_FAILURE_TIME_S
        ),
    }
    return {
        "strategy": strategy,
        "all_channels_resolved": all_resolved,
        "cleared_count": cleared,
        "source_total_count": source_total,
        "cleared_over_total": (
            f"{cleared}/{source_total}"
            if source_total is not None
            else f"{cleared}/unknown"
        ),
        "clearance_ratio": ratio,
        "clearance_ratio_denominator": total_basis,
        "total_virtual_time_s": state.virtual_time_s,
        "virtual_time_breakdown_s": time_breakdown,
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
                "status_basis": (
                    "successful clear"
                    if track.status == "cleared"
                    else "1000 m guaranteed coverage exhausted U_j"
                    if track.status == "absent"
                    else "bearing observations and continuous wedge intersection"
                    if track.status in {"detected", "localized"}
                    else "unresolved"
                ),
                "first_detected_time_s": track.first_detected_time_s,
                "localized_time_s": track.localized_time_s,
                "cleared_time_s": track.cleared_time_s,
                "guaranteed_coverage_fraction": state.coverage_fraction(channel),
                "belief": state.beliefs[channel].geometry_metrics(0.95),
            }
            for channel, track in state.tracks.items()
        },
        "coverage_plan": coverage_plan,
        "official_log_original_names": original_log_names or [],
    }


