"""问题 3 的滚动正多边形策略与联合信念MPC策略。

两种策略共享同一状态/执行器：前者强调连续1000 m覆盖证明和低风险在线插入，后者
用有限候选、`tau+E[V_hat]`、beam search改变动作顺序，但始终保留可恢复覆盖路线。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np

from config.constants import (
    CHANNEL_SWITCH_TIME_S,
    CLEARANCE_SUCCESS_TIME_S,
    MEASUREMENT_TIME_S,
    ROBOT_SPEED_MPS,
    SOURCE_COUNT_MAX,
)
from problems.problem3.config import MPC_STRATEGY, ROBUST_STRATEGY, Problem3Settings
from problems.problem3.coverage import (
    PolygonPlan,
    evaluate_polygon_plan,
    held_karp_open_path,
    insertion_extra_length,
    math_distance,
    optimize_remaining_route,
    route_length,
    search_polygon_plans,
)
from problems.problem3.shared import (
    PlannedAction,
    Problem3Executor,
    Problem3State,
    clear_with_fallback_grid,
    choose_localization_point,
    localize_channel,
    localization_candidate_points,
    localize_and_clear_channel,
)


@dataclass(frozen=True, slots=True)
class StrategyRunResult:
    """策略结束时返回状态、覆盖方案和参数搜索表。"""

    state: Problem3State
    selected_plan: PolygonPlan
    compared_plans: tuple[PolygonPlan, ...]


def _ordered_channels(state: Problem3State, channels: list[int]) -> list[int]:
    """若当前频道仍需检测则排在首位，其余按环形频道距离排序。"""

    if state.current_channel in channels:
        channels.remove(state.current_channel)
        return [state.current_channel, *sorted(channels)]
    return sorted(channels, key=lambda channel: abs(channel - state.current_channel))


class RobustPolygonRollingStrategy:
    """正多边形保证覆盖骨架上的发现即定位、任务滚动插入策略。"""

    name = ROBUST_STRATEGY

    def __init__(self, settings: Problem3Settings) -> None:
        self.settings = settings

    def run(self, executor: Problem3Executor) -> StrategyRunResult:
        """完成覆盖扫描、滚动定位清除，并保留随时可恢复的剩余骨架。"""

        state = executor.state
        selected_plan = evaluate_polygon_plan(
            self.settings,
            self.settings.robust_polygon_sides,
            self.settings.robust_polygon_radius_m,
            self.settings.robust_polygon_rotation_deg,
            self.settings.robust_scan_origin,
        )
        if not selected_plan.coverage.valid:
            raise ValueError(
                "robust origin-plus-hexagon plan does not guarantee coverage"
            )
        plans = (selected_plan,)
        remaining = list(selected_plan.points)
        pending_localization: list[int] = []

        while remaining and not state.all_resolved():
            remaining = optimize_remaining_route(state.position, remaining)
            coverage_point = remaining.pop(0)
            channels = [
                channel
                for channel in self.settings.channels
                if state.scan_needed(channel, coverage_point)
            ]
            for channel in _ordered_channels(state, channels):
                if state.tracks[channel].status != "unknown":
                    continue
                information_gain = state.beliefs[channel].information_gain_bits(
                    coverage_point
                )
                result = executor.measure(
                    coverage_point,
                    channel,
                    "保底正多边形覆盖点；本检测严格减少U_j",
                    information_gain_bits=information_gain,
                )
                if result == "direction":
                    next_point, localization_reason = choose_localization_point(
                        state, channel
                    )
                    following = remaining[0] if remaining else coverage_point
                    extra_length = insertion_extra_length(
                        coverage_point, next_point, following
                    )
                    extra_time = (
                        extra_length / ROBOT_SPEED_MPS
                        + MEASUREMENT_TIME_S
                        + (
                            CHANNEL_SWITCH_TIME_S
                            if channel != state.current_channel
                            else 0.0
                        )
                    )
                    if extra_time <= self.settings.insertion_time_limit_s:
                        localize_channel(
                            executor,
                            channel,
                            "滚动立即插入定位任务；"
                            f"DeltaL={extra_length:.2f}m, DeltaT≈{extra_time:.2f}s；"
                            f"{localization_reason}",
                            clear_when_ready=True,
                            clear_radius_m=self.settings.robust_clear_radius_m,
                            use_grid_fallback=True,
                        )
                        if state.tracks[channel].status in {"detected", "localized"}:
                            pending_localization.append(channel)
                    else:
                        pending_localization.append(channel)
                if state.counters.clear_success_count >= SOURCE_COUNT_MAX:
                    break
            if state.counters.clear_success_count >= SOURCE_COUNT_MAX:
                break

        # 高插入代价任务延后：先完成定位但不立即清除，再用Held--Karp统一排序。
        for channel in dict.fromkeys(pending_localization):
            if state.tracks[channel].status in {"detected", "localized"}:
                localize_channel(
                    executor,
                    channel,
                    "保底路径完成后处理延期定位任务",
                    clear_when_ready=False,
                    clear_radius_m=self.settings.robust_clear_radius_m,
                    use_grid_fallback=True,
                )

        clear_tasks: list[tuple[int, tuple[float, float]]] = []
        for channel, track in state.tracks.items():
            if (
                track.status == "localized"
                and track.clear_circle is not None
                and track.clear_circle.radius
                <= self.settings.robust_clear_radius_m + 1e-9
            ):
                clear_tasks.append(
                    (
                        channel,
                        (
                            float(track.clear_circle.center[0]),
                            float(track.clear_circle.center[1]),
                        ),
                    )
                )
        order = held_karp_open_path(
            state.position, [point for _, point in clear_tasks]
        )
        for index in order:
            channel, point = clear_tasks[index]
            if state.tracks[channel].status != "localized":
                continue
            success = executor.clear(
                point,
                channel,
                "Held-Karp末端开放最短路径；"
                f"最小覆盖圆rho={state.tracks[channel].clear_circle.radius:.2f}",
            )
            if not success:
                clear_with_fallback_grid(
                    executor, channel, "保证清除点意外失败后转入28m网格"
                )
        # 理论上每个unknown频道均经历连续覆盖；离散U仍作为实现一致性断言。
        for channel, track in state.tracks.items():
            if track.status == "unknown" and not np.any(state.uncovered[channel]):
                track.status = "absent"
        return StrategyRunResult(state, selected_plan, plans)


def _mst_length(points: list[tuple[float, float]]) -> float:
    """Prim算法计算任务点MST路线长度下界。"""

    if len(points) <= 1:
        return 0.0
    reached = {0}
    remaining = set(range(1, len(points)))
    total = 0.0
    while remaining:
        distance, target = min(
            (math_distance(points[first], points[second]), second)
            for first in reached
            for second in remaining
        )
        total += distance
        reached.add(target)
        remaining.remove(target)
    return total


def guarantee_route_covers_uncovered(
    state: Problem3State,
    remaining_route: list[tuple[float, float]],
) -> bool:
    """验证每个unknown频道的离散U仍可由保留骨架完成，供运行断言和测试。"""

    route = np.asarray(remaining_route, dtype=float)
    for channel, track in state.tracks.items():
        if track.status != "unknown":
            continue
        points = state.coverage_grid[state.uncovered[channel]]
        if not len(points):
            continue
        if not len(route):
            return False
        nearest = np.min(
            np.linalg.norm(points[:, None, :] - route[None, :, :], axis=2), axis=1
        )
        if np.any(nearest > state.settings.guaranteed_radius_m + 1e-9):
            return False
    return True


class BeliefMPCStrategy:
    """以 `Q=tau+E[V_hat]` 和有限时域beam search滚动选择单频道动作。"""

    name = MPC_STRATEGY

    def __init__(self, settings: Problem3Settings) -> None:
        self.settings = settings
        self._noncoverage_actions = 0

    def _remaining_value(
        self,
        state: Problem3State,
        remaining_route: list[tuple[float, float]],
    ) -> float:
        """估计剩余路线、measure、切频、clear和定位时间 `V_hat`。"""

        detected_channels = [
            channel
            for channel, track in state.tracks.items()
            if track.status in {"detected", "localized"}
        ]
        localization_points = [
            state.beliefs[channel].representative_point()
            for channel in detected_channels
        ]
        task_points = [state.position, *remaining_route, *localization_points]
        route_time = _mst_length(task_points) / ROBOT_SPEED_MPS
        coverage_measures = sum(
            state.scan_needed(channel, point)
            for point in remaining_route
            for channel in self.settings.channels
        )
        localization_measures = 0
        for channel in detected_channels:
            belief = state.beliefs[channel]
            representative = belief.representative_point()
            mean_gain = belief.information_gain_bits(representative)
            localization_measures += belief.estimated_localization_measurements(
                self.settings.entropy_clear_threshold_bits,
                mean_gain,
                self.settings.mean_information_gain_floor_bits,
            )
        total_measures = coverage_measures + localization_measures
        switch_estimate = max(total_measures - len(remaining_route), 0)
        clear_time = len(detected_channels) * CLEARANCE_SUCCESS_TIME_S
        return (
            route_time
            + total_measures * MEASUREMENT_TIME_S
            + switch_estimate * CHANNEL_SWITCH_TIME_S
            + clear_time
        )

    def _action_time(self, state: Problem3State, action: PlannedAction) -> float:
        """计算单动作真实结构的移动、5秒检测/清除和切频成本。"""

        movement = math_distance(state.position, action.position) / ROBOT_SPEED_MPS
        if action.kind == "clear":
            return movement + CLEARANCE_SUCCESS_TIME_S
        channel = action.channels[0]
        switch = CHANNEL_SWITCH_TIME_S if channel != state.current_channel else 0.0
        return movement + switch + MEASUREMENT_TIME_S

    @staticmethod
    def _transition_time(
        position: tuple[float, float],
        current_channel: int,
        action: PlannedAction,
    ) -> tuple[float, tuple[float, float], int]:
        """计算beam序列中的路径相关成本；clear不改变接收机频道。"""

        movement = math_distance(position, action.position) / ROBOT_SPEED_MPS
        if action.kind == "clear":
            return movement + CLEARANCE_SUCCESS_TIME_S, action.position, current_channel
        next_channel = action.channels[0]
        switch = CHANNEL_SWITCH_TIME_S if next_channel != current_channel else 0.0
        return movement + switch + MEASUREMENT_TIME_S, action.position, next_channel

    def _generate_candidates(
        self,
        state: Problem3State,
        remaining_route: list[tuple[float, float]],
    ) -> list[PlannedAction]:
        """从覆盖、U代表点、定位点和清除点生成有限单频道候选。"""

        coverage_candidates: list[PlannedAction] = []
        exploratory_candidates: list[PlannedAction] = []
        mandatory_candidates: list[PlannedAction] = []
        for point in remaining_route[: self.settings.coverage_lookahead_nodes]:
            channels = [
                channel
                for channel in self.settings.channels
                if state.scan_needed(channel, point)
            ]
            for channel in _ordered_channels(state, channels):
                gain = state.beliefs[channel].information_gain_bits(point)
                coverage_candidates.append(
                    PlannedAction(
                        "measure",
                        point,
                        (channel,),
                        "保留的1000m保证覆盖路线节点",
                        gain,
                    )
                )

        # 每个unknown频道从U_j中选离当前最近的代表点，作为低插入代价候选。
        for channel in self.settings.channels:
            if state.tracks[channel].status != "unknown":
                continue
            uncovered_points = state.coverage_grid[state.uncovered[channel]]
            if not len(uncovered_points):
                continue
            distances = np.linalg.norm(
                uncovered_points - np.asarray(state.position), axis=1
            )
            representative_array = uncovered_points[int(np.argmin(distances))]
            representative = (
                float(representative_array[0]),
                float(representative_array[1]),
            )
            gain = state.beliefs[channel].information_gain_bits(representative)
            belief = state.beliefs[channel]
            probability = belief.detection_probability(representative)
            normalized_gain = gain / max(belief.entropy_bits, 1e-9)
            if probability >= self.settings.p0 or normalized_gain >= self.settings.g0:
                exploratory_candidates.append(
                    PlannedAction(
                        "measure",
                        representative,
                        (channel,),
                        "unknown频道U_j代表点通过p0或g0预筛："
                        f"p={probability:.3f}, g={normalized_gain:.3f}",
                        gain,
                    )
                )

        for channel, track in state.tracks.items():
            if track.status == "localized" and track.clear_circle is not None:
                point = (
                    float(track.clear_circle.center[0]),
                    float(track.clear_circle.center[1]),
                )
                mandatory_candidates.append(
                    PlannedAction("clear", point, (channel,), "rho<=20的保证清除任务")
                )
            elif track.status == "detected":
                for point, reason in localization_candidate_points(
                    state, channel, self.settings.detected_candidate_count
                ):
                    gain = state.beliefs[channel].information_gain_bits(point)
                    mandatory_candidates.append(
                        PlannedAction("measure", point, (channel,), reason, gain)
                    )

        # information/time只做预排序；最终仍由下面的Q和beam search决定。
        exploratory_candidates.sort(
            key=lambda action: action.information_gain_bits
            / max(self._action_time(state, action), 1e-9),
            reverse=True,
        )
        # 保证清除/定位任务不能被大量unknown候选挤出；下一个保底节点也始终保留。
        protected = mandatory_candidates + coverage_candidates[: len(self.settings.channels)]
        remaining_slots = max(self.settings.candidate_action_limit - len(protected), 0)
        result = protected + exploratory_candidates[:remaining_slots]
        return result[: max(self.settings.candidate_action_limit, len(mandatory_candidates))]

    def _select_action(
        self,
        state: Problem3State,
        remaining_route: list[tuple[float, float]],
        candidates: list[PlannedAction],
    ) -> PlannedAction:
        """以beam search最小化动作时间加信息更新后的预计剩余时间。"""

        if not candidates:
            raise RuntimeError("belief MPC has no feasible action")
        base_value = self._remaining_value(state, remaining_route)
        mean_gain = max(
            np.mean([action.information_gain_bits for action in candidates]),
            self.settings.mean_information_gain_floor_bits,
        )
        # beam元素额外携带序列末端位置/频道，避免把后续移动都错误地从当前点计算。
        beam: list[
            tuple[
                float,
                float,
                float,
                tuple[int, ...],
                tuple[float, float],
                int,
            ]
        ] = [
            (base_value, 0.0, 0.0, (), state.position, state.current_channel)
        ]
        for _ in range(self.settings.horizon):
            expanded: list[
                tuple[
                    float,
                    float,
                    float,
                    tuple[int, ...],
                    tuple[float, float],
                    int,
                ]
            ] = []
            for _, elapsed, saved, sequence, position, current_channel in beam:
                for index, action in enumerate(candidates):
                    if index in sequence:
                        continue
                    tau, next_position, next_channel = self._transition_time(
                        position, current_channel, action
                    )
                    information_saved_time = (
                        action.information_gain_bits / mean_gain * MEASUREMENT_TIME_S
                    )
                    completion_saved = (
                        CLEARANCE_SUCCESS_TIME_S if action.kind == "clear" else 0.0
                    )
                    next_elapsed = elapsed + tau
                    next_saved = saved + information_saved_time + completion_saved
                    expected_remaining = max(base_value - next_saved, 0.0)
                    q_value = next_elapsed + expected_remaining
                    expanded.append(
                        (
                            q_value,
                            next_elapsed,
                            next_saved,
                            (*sequence, index),
                            next_position,
                            next_channel,
                        )
                    )
            if not expanded:
                break
            beam = sorted(expanded, key=lambda item: item[0])[
                : self.settings.beam_width
            ]
        best = min(beam, key=lambda item: item[0])
        first = candidates[best[3][0]]
        tau = self._action_time(state, first)
        expected_remaining = max(
            base_value
            - first.information_gain_bits / mean_gain * MEASUREMENT_TIME_S
            - (CLEARANCE_SUCCESS_TIME_S if first.kind == "clear" else 0.0),
            0.0,
        )
        return PlannedAction(
            first.kind,
            first.position,
            first.channels,
            first.reason,
            first.information_gain_bits,
            tau + expected_remaining,
            expected_remaining,
        )

    def run(self, executor: Problem3Executor) -> StrategyRunResult:
        """滚动执行最优序列首动作，真实观测后立即更新并重新规划。"""

        state = executor.state
        selected_plan, plans = search_polygon_plans(self.settings)
        remaining_route = list(selected_plan.points)
        maximum_iterations = 5_000
        for _ in range(maximum_iterations):
            # 只有当一个覆盖点对任何unknown频道都不再减少U时，才可从保底路线移除。
            remaining_route = [
                point
                for point in remaining_route
                if any(
                    state.scan_needed(channel, point)
                    for channel in self.settings.channels
                )
            ]
            if state.all_resolved() or state.counters.clear_success_count >= SOURCE_COUNT_MAX:
                break
            candidates = self._generate_candidates(state, remaining_route)
            if not candidates:
                break
            action = self._select_action(state, remaining_route, candidates)
            is_coverage = action.position in remaining_route
            if (
                self._noncoverage_actions >= self.settings.maximum_noncoverage_actions
                and remaining_route
                and not is_coverage
            ):
                point = optimize_remaining_route(state.position, remaining_route)[0]
                channel = _ordered_channels(
                    state,
                    [
                        item
                        for item in self.settings.channels
                        if state.scan_needed(item, point)
                    ],
                )[0]
                gain = state.beliefs[channel].information_gain_bits(point)
                action = PlannedAction(
                    "measure",
                    point,
                    (channel,),
                    "安全约束：恢复可完成的1000m保证覆盖路线",
                    gain,
                    action.q_value_s,
                    action.estimated_remaining_time_s,
                )
                is_coverage = True
            if action.kind == "clear":
                executor.clear(
                    action.position,
                    action.channels[0],
                    action.reason,
                    action.q_value_s,
                    action.estimated_remaining_time_s,
                )
            else:
                executor.measure(
                    action.position,
                    action.channels[0],
                    action.reason,
                    action.information_gain_bits,
                    action.q_value_s,
                    action.estimated_remaining_time_s,
                )
            self._noncoverage_actions = 0 if is_coverage else self._noncoverage_actions + 1
            remaining_route = optimize_remaining_route(state.position, remaining_route)
            if not guarantee_route_covers_uncovered(state, remaining_route):
                raise RuntimeError("belief MPC lost its recoverable guaranteed route")
        else:
            raise RuntimeError("belief MPC exceeded maximum planning iterations")

        for channel, track in state.tracks.items():
            if track.status == "unknown" and not np.any(state.uncovered[channel]):
                track.status = "absent"
        return StrategyRunResult(state, selected_plan, plans)
