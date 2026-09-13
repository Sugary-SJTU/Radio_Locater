"""问题 3 的滚动正多边形策略与联合信念MPC策略。

两种策略共享同一状态/执行器：前者强调连续1000 m覆盖证明和低风险在线插入，后者
用有限候选、`tau+E[V_hat]`、beam search改变动作顺序，但始终保留可恢复覆盖路线。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Any

import numpy as np

from config.constants import (
    CHANNEL_SWITCH_TIME_S,
    CLEARANCE_FAILURE_TIME_S,
    CLEARANCE_SUCCESS_TIME_S,
    MEASUREMENT_TIME_S,
    ROBOT_SPEED_MPS,
    SOURCE_COUNT_MAX,
)
from problems.problem3.config import MPC_STRATEGY, ROBUST_STRATEGY, Problem3Settings
from problems.problem3.coverage import (
    PolygonPlan,
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
    choose_localization_point,
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
        selected_plan, plans = search_polygon_plans(self.settings)
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
                        localize_and_clear_channel(
                            executor,
                            channel,
                            "滚动立即插入定位任务；"
                            f"DeltaL={extra_length:.2f}m, DeltaT≈{extra_time:.2f}s；"
                            f"{localization_reason}",
                        )
                        if state.tracks[channel].status in {"detected", "localized"}:
                            pending_localization.append(channel)
                    else:
                        pending_localization.append(channel)
                if state.counters.clear_success_count >= SOURCE_COUNT_MAX:
                    break
            if state.counters.clear_success_count >= SOURCE_COUNT_MAX:
                break

        # 高插入代价任务延后，但只有保底覆盖完成后才处理；detected频道早已从扫描移出。
        for channel in dict.fromkeys(pending_localization):
            if state.tracks[channel].status in {"detected", "localized"}:
                localize_and_clear_channel(
                    executor, channel, "保底路径完成后处理延期定位任务"
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
        self._last_scored_candidates: list[PlannedAction] = []

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

    # 以下方法是完整网格信念MPC实现。保留上方基础版便于论文消融说明；Python采用
    # 后定义的方法，因此正式 belief_mpc 始终执行下面的条件观测与时间边界模型。
    @staticmethod
    def _switches(current_channel: int, channels: tuple[int, ...]) -> int:
        """计算给定批次顺序的真实换频次数。"""

        switches = 0
        current = current_channel
        for channel in channels:
            switches += int(channel != current)
            current = channel
        return switches

    def _action_time(self, state: Problem3State, action: PlannedAction) -> float:
        movement = math_distance(state.position, action.position) / ROBOT_SPEED_MPS
        if action.kind == "clear":
            failure = self.settings.expected_clear_failure_probability
            operation = (1.0 - failure) * CLEARANCE_SUCCESS_TIME_S + failure * CLEARANCE_FAILURE_TIME_S
            return movement + operation
        return (
            movement
            + len(action.channels) * MEASUREMENT_TIME_S
            + self._switches(state.current_channel, action.channels) * CHANNEL_SWITCH_TIME_S
        )

    @staticmethod
    def _nearest_neighbor_upper(
        start: tuple[float, float], points: list[tuple[float, float]]
    ) -> float:
        """最近邻后用2-opt得到可执行路线长度上界。"""

        remaining = list(dict.fromkeys(points))
        route = [start]
        while remaining:
            index = min(range(len(remaining)), key=lambda i: math_distance(route[-1], remaining[i]))
            route.append(remaining.pop(index))
        improved = True
        while improved and len(route) > 3:
            improved = False
            for first in range(1, len(route) - 2):
                for second in range(first + 1, len(route) - 1):
                    old = math_distance(route[first-1], route[first]) + math_distance(route[second], route[second+1])
                    new = math_distance(route[first-1], route[second]) + math_distance(route[first], route[second+1])
                    if new + 1e-9 < old:
                        route[first:second+1] = reversed(route[first:second+1]); improved = True
        return sum(math_distance(a, b) for a, b in zip(route, route[1:]))

    def _remaining_bounds(
        self, state: Problem3State, remaining_route: list[tuple[float, float]]
    ) -> tuple[float, float, dict[str, float]]:
        """返回MST/最少动作下界和安全路线+NN/2-opt可执行上界。"""

        unresolved = state.unresolved_channels()
        detected = [c for c in unresolved if state.tracks[c].status in {"detected", "localized"}]
        unknown = [c for c in unresolved if state.tracks[c].status == "unknown"]
        location_points = [state.beliefs[c].representative_point() for c in detected]
        task_points = [state.position, *remaining_route, *location_points]
        route_lower = _mst_length(task_points)
        route_upper = self._nearest_neighbor_upper(state.position, [*remaining_route, *location_points])
        minimum_measures = len(unknown) + sum(state.tracks[c].status == "detected" for c in detected)
        coverage_measures = sum(
            state.scan_needed(channel, point)
            for point in remaining_route for channel in self.settings.channels
        )
        localization_measures = 0
        for channel in detected:
            belief = state.beliefs[channel]
            gain = max(belief.information_gain_bits(belief.representative_point()), self.settings.mean_information_gain_floor_bits)
            localization_measures += belief.estimated_localization_measurements(
                self.settings.entropy_clear_threshold_bits, gain,
                self.settings.mean_information_gain_floor_bits,
            )
        minimum_clears = max(0, 10 - state.counters.clear_success_count)
        expected_clears = sum(state.beliefs[c].existence_probability for c in unresolved)
        unavoidable_switches = max(len(set(unresolved)) - 1, 0) if unresolved else 0
        upper_switches = max(coverage_measures + localization_measures - len(remaining_route), 0)
        lower = (
            route_lower / ROBOT_SPEED_MPS + minimum_measures * MEASUREMENT_TIME_S
            + minimum_clears * CLEARANCE_SUCCESS_TIME_S
            + unavoidable_switches * CHANNEL_SWITCH_TIME_S
        )
        upper = (
            route_upper / ROBOT_SPEED_MPS
            + (coverage_measures + localization_measures) * MEASUREMENT_TIME_S
            + upper_switches * CHANNEL_SWITCH_TIME_S
            + expected_clears * CLEARANCE_SUCCESS_TIME_S
            + expected_clears * self.settings.expected_clear_failure_probability * CLEARANCE_FAILURE_TIME_S
        )
        return lower, max(upper, lower), {
            "route_lower_m": route_lower, "route_upper_m": route_upper,
            "minimum_measure_count": float(minimum_measures),
            "upper_measure_count": float(coverage_measures + localization_measures),
            "minimum_clear_count": float(minimum_clears),
        }

    @staticmethod
    def _prune_by_bounds(
        candidates: list[PlannedAction], best_feasible_upper_s: float
    ) -> list[PlannedAction]:
        """删除下界已超过当前可行上界的动作，但始终保留清除动作。"""

        return [
            action for action in candidates
            if action.kind == "clear"
            or "安全" in action.reason
            or action.lower_bound_s is None
            or action.lower_bound_s <= best_feasible_upper_s
        ]

    def _fallback_required(
        self, state: Problem3State, candidates: list[PlannedAction], has_safe_route: bool = True
    ) -> bool:
        """集中定义安全回退触发器，便于单测和人工审查。"""

        return (
            not candidates
            or (
                has_safe_route
                and state.no_progress_steps >= self.settings.fallback_no_progress_steps
            )
            or state.remaining_real_duration_s <= 0
            or (
                has_safe_route
                and state.max_virtual_duration_s - state.virtual_time_s
                <= self.settings.minimum_safe_remaining_time_s
            )
            or any(not np.isfinite(belief.weights).all() for belief in state.beliefs.values())
        )

    def _coverage_gain(self, state: Problem3State, position: tuple[float,float], channels: tuple[int,...]) -> float:
        """批次在各频道U_j上新覆盖的网格比例之和。"""

        distances=np.linalg.norm(state.coverage_grid-np.asarray(position,dtype=float),axis=1)
        covered=distances<=self.settings.guaranteed_radius_m+1e-9
        return float(sum(np.mean(state.uncovered[c] & covered) for c in channels if state.tracks[c].status=="unknown"))

    def _generate_candidates(self, state: Problem3State, remaining_route: list[tuple[float,float]]) -> list[PlannedAction]:
        """分层生成安全、U中心、后验中心/MAP、问题2定位、MEC及批量候选。"""

        raw: list[PlannedAction] = []
        safe: list[PlannedAction] = []
        for point in remaining_route[:self.settings.coverage_lookahead_nodes]:
            channels=_ordered_channels(state,[c for c in self.settings.channels if state.scan_needed(c,point)])
            for channel in channels:
                item=PlannedAction("measure",point,(channel,),"安全覆盖骨架后续节点",state.beliefs[channel].information_gain_bits(point))
                safe.append(item); raw.append(item)
            if len(channels)>1:
                batch=tuple(channels[:min(4,len(channels))])
                raw.append(PlannedAction("batch_measure",point,batch,"同一安全节点的高价值频道批量检测",sum(state.beliefs[c].information_gain_bits(point) for c in batch)))
        for channel in self.settings.channels:
            track=state.tracks[channel]; belief=state.beliefs[channel]
            if track.status=="unknown":
                uncovered=state.coverage_grid[state.uncovered[channel]]
                if len(uncovered):
                    center=np.mean(uncovered,axis=0); candidates=[tuple(center),belief.representative_point(),belief.map_point()]
                    nearest=uncovered[int(np.argmin(np.linalg.norm(uncovered-np.asarray(state.position),axis=1)))]; candidates.append(tuple(nearest))
                    for point in candidates:
                        point=(float(point[0]),float(point[1])); probability=belief.detection_probability(point); gain=belief.information_gain_bits(point)
                        if probability>=self.settings.p0 or gain/max(belief.entropy_bits,1e-9)>=self.settings.g0 or state.scan_needed(channel,point):
                            raw.append(PlannedAction("measure",point,(channel,),f"U中心/高熵中心/MAP候选；p={probability:.3f}",gain))
            elif track.status=="localized" and track.clear_circle is not None:
                point=(float(track.clear_circle.center[0]),float(track.clear_circle.center[1]))
                raw.append(PlannedAction("clear",point,(channel,),f"连续几何最小覆盖圆rho={track.clear_circle.radius:.3f}<=20"))
            elif track.status=="detected":
                for point,reason in localization_candidate_points(state,channel,self.settings.detected_candidate_count):
                    raw.append(PlannedAction("measure",point,(channel,),reason,belief.information_gain_bits(point)))
                metrics=belief.geometry_metrics(self.settings.clear_probability)
                center=tuple(float(value) for value in metrics["minimum_enclosing_center"])
                probability=belief.clear_probability_at(center,self.settings.clearance_radius_m)
                new_evidence_after_failure = (
                    track.last_clear_failure_time_s is None
                    or (
                        track.last_measurement_time_s is not None
                        and track.last_measurement_time_s > track.last_clear_failure_time_s
                    )
                )
                fine_enough = metrics.get("grid_resolution_m", math.inf) <= self.settings.clearance_grid_size
                if probability>=self.settings.clear_probability and fine_enough and new_evidence_after_failure:
                    raw.append(PlannedAction("clear",center,(channel,),f"概率清除P={probability:.5f}，失败后恢复detected"))
        # 坐标、动作类型和频道完全相同的候选去重。
        unique: dict[tuple[Any,...],PlannedAction]={}
        for action in raw:
            key=(action.kind,round(action.position[0],6),round(action.position[1],6),action.channels)
            unique.setdefault(key,action)
        decorated=[]
        for action in unique.values():
            tau=self._action_time(state,action); coverage=self._coverage_gain(state,action.position,action.channels)
            localization=sum(state.tracks[c].status in {"detected","localized"} for c in action.channels)
            efficiency=(action.information_gain_bits+self.settings.efficiency_coverage_weight*coverage+self.settings.efficiency_localization_weight*localization)/max(tau,1e-9)
            decorated.append(replace(action,immediate_time_s=tau,coverage_gain=coverage,localization_gain=float(localization),efficiency=efficiency))
        protected_keys={(a.kind,a.position,a.channels) for a in safe[:len(self.settings.channels)]}
        protected=[a for a in decorated if (a.kind,a.position,a.channels) in protected_keys or a.kind=="clear"]
        others=sorted((a for a in decorated if a not in protected),key=lambda a:a.efficiency,reverse=True)
        limit=self.settings.candidate_top_k
        return (protected+others[:max(limit-len(protected),0)])[:max(limit,len(protected))]

    def _expected_entropy_after(self, belief: Any, action: PlannedAction) -> float:
        """显式虚拟贝叶斯更新各观测分支，返回条件后验熵期望。"""

        total=0.0
        for scenario in belief.observation_scenarios(action.position,self.settings.observation_scenario_limit,self.settings.observation_merge_probability):
            virtual=belief.copy(); virtual.update(action.position,scenario.result,scenario.bearing_deg)
            total += scenario.probability*virtual.entropy_bits
        return total

    def _select_action(self, state: Problem3State, remaining_route: list[tuple[float,float]], candidates: list[PlannedAction]) -> PlannedAction:
        """用 ``tau+E[V_hat]``、场景虚拟更新、有限时域和上下界剪枝选首动作。"""

        if not candidates: raise RuntimeError("belief MPC has no feasible action")
        lower,upper,_=self._remaining_bounds(state,remaining_route)
        mean_gain=max(np.mean([a.information_gain_bits for a in candidates if a.kind!="clear"] or [self.settings.mean_information_gain_floor_bits]),self.settings.mean_information_gain_floor_bits)
        horizon=self.settings.planning_horizon
        variant=self.settings.ablation_variant.upper()
        expected_reductions: dict[tuple[int, float, float], float] = {}
        for item in candidates:
            if item.kind == "clear": continue
            for channel in item.channels:
                key=(channel,round(item.position[0],6),round(item.position[1],6))
                if key not in expected_reductions:
                    belief=state.beliefs[channel]
                    expected_reductions[key]=max(belief.entropy_bits-self._expected_entropy_after(belief,item),0.0)
        scored=[]
        for action in candidates:
            tau=action.immediate_time_s if action.immediate_time_s is not None else self._action_time(state,action)
            entropy_reduction=0.0
            if action.kind!="clear":
                entropy_reduction=sum(expected_reductions[(c,round(action.position[0],6),round(action.position[1],6))] for c in action.channels)
            saved=entropy_reduction/max(mean_gain,1e-9)*MEASUREMENT_TIME_S
            if action.kind=="clear": saved+=CLEARANCE_SUCCESS_TIME_S
            remaining=max(upper-saved,lower)
            if variant=="A": q=tau/max(action.information_gain_bits+action.coverage_gain+action.localization_gain,1e-9)
            else:
                depth=horizon if variant in {"C","D","E"} else 1
                initial_channel=state.current_channel if action.kind=="clear" else action.channels[-1]
                # beam节点=(累计后续耗时,位置,频道,剩余估计,已用候选下标)。
                beam=[(0.0,action.position,initial_channel,remaining,frozenset())]
                for _ in range(1,depth):
                    expanded=[]
                    for elapsed,position,channel,future,used in beam:
                        for index,item in enumerate(candidates):
                            if item is action or index in used: continue
                            move=math_distance(position,item.position)/ROBOT_SPEED_MPS
                            if item.kind=="clear":
                                step=move+CLEARANCE_SUCCESS_TIME_S; saved_next=CLEARANCE_SUCCESS_TIME_S
                                next_channel=channel
                            else:
                                step=move+len(item.channels)*MEASUREMENT_TIME_S+self._switches(channel,item.channels)*CHANNEL_SWITCH_TIME_S
                                saved_next=sum(expected_reductions[(c,round(item.position[0],6),round(item.position[1],6))] for c in item.channels)/max(mean_gain,1e-9)*MEASUREMENT_TIME_S
                                next_channel=item.channels[-1]
                            next_future=max(future-saved_next,lower)
                            expanded.append((elapsed+step,item.position,next_channel,next_future,used|{index}))
                    if not expanded: break
                    beam=sorted(expanded,key=lambda node:node[0]+node[3])[:self.settings.beam_width]
                q=tau+min(node[0]+node[3] for node in beam)
            action_lower=tau+lower
            scored.append(replace(action,q_value_s=q,estimated_remaining_time_s=remaining,lower_bound_s=action_lower,upper_bound_s=tau+upper))
        unpruned = scored
        scored = self._prune_by_bounds(scored, upper)
        # 上界近似过紧时不得让概率规划中断；恢复剪枝前的最低下界动作。
        if not scored:
            scored = [min(unpruned, key=lambda action: action.lower_bound_s or math.inf)]
        self._last_scored_candidates = scored
        best=min(scored,key=lambda action:action.q_value_s if action.q_value_s is not None else math.inf)
        # 完整模型只在可恢复且预计节省超过迟滞阈值时偏离安全骨架。
        if variant=="E" and remaining_route:
            safe=[a for a in scored if a.position==remaining_route[0] and a.kind in {"measure","batch_measure"}]
            if safe:
                safe_best=min(safe,key=lambda a:a.q_value_s if a.q_value_s is not None else math.inf)
                recoverable=guarantee_route_covers_uncovered(state,remaining_route)
                saving=(safe_best.q_value_s or math.inf)-(best.q_value_s or math.inf)
                if not recoverable or (best.position!=remaining_route[0] and saving<=self.settings.replan_time_margin): best=safe_best
        return best

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
            if state.should_terminate():
                break
            candidates = self._generate_candidates(state, remaining_route)
            force_fallback = self._fallback_required(state, candidates, bool(remaining_route))
            if force_fallback:
                if not remaining_route:
                    raise RuntimeError("安全回退触发，但已无可恢复的覆盖节点")
                point = optimize_remaining_route(state.position, remaining_route)[0]
                needed = _ordered_channels(state, [c for c in self.settings.channels if state.scan_needed(c, point)])
                if not needed:
                    remaining_route.remove(point)
                    continue
                channel = needed[0]
                gain = state.beliefs[channel].information_gain_bits(point)
                action = PlannedAction(
                    "measure", point, (channel,),
                    "安全回退：候选为空/数值异常/连续无有效观测/实时时间不足",
                    gain, immediate_time_s=math_distance(state.position, point)/ROBOT_SPEED_MPS
                    + MEASUREMENT_TIME_S + int(channel != state.current_channel),
                    coverage_gain=self._coverage_gain(state, point, (channel,)),
                )
                candidates = [action]
            else:
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
            selection_reason = (
                f"最小化tau+E[V_hat]；tau={action.immediate_time_s}, "
                f"IG={action.information_gain_bits:.6f}, coverage={action.coverage_gain:.6f}, "
                f"V_hat={action.estimated_remaining_time_s}, Q={action.q_value_s};"
                f"消融版本={self.settings.ablation_variant}"
            )
            logged_candidates = self._last_scored_candidates if self._last_scored_candidates else candidates
            executor.log_planning(logged_candidates, action, selection_reason)
            if action.kind == "clear":
                executor.clear(
                    action.position,
                    action.channels[0],
                    action.reason,
                    action.q_value_s,
                    action.estimated_remaining_time_s,
                )
            elif action.kind == "measure":
                executor.measure(
                    action.position,
                    action.channels[0],
                    action.reason,
                    action.information_gain_bits,
                    action.q_value_s,
                    action.estimated_remaining_time_s,
                )
            else:
                # 附件没有batch端点：批量动作严格展开为同一点的多个真实/measure。
                for channel in action.channels:
                    if state.tracks[channel].status in {"cleared", "absent"}:
                        continue
                    executor.measure(
                        action.position,
                        channel,
                        f"{action.reason}；批量动作展开为附件原生/measure",
                        state.beliefs[channel].information_gain_bits(action.position),
                        action.q_value_s,
                        action.estimated_remaining_time_s,
                    )
                    if state.counters.clear_success_count >= SOURCE_COUNT_MAX:
                        break
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
