"""问题 3 的滚动正多边形策略与联合信念MPC策略。

两种策略共享同一状态/执行器：前者强调连续1000 m覆盖证明和低风险在线插入，后者
用有限候选、`tau+E[V_hat]`、beam search改变动作顺序，但始终保留可恢复覆盖路线。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from config.constants import (
    CHANNEL_SWITCH_TIME_S,
    CLEARANCE_SUCCESS_TIME_S,
    MEASUREMENT_TIME_S,
    RECEPTION_RADIUS_MAX_M,
    ROBOT_SPEED_MPS,
    SOURCE_COUNT_MAX,
)
from problems.problem3.config import (
    DISTANCE_TOUR_STRATEGY,
    DYNAMIC_COVERAGE_TOUR_STRATEGY,
    MPC_STRATEGY,
    ROBUST_STRATEGY,
    SAFE_CLEAR_TOUR_STRATEGY,
    TOUR_STRATEGY,
    Problem3Settings,
)
from problems.problem3.coverage import (
    PolygonPlan,
    evaluate_polygon_plan,
    held_karp_open_path,
    math_distance,
    optimize_remaining_route,
    optimize_remaining_route_multistart,
    search_polygon_plans,
)
from problems.problem3.shared import (
    PlannedAction,
    Problem3Executor,
    Problem3State,
    choose_localization_point,
    clear_with_fallback_grid,
    localization_candidate_points,
    localize_channel,
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
                    pending_localization.append(channel)
                if state.counters.clear_success_count >= SOURCE_COUNT_MAX:
                    break

            # 同一覆盖点先扫完全部频道，避免每发现一源就往返一次。
            # 旧队列也在每个覆盖点重新评估；按当前位置而非发现顺序执行。
            while pending_localization:
                remaining = optimize_remaining_route(state.position, remaining)
                following = remaining[0] if remaining else None
                options = []
                for channel in dict.fromkeys(pending_localization):
                    track = state.tracks[channel]
                    if track.status not in {"detected", "localized"}:
                        continue
                    point, reason = choose_localization_point(state, channel)
                    # 圆心仅用于估算任务末端，实际清除仍要求19.8 m证明。
                    center = tuple(float(x) for x in track.clear_circle.center)
                    length = math_distance(state.position, point) + math_distance(
                        point, center
                    )
                    if following is not None:
                        length += math_distance(center, following) - math_distance(
                            state.position, following
                        )
                    cost = (
                        length / ROBOT_SPEED_MPS
                        + 2 * MEASUREMENT_TIME_S
                        + CLEARANCE_SUCCESS_TIME_S
                    )
                    if channel != state.current_channel:
                        cost += CHANNEL_SWITCH_TIME_S
                    options.append((cost, channel, reason))
                if not options:
                    pending_localization.clear()
                    break
                cost, channel, reason = min(options)
                if (
                    following is not None
                    and cost > self.settings.insertion_time_limit_s
                ):
                    break
                localize_channel(
                    executor,
                    channel,
                    f"覆盖点整批扫描后滚动插入；含估计清除末端DeltaT≈{cost:.2f}s；{reason}",
                    clear_when_ready=True,
                    clear_radius_m=self.settings.robust_clear_radius_m,
                    use_grid_fallback=True,
                )
                pending_localization = [
                    item for item in pending_localization if item != channel
                ]
            if state.counters.clear_success_count >= SOURCE_COUNT_MAX:
                break

        # 覆盖完成后，按到下一定位点的距离滚动处理，定位后就地清除，避免二次巡回。
        pending_localization = [
            channel
            for channel, track in state.tracks.items()
            if track.status in {"detected", "localized"}
        ]
        while pending_localization:
            channel = min(
                pending_localization,
                key=lambda item: math_distance(
                    state.position, choose_localization_point(state, item)[0]
                ),
            )
            localize_channel(
                executor,
                channel,
                "覆盖完成后按当前位置滚动定位并清除",
                clear_when_ready=True,
                clear_radius_m=self.settings.robust_clear_radius_m,
                use_grid_fallback=True,
            )
            pending_localization.remove(channel)

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
        order = held_karp_open_path(state.position, [point for _, point in clear_tasks])
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
            key=lambda action: (
                action.information_gain_bits
                / max(self._action_time(state, action), 1e-9)
            ),
            reverse=True,
        )
        # 保证清除/定位任务不能被大量unknown候选挤出；下一个保底节点也始终保留。
        protected = (
            mandatory_candidates + coverage_candidates[: len(self.settings.channels)]
        )
        remaining_slots = max(self.settings.candidate_action_limit - len(protected), 0)
        result = protected + exploratory_candidates[:remaining_slots]
        return result[
            : max(self.settings.candidate_action_limit, len(mandatory_candidates))
        ]

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
        ] = [(base_value, 0.0, 0.0, (), state.position, state.current_channel)]
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
            if (
                state.all_resolved()
                or state.counters.clear_success_count >= SOURCE_COUNT_MAX
            ):
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
            self._noncoverage_actions = (
                0 if is_coverage else self._noncoverage_actions + 1
            )
            remaining_route = optimize_remaining_route(state.position, remaining_route)
            if not guarantee_route_covers_uncovered(state, remaining_route):
                raise RuntimeError("belief MPC lost its recoverable guaranteed route")
        else:
            raise RuntimeError("belief MPC exceeded maximum planning iterations")

        for channel, track in state.tracks.items():
            if track.status == "unknown" and not np.any(state.uncovered[channel]):
                track.status = "absent"
        return StrategyRunResult(state, selected_plan, plans)


class IntegratedBearingTourStrategy:
    """让覆盖点与交会目标共用测站并参加动态开放旅行商规划。"""

    name = TOUR_STRATEGY

    def __init__(self, settings: Problem3Settings) -> None:
        self.settings = settings
        self._scan_index = 0

    def _channel_order(self, state: Problem3State, channels: list[int]) -> list[int]:
        if not channels:
            return []
        if self.settings.tour_channel_order == "legacy":
            return _ordered_channels(state, channels)
        reverse = self._scan_index % 2 == 1
        self._scan_index += 1
        return sorted(channels, reverse=reverse)

    @staticmethod
    def _useful_bearing(
        state: Problem3State, channel: int, point: tuple[float, float]
    ) -> bool:
        track = state.tracks[channel]
        if track.status != "detected" or track.clear_circle is None:
            return False
        if any(math_distance(point, m.station) < 50.0 for m in track.measurements):
            return False
        # 保守筛掉一定超出最大接收半径的测站，不把接收概率当作不存在证明。
        return (
            math_distance(point, tuple(track.clear_circle.center))
            <= RECEPTION_RADIUS_MAX_M + track.clear_circle.radius
        )

    def _scan(
        self,
        executor: Problem3Executor,
        point: tuple[float, float],
        reason: str = "共享测站：全域覆盖与多频道交会复测共用移动",
    ) -> None:
        state = executor.state
        channels = [
            c
            for c in self.settings.channels
            if state.scan_needed(c, point) or self._useful_bearing(state, c, point)
        ]
        for channel in self._channel_order(state, channels):
            executor.measure(point, channel, reason)

    def _target_candidates(
        self, state: Problem3State, remaining: list[tuple[float, float]]
    ):
        """末段把粗定位源纳入规划；未知频道从不使用未来坐标入队。"""
        allow_uncertain = (
            self.settings.tour_endgame_mode == "probe" and len(remaining) <= 2
        )
        return [
            (c, tuple(float(x) for x in track.clear_circle.center))
            for c, track in state.tracks.items()
            if track.status in {"detected", "localized"}
            and track.clear_circle is not None
            and (
                not remaining
                or allow_uncertain
                or track.clear_circle.radius <= self.settings.tour_target_radius_m
            )
        ]

    def _order_points(self, position, points):
        """十个以内候选用精确开放TSP，其余保留2-opt避免指数计算膨胀。"""
        if self.settings.tour_endgame_mode != "legacy" and len(points) <= 10:
            return [points[i] for i in held_karp_open_path(position, points)]
        return optimize_remaining_route(position, points)

    @staticmethod
    def _probe_point(track, position, center, following):
        """侧向150m的共享交会候选；仅估计绕行成本，不作为安全清除点。"""
        angle = math.radians(track.measurements[-1].bearing_deg)
        perpendicular = np.array([-math.sin(angle), math.cos(angle)])
        candidates = [
            tuple(np.asarray(center) + sign * 150.0 * perpendicular) for sign in (-1, 1)
        ]
        return min(
            candidates,
            key=lambda p: math_distance(position, p) + math_distance(p, following),
        )

    def _initial_plan(
        self, executor: Problem3Executor
    ) -> tuple[PolygonPlan, list[tuple[float, float]]]:
        """构造初始覆盖骨架；子类可在不改主循环的前提下自适应选形。"""

        state = executor.state
        plan = evaluate_polygon_plan(
            self.settings,
            self.settings.tour_polygon_sides,
            self.settings.tour_polygon_radius_m,
            0.0,
            self.settings.tour_scan_origin,
        )
        if not plan.coverage.valid:
            raise ValueError("integrated bearing tour requires guaranteed coverage")
        if not self.settings.tour_scan_origin:
            self._scan(executor, state.position)
        return plan, list(plan.points)

    def _localize_target(
        self,
        executor: Problem3Executor,
        channel: int,
        following: tuple[float, float] | None,
    ) -> None:
        """执行目标定位；保留 following 钩子供路线感知变体使用。"""

        del following
        localize_channel(
            executor,
            channel,
            "覆盖与目标联合开放旅行商路线",
            clear_when_ready=True,
            clear_radius_m=self.settings.robust_clear_radius_m,
            use_grid_fallback=True,
        )

    def _refresh_remaining(
        self,
        executor: Problem3Executor,
        remaining: list[tuple[float, float]],
    ) -> list[tuple[float, float]]:
        """覆盖扫描后的骨架更新钩子；固定策略保持原节点集合。"""

        del executor
        return remaining

    def run(self, executor: Problem3Executor) -> StrategyRunResult:
        state = executor.state
        plan, remaining = self._initial_plan(executor)
        probed: set[int] = set()
        for _ in range(100):
            if state.all_resolved():
                return StrategyRunResult(state, plan, (plan,))
            targets = self._target_candidates(state, remaining)
            points = [*remaining, *(p for _, p in targets)]
            if not points:
                raise RuntimeError("integrated bearing tour has no remaining task")
            route = self._order_points(state.position, points)
            point = route[0]
            if point in remaining:
                remaining.remove(point)
                self._scan(executor, point)
                remaining = self._refresh_remaining(executor, remaining)
            else:
                channel = next(c for c, p in targets if p == point)
                track = state.tracks[channel]
                if (
                    self.settings.tour_endgame_mode == "probe"
                    and len(remaining) <= 2
                    and track.clear_circle.radius > self.settings.tour_target_radius_m
                    and channel not in probed
                ):
                    # 粗定位中心不直接作为清除点。在示向侧向偏移处共享补测，建立交会基线。
                    following = route[1] if len(route) > 1 else point
                    probe = self._probe_point(track, state.position, point, following)
                    self._scan(
                        executor,
                        probe,
                        f"末段频道{channel}粗定位侧向150m补测；兼顾未知频道覆盖与共享交会",
                    )
                    remaining = self._refresh_remaining(executor, remaining)
                    probed.add(channel)
                    continue
                following = route[1] if len(route) > 1 else None
                self._localize_target(executor, channel, following)
                if state.tracks[channel].status != "cleared":
                    raise RuntimeError("integrated tour could not clear target")
                # 只复测已发现频道；未知频道由保留骨架完成，避免每清一源扫描20频道。
                for other in self._channel_order(
                    state,
                    [
                        c
                        for c in self.settings.channels
                        if self._useful_bearing(state, c, state.position)
                    ],
                ):
                    executor.measure(
                        state.position, other, "清除点共享已发现频道交会测向"
                    )
        raise RuntimeError("integrated bearing tour iteration limit")


class DistanceOptimizedBearingTourStrategy(IntegratedBearingTourStrategy):
    """保持联合巡回动作规则不变，以更强局部搜索缩短中段开放路线。"""

    name = DISTANCE_TOUR_STRATEGY

    def _order_points(self, position, points):
        if self.settings.tour_endgame_mode != "legacy" and len(points) <= 10:
            return [points[i] for i in held_karp_open_path(position, points)]
        return optimize_remaining_route_multistart(position, points)


class SafeClearRouteAlignedTourStrategy(DistanceOptimizedBearingTourStrategy):
    """在线旋转覆盖骨架，并在安全余量内把清除点移向开放路线。"""

    name = SAFE_CLEAR_TOUR_STRATEGY
    _ROTATION_CANDIDATE_COUNT = 12

    @staticmethod
    def _expected_intersection_quality(
        state: Problem3State,
        channel: int,
        point: tuple[float, float],
    ) -> float:
        """估计候选点能收到信号并形成大交会角的联合质量。"""

        track = state.tracks[channel]
        belief = state.beliefs[channel]
        first_angle = math.radians(track.measurements[0].bearing_deg)
        first_direction = np.array([math.cos(first_angle), math.sin(first_angle)])
        vectors = belief.points - np.asarray(point, dtype=float)
        distances = np.linalg.norm(vectors, axis=1)
        received = belief.exists & (distances <= belief.radii_m + 1e-9)
        usable = received & (distances > 1e-9)
        sine = np.zeros(len(vectors), dtype=float)
        sine[usable] = (
            np.abs(
                first_direction[0] * vectors[usable, 1]
                - first_direction[1] * vectors[usable, 0]
            )
            / distances[usable]
        )
        # sin²抑制近共线交会；权重同时包含后验位置与可接收概率。
        return float(np.sum(belief.weights * usable * sine**2))

    def _initial_plan(
        self, executor: Problem3Executor
    ) -> tuple[PolygonPlan, list[tuple[float, float]]]:
        state = executor.state
        # 起点本来就在覆盖方案中；先执行它，才能仅使用在线观测自适应选角。
        self._scan(
            executor, state.position, "起点覆盖扫描；为覆盖骨架旋转提供在线粗测向"
        )
        detected_channels = [
            channel
            for channel, track in state.tracks.items()
            if track.status == "detected" and track.measurements
        ]
        candidates: list[PolygonPlan] = []
        quality_scores: list[float] = []
        sides = self.settings.route_aligned_polygon_sides
        radius_m = self.settings.route_aligned_polygon_radius_m
        rotation_period = 360.0 / sides
        rotations = np.linspace(
            0.0,
            rotation_period,
            self._ROTATION_CANDIDATE_COUNT,
            endpoint=False,
        )
        for rotation in rotations:
            plan = evaluate_polygon_plan(
                self.settings,
                sides,
                radius_m,
                float(rotation),
                True,
            )
            if not plan.coverage.valid:
                continue
            vertices = [
                point
                for point in plan.points
                if math_distance(point, state.position) > 1e-9
            ]
            candidates.append(plan)
            quality_scores.append(
                sum(
                    max(
                        self._expected_intersection_quality(state, channel, point)
                        for point in vertices
                    )
                    for channel in detected_channels
                )
            )
        if not candidates:
            raise ValueError(
                "route-aligned bearing tour has no valid coverage rotation"
            )
        selected_index = min(
            range(len(candidates)),
            key=lambda index: (
                -quality_scores[index],
                candidates[index].rotation_deg,
            ),
        )
        selected = candidates[selected_index]
        remaining = [
            point
            for point in selected.points
            if math_distance(point, state.position) > 1e-9
        ]
        return selected, remaining

    def _localize_target(
        self,
        executor: Problem3Executor,
        channel: int,
        following: tuple[float, float] | None,
    ) -> None:
        localize_channel(
            executor,
            channel,
            "覆盖与目标联合开放旅行商路线；安全余量内顺路清除",
            clear_when_ready=True,
            clear_radius_m=self.settings.robust_clear_radius_m,
            use_grid_fallback=True,
            route_following_point=following,
            optimize_clear_point=True,
        )


class DynamicCoverageRouteAlignedTourStrategy(SafeClearRouteAlignedTourStrategy):
    """达到题设干扰源数量上限后，动态终止已经无必要的剩余覆盖。"""

    name = DYNAMIC_COVERAGE_TOUR_STRATEGY

    def _refresh_remaining(
        self,
        executor: Problem3Executor,
        remaining: list[tuple[float, float]],
    ) -> list[tuple[float, float]]:
        state = executor.state
        found_count = sum(
            track.status in {"detected", "localized", "cleared"}
            for track in state.tracks.values()
        )
        if found_count < SOURCE_COUNT_MAX:
            return remaining
        # 达到题设干扰源数量上限后，其余未知频道必定不存在。
        for track in state.tracks.values():
            if track.status == "unknown":
                track.status = "absent"
        return []
