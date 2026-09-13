"""问题四全向/定向混合干扰源的保证发现与联合定位清除策略。"""

from __future__ import annotations

import math
from dataclasses import dataclass

from config.constants import SOURCE_COUNT_MAX
from problems.problem3.coverage import (
    held_karp_open_path,
    math_distance,
    optimize_remaining_route_multistart,
    route_length,
)
from problems.problem3.shared import (
    Problem3Executor,
    Problem3State,
    localize_channel,
    route_aligned_clearance_point,
)
from problems.problem4.config import (
    GUARANTEED_DIRECTIONAL_LATTICE,
    LEGACY_OUTER_PROBE_FAST,
    OPTIMIZED_GUARANTEED_LATTICE,
    Problem4Settings,
)
from problems.problem4.model import (
    directional_lattice_points,
    directional_ring_mesh_points,
    validate_directional_lattice,
)


class Problem4State(Problem3State):
    """禁用问题三的 ``no_signal=>圆域排除``，避免误删定向源真位置。"""

    def apply_no_signal(self, channel: int, position: tuple[float, float]) -> None:
        # 定向源在接收半径内仍可能因背向而无信号；单点阴性不提供位置排除信息。
        return None

    def finalize_directional_discovery(self) -> None:
        """完成保证格扫描后，仍未知的频道才可被严格判为不存在。"""

        for track in self.tracks.values():
            if track.status == "unknown":
                track.status = "absent"


@dataclass(frozen=True, slots=True)
class Problem4RunResult:
    state: Problem4State
    plan: dict[str, object]


def _channel_order(state: Problem4State, channels: list[int]) -> list[int]:
    """优先当前频道，减少同一测站批量扫描的换频次数。"""

    if state.current_channel in channels:
        channels.remove(state.current_channel)
        return [state.current_channel, *sorted(channels)]
    return sorted(channels, key=lambda channel: abs(channel - state.current_channel))


class GuaranteedDirectionalLatticeStrategy:
    """三角格保证发现，途中积累交会示向，末段统一安全清除。"""

    name = GUARANTEED_DIRECTIONAL_LATTICE

    def __init__(self, settings: Problem4Settings) -> None:
        self.settings = settings

    def _opportunistic_measurement_is_useful(
        self,
        state: Problem4State,
        channel: int,
        station: tuple[float, float],
    ) -> bool:
        return True

    def _probe_insertion_limit_m(self) -> float:
        return self.settings.route_probe_insertion_limit_m

    def _ring_inner_radius_m(self) -> float:
        return self.settings.directional_ring_inner_radius_m

    @staticmethod
    def _pending_reference_point(
        state: Problem4State,
        channel: int,
    ) -> tuple[float, float]:
        track = state.tracks[channel]
        if track.clear_circle is not None:
            return tuple(float(value) for value in track.clear_circle.center)
        return track.measurements[-1].station

    def _finish_pending_channels(
        self,
        executor: Problem3Executor,
        pending: list[int],
    ) -> None:
        state = executor.state
        pending_points = [
            self._pending_reference_point(state, channel) for channel in pending
        ]
        if pending_points:
            ordered_points = optimize_remaining_route_multistart(
                state.position, pending_points
            )
            point_channels: dict[tuple[float, float], list[int]] = {}
            for point, channel in zip(pending_points, pending, strict=True):
                point_channels.setdefault(point, []).append(channel)
            pending = [point_channels[point].pop(0) for point in ordered_points]
        for channel in pending:
            localize_channel(
                executor,
                channel,
                "定向源主动复测定位",
                clear_when_ready=True,
                clear_radius_m=self.settings.directional_clear_radius_m,
                use_grid_fallback=True,
                optimize_clear_point=True,
            )

    def _finish_endgame(self, executor: Problem3Executor) -> None:
        """基线收尾：先清除已定位频道，再集中处理其余频道。"""

        state = executor.state
        localized = [
            channel
            for channel, track in state.tracks.items()
            if track.status == "localized" and track.clear_circle is not None
        ]
        localized_points = [
            tuple(float(value) for value in state.tracks[channel].clear_circle.center)
            for channel in localized
        ]
        if localized_points:
            order = optimize_remaining_route_multistart(
                state.position, localized_points
            )
            point_channels = {
                point: channel
                for point, channel in zip(localized_points, localized, strict=True)
            }
            for point in order:
                executor.clear(
                    point, point_channels[point], "定向格多站交会后开放路线安全清除"
                )
        pending = [
            channel
            for channel, track in state.tracks.items()
            if track.status in {"detected", "localized"}
        ]
        self._finish_pending_channels(executor, pending)

    def run(self, executor: Problem3Executor) -> Problem4RunResult:
        state = executor.state
        if not isinstance(state, Problem4State):
            raise TypeError("problem4 strategy requires Problem4State")
        if self.settings.directional_mesh_layout == "concentric_ring":
            stations = directional_ring_mesh_points(
                self.settings.arena_radius_m,
                self.settings.guaranteed_radius_m,
                rotation_deg=self.settings.directional_grid_rotation_deg,
                inner_radius_m=self._ring_inner_radius_m(),
            )
        elif self.settings.directional_mesh_layout == "triangular_lattice":
            stations = directional_lattice_points(
                self.settings.arena_radius_m,
                self.settings.guaranteed_radius_m,
                spacing_m=self.settings.directional_grid_spacing_m,
                rotation_deg=self.settings.directional_grid_rotation_deg,
                offset=(
                    self.settings.directional_grid_offset_x_m,
                    self.settings.directional_grid_offset_y_m,
                ),
            )
        else:
            raise ValueError(
                f"unknown directional mesh: {self.settings.directional_mesh_layout}"
            )
        audit = validate_directional_lattice(
            stations,
            self.settings.arena_radius_m,
            self.settings.guaranteed_radius_m,
        )
        if not audit["valid"]:
            raise ValueError("finite directional lattice failed coverage audit")

        initial_route = optimize_remaining_route_multistart(
            state.position, list(stations)
        )
        planned_route_length = route_length(tuple(initial_route), state.position)
        remaining = list(stations)
        station_index = 0
        inserted_probe_channels: set[int] = set()
        while remaining:
            if state.counters.clear_success_count >= SOURCE_COUNT_MAX:
                break
            # 清除插入会改变当前位置；每轮据此重排剩余骨架，避免返回旧路线造成回折。
            route = optimize_remaining_route_multistart(state.position, remaining)
            station = route[0]
            remaining.remove(station)
            station_index += 1
            channels = [
                channel
                for channel, track in state.tracks.items()
                if track.status == "unknown"
                or (
                    track.status == "detected"
                    and len(track.measurements)
                    < self.settings.opportunistic_bearing_limit
                    and self._opportunistic_measurement_is_useful(
                        state, channel, station
                    )
                )
            ]
            for channel in _channel_order(state, channels):
                executor.measure(
                    station,
                    channel,
                    f"定向保证三角格第{station_index}/{len(stations)}站；"
                    "未知频道发现或已发现频道机会式交会",
                )
            # 只插入几乎顺路的安全清除点；较远目标留到末段统一做开放路线。
            following = route[1] if len(route) > 1 else None
            while True:
                candidates: list[tuple[float, int, tuple[float, float]]] = []
                for channel, track in state.tracks.items():
                    if (
                        track.status != "localized"
                        or track.clear_circle is None
                        or track.clear_circle.radius
                        > self.settings.directional_clear_radius_m
                    ):
                        continue
                    point = route_aligned_clearance_point(
                        track.clear_circle,
                        state.position,
                        following,
                        self.settings.directional_clear_radius_m,
                    )
                    extra = math_distance(state.position, point)
                    if following is not None:
                        extra += math_distance(point, following) - math_distance(
                            state.position, following
                        )
                    candidates.append((extra, channel, point))
                if not candidates:
                    break
                extra, channel, point = min(candidates)
                if extra > self.settings.route_clear_insertion_limit_m:
                    break
                executor.clear(
                    point,
                    channel,
                    f"保证格路线顺路插入安全清除；额外路程{extra:.1f}m",
                )
            # 对已有两条示向但交会仍偏长的频道，在路线附近顺带到区域中心复测。
            # 每频道最多插入一次；背向导致 no_signal 时不会反复访问同一点。
            probe_candidates: list[tuple[float, int, tuple[float, float]]] = []
            for channel, track in state.tracks.items():
                if (
                    channel in inserted_probe_channels
                    or track.status != "detected"
                    or len(track.measurements) < 2
                    or track.clear_circle is None
                ):
                    continue
                point = tuple(float(value) for value in track.clear_circle.center)
                extra = math_distance(state.position, point)
                if following is not None:
                    extra += math_distance(point, following) - math_distance(
                        state.position, following
                    )
                probe_candidates.append((extra, channel, point))
            if probe_candidates:
                extra, channel, point = min(probe_candidates)
                if extra <= self._probe_insertion_limit_m():
                    inserted_probe_channels.add(channel)
                    executor.measure(
                        point,
                        channel,
                        f"保证格路线顺路插入交会复测；额外路程{extra:.1f}m",
                    )

        state.finalize_directional_discovery()
        self._finish_endgame(executor)
        if not state.all_resolved():
            unresolved = state.unresolved_channels()
            raise RuntimeError(
                f"problem4 finished with unresolved channels: {unresolved}"
            )
        plan = {
            "name": self.name,
            "stations": [list(point) for point in stations],
            "station_count": len(stations),
            "planned_route_length_m": planned_route_length,
            "coverage_audit": audit,
        }
        return Problem4RunResult(state, plan)


class OptimizedGuaranteedLatticeStrategy(GuaranteedDirectionalLatticeStrategy):
    """保证格不变，将定位与清除统一为精确开放路径收尾。"""

    name = OPTIMIZED_GUARANTEED_LATTICE

    def _finish_endgame(self, executor: Problem3Executor) -> None:
        state = executor.state
        channels = [
            channel
            for channel, track in state.tracks.items()
            if track.status in {"detected", "localized"}
        ]
        points = [self._pending_reference_point(state, channel) for channel in channels]
        order = held_karp_open_path(state.position, points)
        ordered_channels = [channels[index] for index in order]
        ordered_points = [points[index] for index in order]
        for index, channel in enumerate(ordered_channels):
            following = (
                ordered_points[index + 1] if index + 1 < len(ordered_points) else None
            )
            track = state.tracks[channel]
            if track.status == "localized" and track.clear_circle is not None:
                point = route_aligned_clearance_point(
                    track.clear_circle,
                    state.position,
                    following,
                    self.settings.directional_clear_radius_m,
                )
                executor.clear(point, channel, "精确联合开放路线安全清除")
                continue
            localize_channel(
                executor,
                channel,
                "精确联合开放路线主动定位",
                clear_when_ready=True,
                clear_radius_m=self.settings.directional_clear_radius_m,
                use_grid_fallback=True,
                route_following_point=following,
                optimize_clear_point=True,
            )


class LegacyOuterProbeFastStrategy:
    """复现父目录p4项目的“七边形基础扫描+外侧六点补测”路线。

    与原项目相比这里只替换了状态更新和定位清除的工程实现；13个发现测站、两阶段
    扫描顺序和“频道首次检出后停止发现扫描”的算法思想保持一致。该点集没有定向
    完备证明，因此策略允许以未解决频道结束，供完成率优先的公平比较。
    """

    name = LEGACY_OUTER_PROBE_FAST

    def __init__(self, settings: Problem4Settings) -> None:
        self.settings = settings

    @staticmethod
    def _outer_probe_points() -> list[tuple[float, float]]:
        radius = 2_246.0
        return [
            (
                radius * math.cos(math.radians(30.0 + index * 60.0)),
                radius * math.sin(math.radians(30.0 + index * 60.0)),
            )
            for index in range(6)
        ]

    def _scan_phase(
        self,
        executor: Problem3Executor,
        points: list[tuple[float, float]],
        phase: str,
    ) -> None:
        state = executor.state
        route = optimize_remaining_route_multistart(state.position, points)
        for station_index, point in enumerate(route, start=1):
            channels = [
                channel
                for channel, track in state.tracks.items()
                if track.status == "unknown"
            ]
            for channel in _channel_order(state, channels):
                executor.measure(
                    point,
                    channel,
                    f"历史外侧补测策略：{phase}第{station_index}/{len(route)}站",
                )

    def run(self, executor: Problem3Executor) -> Problem4RunResult:
        state = executor.state
        if not isinstance(state, Problem4State):
            raise TypeError("problem4 strategy requires Problem4State")
        base = [
            (
                1_000.0 * math.cos(math.radians(7.5 + index * 360.0 / 7.0)),
                1_000.0 * math.sin(math.radians(7.5 + index * 360.0 / 7.0)),
            )
            for index in range(7)
        ]
        outer = self._outer_probe_points()
        self._scan_phase(executor, base, "七边形基础覆盖")
        self._scan_phase(executor, outer, "外侧定向补测")

        # 原策略按频道号集中定位；沿用该顺序以忠实比较路线思想。
        for channel in sorted(self.settings.channels):
            if state.tracks[channel].status in {"detected", "localized"}:
                localize_channel(
                    executor,
                    channel,
                    "历史外侧补测策略集中定位清除",
                    clear_when_ready=True,
                    clear_radius_m=self.settings.directional_clear_radius_m,
                    use_grid_fallback=True,
                )
        plan = {
            "name": self.name,
            "source_project": "../p4 ",
            "base_station_count": len(base),
            "outer_station_count": len(outer),
            "station_count": len(base) + len(outer),
            "base_points": [list(point) for point in base],
            "outer_points": [list(point) for point in outer],
            "directional_guarantee": False,
        }
        return Problem4RunResult(state, plan)
