"""问题四定向保证格与端到端策略测试。"""

from problems.problem3.shared import Problem3Executor
from problems.problem4.config import Problem4Settings
from problems.problem4.model import (
    directional_lattice_points,
    directional_ring_mesh_points,
    validate_directional_lattice,
)
from problems.problem4.strategies import (
    GuaranteedDirectionalLatticeStrategy,
    OptimizedGuaranteedLatticeStrategy,
    Problem4State,
)
from radio_locator.client import ActionExchange
from radio_locator.local_simulator import SimulatorEngine, SimulatorScenario


class _EngineClient:
    def __init__(self, seed: int) -> None:
        self.engine = SimulatorEngine(
            SimulatorScenario.generate(4, seed), "test", monotonic_s=lambda: 0.0
        )
        self.counter = 0

    def check_connection(self) -> None:
        return None

    def _call(self, path: str, position=None, channel=None) -> ActionExchange:
        self.counter += 1
        payload = {
            "arena_id": "default", "robot_id": "test", "request_id": str(self.counter)
        }
        if position is not None:
            payload.update(position={"x": position[0], "y": position[1]}, channel=channel)
        status, response = self.engine.process(path, payload)
        assert status == 200 and response["accepted"]
        return ActionExchange(path, payload, status, response)

    def enter(self) -> ActionExchange:
        return self._call("/enter")

    def measure(self, position, channel) -> ActionExchange:
        return self._call("/measure", position, channel)

    def clear(self, position, channel) -> ActionExchange:
        return self._call("/clear", position, channel)


class _NullLogger:
    def write(self, _record: dict) -> None:
        return None


def test_shifted_lattice_has_27_stations_and_dense_directional_guarantee() -> None:
    settings = Problem4Settings()
    stations = directional_lattice_points(
        settings.arena_radius_m,
        settings.guaranteed_radius_m,
        spacing_m=settings.directional_grid_spacing_m,
        rotation_deg=settings.directional_grid_rotation_deg,
        offset=(settings.directional_grid_offset_x_m, settings.directional_grid_offset_y_m),
    )
    audit = validate_directional_lattice(
        stations, settings.arena_radius_m, settings.guaranteed_radius_m, 50.0
    )
    assert len(stations) == 27
    assert audit["valid"] is True
    assert audit["failed_sample_count"] == 0


def test_concentric_ring_mesh_uses_25_stations_with_same_guarantee() -> None:
    settings = Problem4Settings()
    stations = directional_ring_mesh_points(
        settings.arena_radius_m, settings.guaranteed_radius_m
    )
    audit = validate_directional_lattice(
        stations, settings.arena_radius_m, settings.guaranteed_radius_m, 50.0
    )
    assert len(stations) == 25
    assert audit["valid"] is True


def test_default_optimized_inner_ring_preserves_directional_guarantee() -> None:
    settings = Problem4Settings()
    stations = directional_ring_mesh_points(
        settings.arena_radius_m,
        settings.guaranteed_radius_m,
        inner_radius_m=settings.directional_ring_inner_radius_m,
    )
    audit = validate_directional_lattice(
        stations, settings.arena_radius_m, settings.guaranteed_radius_m, 50.0
    )
    assert len(stations) == 25
    assert audit["valid"] is True


def test_directional_no_signal_does_not_mark_channel_absent_early() -> None:
    state = Problem4State(Problem4Settings())
    state.apply_no_signal(1, (0.0, 0.0))
    assert state.tracks[1].status == "unknown"
    state.finalize_directional_discovery()
    assert state.tracks[1].status == "absent"


def test_guaranteed_strategy_clears_all_sources_on_fixed_mixed_case() -> None:
    settings = Problem4Settings(seed=0)
    state = Problem4State(settings)
    client = _EngineClient(seed=0)
    executor = Problem3Executor(
        client, state, _NullLogger(), GuaranteedDirectionalLatticeStrategy.name
    )
    executor.enter()
    result = GuaranteedDirectionalLatticeStrategy(settings).run(executor)
    assert result.plan["coverage_audit"]["valid"] is True
    assert state.all_resolved()
    assert state.counters.clear_success_count == len(client.engine.scenario.sources)

def test_optimized_guaranteed_strategy_preserves_full_clearance() -> None:
    settings = Problem4Settings(seed=0)
    state = Problem4State(settings)
    client = _EngineClient(seed=0)
    executor = Problem3Executor(
        client, state, _NullLogger(), OptimizedGuaranteedLatticeStrategy.name
    )
    executor.enter()
    result = OptimizedGuaranteedLatticeStrategy(settings).run(executor)
    assert result.plan["coverage_audit"]["valid"] is True
    assert state.all_resolved()
    assert state.counters.clear_success_count == len(client.engine.scenario.sources)
