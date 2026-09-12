"""共享交会与覆盖/清除联合路线的回归测试，不依赖绘图模块。"""
from pathlib import Path
import numpy as np
from problems.problem3.config import Problem3Settings
from problems.problem3.shared import Problem3State, Problem3Executor, JsonlRunLogger
from problems.problem3.strategies import IntegratedBearingTourStrategy, CooperativeBearingTourStrategy
from radio_locator.local_simulator import SimulatorScenario, SimulatorEngine
from radio_locator.client import ActionExchange

class EngineClient:
    def __init__(self, scenario):
        self.engine = SimulatorEngine(scenario, 'test', monotonic_s=lambda: 0.0)
        self.request_id = 0
    def check_connection(self):
        pass
    def call(self, path, position=None, channel=None):
        self.request_id += 1
        payload = dict(arena_id='default', robot_id='test', request_id=str(self.request_id))
        if position is not None:
            payload.update(position=dict(x=position[0], y=position[1]), channel=channel)
        status, response = self.engine.process(path, payload)
        assert status == 200 and response['accepted']
        return ActionExchange(path, payload, status, response)
    def enter(self):
        return self.call('/enter')
    def measure(self, position, channel):
        return self.call('/measure', position, channel)
    def clear(self, position, channel):
        return self.call('/clear', position, channel)

def test_shared_station_does_not_repeat_same_bearing():
    state = Problem3State(Problem3Settings())
    state.apply_direction(1, (0.0, 0.0), 0.0)
    assert not CooperativeBearingTourStrategy._useful_bearing(state, 1, (0.0, 0.0))
    assert CooperativeBearingTourStrategy._useful_bearing(state, 1, (500.0, 300.0))

def test_integrated_tour_resolves_empty_arena(tmp_path: Path):
    state = Problem3State(Problem3Settings())
    client = EngineClient(SimulatorScenario(3, 1, []))
    logger = JsonlRunLogger(tmp_path / 'empty.jsonl')
    try:
        executor = Problem3Executor(client, state, logger, 'integrated_bearing_tour')
        executor.enter()
        result = IntegratedBearingTourStrategy(state.settings).run(executor)
    finally:
        logger.close()
    assert result.selected_plan.coverage.valid
    assert state.all_resolved()
    assert all(t.status == 'absent' for t in state.tracks.values())
    assert all(not np.any(u) for u in state.uncovered.values())

def test_integrated_tour_clears_seed5_without_oracle(tmp_path: Path):
    state = Problem3State(Problem3Settings())
    client = EngineClient(SimulatorScenario.generate(3, 5))
    logger = JsonlRunLogger(tmp_path / 'seed5.jsonl')
    try:
        executor = Problem3Executor(client, state, logger, 'integrated_bearing_tour')
        executor.enter()
        IntegratedBearingTourStrategy(state.settings).run(executor)
    finally:
        logger.close()
    assert state.all_resolved()
    assert all(source.cleared for source in client.engine.scenario.sources)
    assert state.counters.clear_failure_count == 0
    # 宽裕的性能回归线：上一版同场景5677秒，避免退回逐源专程交会。
    assert state.virtual_time_s < 4500.0

def test_origin_hexagon_exact_coverage_boundary():
    import math
    from problems.problem3.coverage import evaluate_polygon_plan
    settings = Problem3Settings()
    lower = 1800 * math.cos(math.pi / 6) - math.sqrt(1000**2 - (1800 * math.sin(math.pi / 6))**2)
    assert not evaluate_polygon_plan(settings, 6, lower - 0.01, 0.0, True).coverage.valid
    assert evaluate_polygon_plan(settings, 6, lower + 0.01, 0.0, True).coverage.valid
    plan = evaluate_polygon_plan(settings, 6, 1150.0, 0.0, True)
    assert plan.coverage.valid
    assert abs(plan.coverage.analytic_max_distance_m - 988.5114204360128) < 1e-6
    # 密集极坐标网格含外边界，验证解析最大值不低估。
    from problems.problem3.coverage import regular_polygon_points
    points = np.asarray(regular_polygon_points(6, 1150.0, scan_origin=True))
    angles = np.linspace(0, 2 * math.pi, 721)
    radii = np.linspace(0, 1800, 181)
    grid = (radii[:, None, None] * np.stack([np.cos(angles), np.sin(angles)], axis=1)).reshape(-1, 2)
    worst = np.min(np.linalg.norm(grid[:, None, :] - points[None, :, :], axis=2), axis=1).max()
    assert worst <= plan.coverage.analytic_max_distance_m + 1e-6

def test_alternating_channel_order_and_switch_cost():
    from dataclasses import replace
    settings = replace(Problem3Settings(), tour_channel_order='alternating')
    state = Problem3State(settings)
    strategy = IntegratedBearingTourStrategy(settings)
    first = strategy._channel_order(state, [1, 2, 3, 4])
    state.current_channel = first[-1]
    assert first == [1, 2, 3, 4]
    # 没有检测的站不推进交替方向。
    assert strategy._channel_order(state, []) == []
    assert strategy._channel_order(state, [1, 2, 3, 4]) == [4, 3, 2, 1]
    client = EngineClient(SimulatorScenario(3, 1, []))
    client.enter()
    first_response = client.measure((0., 0.), 20).response
    second_response = client.measure((0., 0.), 1).response
    assert first_response['virtual_time_s'] == 6
    assert second_response['virtual_time_s'] - first_response['virtual_time_s'] == 6

def test_endgame_p5_includes_uncertain_sources_but_not_undetected():
    from dataclasses import replace
    from types import SimpleNamespace
    from radio_locator.geometry import MinimumEnclosingCircle
    settings = replace(Problem3Settings(), tour_endgame_mode='probe')
    state = Problem3State(settings)
    # 从真实P5历史观测重建所得的中心与半径；没有添加后来才发现的C3。
    known = {7: ((1267.310384, -575.057585), 586.901144),
             11: ((-152.572564, -1387.786605), 576.191942),
             14: ((-890.985458, -1165.643260), 358.677664),
             18: ((752.247725, -608.179092), 22.219198)}
    for channel, (point, radius) in known.items():
        state.tracks[channel].status = 'detected'
        state.tracks[channel].clear_circle = MinimumEnclosingCircle(np.array(point), radius)
    strategy = IntegratedBearingTourStrategy(settings)
    p6 = (575., -995.929214)
    targets = strategy._target_candidates(state, [p6])
    assert {c for c,p in targets} == {7,11,14,18}
    assert 3 not in {c for c,p in targets}
    route = strategy._order_points((-575., -995.929214), [p6]+[p for c,p in targets])
    assert route[0] == known[14][0]
    original = IntegratedBearingTourStrategy(replace(settings,tour_endgame_mode='legacy'))
    assert [c for c,p in original._target_candidates(state,[p6])] == [18]
    # 仍有三个覆盖点时，保留原先的精度门槛。
    assert [c for c,p in strategy._target_candidates(state,[p6]*3)] == [18]

def test_probe_has_lateral_baseline():
    from types import SimpleNamespace
    track = SimpleNamespace(measurements=[SimpleNamespace(bearing_deg=0.0)])
    point = IntegratedBearingTourStrategy._probe_point(track,(0.,0.),(500.,0.),(1000.,100.))
    assert point == (500.,150.)
