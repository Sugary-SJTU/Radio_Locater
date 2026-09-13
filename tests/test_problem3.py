"""问题3覆盖证明、联合粒子信念、时间规则与滚动安全约束测试。"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from problems.problem3.belief import ChannelBelief
from problems.problem3.config import Problem3Settings
from problems.problem3.coverage import (
    analytic_polygon_max_distance,
    evaluate_polygon_plan,
    held_karp_open_path,
    search_polygon_plans,
)
from problems.problem3.main import (
    format_run_report,
    load_truth_statistics,
    summarize_action_times,
)
from problems.problem3.plotting import plot_run_replay
from problems.problem3.shared import (
    JsonlRunLogger,
    Problem3Executor,
    Problem3State,
    fallback_clearance_grid,
)
from problems.problem3.strategies import (
    RobustPolygonRollingStrategy,
    guarantee_route_covers_uncovered,
)
from radio_locator.client import ActionExchange
from radio_locator.geometry import minimum_enclosing_circle


class FakeClient:
    """只用于执行器单测的附件形状响应，不参与策略性能测试。"""

    def __init__(self, measure_result: str = "no_signal") -> None:
        self.measure_result = measure_result
        self.virtual_time = 0.0
        self.position = (0.0, 0.0)
        self.channel = 1
        self.clear_calls = 0

    def check_connection(self) -> None:
        return None

    def enter(self) -> ActionExchange:
        response = {
            "accepted": True,
            "virtual_time_s": 0.0,
            "remaining_real_duration_s": 1200,
            "max_virtual_duration_s": 360000,
            "max_real_duration_s": 1200,
        }
        return ActionExchange("/enter", {}, 200, response)

    def measure(self, position: tuple[float, float], channel: int) -> ActionExchange:
        self.virtual_time += math.dist(self.position, position) / 5.0
        self.virtual_time += 5.0 + (1.0 if channel != self.channel else 0.0)
        self.position = position
        self.channel = channel
        response = {
            "accepted": True,
            "virtual_time_s": self.virtual_time,
            "measure_result": self.measure_result,
        }
        if self.measure_result == "direction":
            response["svd_deg"] = 0.0
        request = {"position": {"x": position[0], "y": position[1]}, "channel": channel}
        return ActionExchange("/measure", request, 200, response)

    def clear(self, position: tuple[float, float], channel: int) -> ActionExchange:
        self.virtual_time += math.dist(self.position, position) / 5.0 + 5.0
        self.position = position
        self.clear_calls += 1
        request = {"position": {"x": position[0], "y": position[1]}, "channel": channel}
        return ActionExchange(
            "/clear", request, 200,
            {"accepted": True, "virtual_time_s": self.virtual_time, "clear_result": "success"},
        )


def _small_settings(**changes: object) -> Problem3Settings:
    values = {
        "particle_count_per_channel": 80,
        "coverage_grid_step_m": 200.0,
        "coverage_validation_step_m": 25.0,
        "robustness_margin_m": 0.0,
    }
    values.update(changes)
    return Problem3Settings(**values)


def test_polygon_formula_matches_numerical_and_seven_side_baseline() -> None:
    """解析上界不得小于网格观测值，且七边形基准理想条件恰好覆盖。"""

    settings = _small_settings(optimize_polygon=False)
    plan = evaluate_polygon_plan(settings, 7, 1000.0, 0.0, False)
    expected = analytic_polygon_max_distance(7, 1000.0, 1800.0)
    assert math.isclose(expected, 1000.0, abs_tol=1e-9)
    assert plan.coverage.numerical_max_distance_m <= expected + 25.0
    assert plan.coverage.valid
    assert math.isclose(plan.coverage.coverage_ratio, 1.0)


def test_polygon_search_really_compares_sides_radius_rotation_and_origin() -> None:
    """参数搜索必须覆盖四个维度，并只从连续覆盖有效的组合中选最短者。"""

    settings = _small_settings(optimize_polygon=True)
    best, plans = search_polygon_plans(settings)
    expected_count = (
        len(settings.polygon_side_candidates)
        * len(settings.polygon_radius_candidates_m)
        * len(settings.polygon_rotation_candidates_deg)
        * len(settings.scan_origin_candidates)
    )
    assert len(plans) == expected_count
    assert {plan.sides for plan in plans} == {6, 7, 8, 9}
    assert {plan.scan_origin for plan in plans} == {False, True}
    assert best.coverage.valid
    assert best.estimated_base_time_s == min(
        plan.estimated_base_time_s for plan in plans if plan.coverage.valid
    )


def test_robust_strategy_uses_origin_plus_hexagon_independently(tmp_path: Path) -> None:
    """保守策略固定使用先前确定的原点+1200 m正六边形，不受MPC参数搜索影响。"""

    settings = _small_settings()
    state = Problem3State(settings)
    logger = JsonlRunLogger(tmp_path / "robust.jsonl")
    executor = Problem3Executor(FakeClient("no_signal"), state, logger, "robust")
    executor.enter()
    result = RobustPolygonRollingStrategy(settings).run(executor)
    logger.close()
    assert result.selected_plan.sides == 6
    assert result.selected_plan.scan_origin
    assert math.isclose(result.selected_plan.radius_m, 1200.0)
    assert len(result.selected_plan.points) == 7
    assert math.isclose(result.selected_plan.route_length_m, 7200.0, abs_tol=1e-6)
    assert state.all_resolved()


def test_held_karp_returns_shortest_open_clearance_path() -> None:
    """末端清除路径不返航，并选择比输入顺序更短的精确访问顺序。"""

    points = [(10.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    order = held_karp_open_path((0.0, 0.0), points)
    assert order == [1, 2, 0]


def test_28m_fallback_grid_covers_localization_region() -> None:
    """定位多边形内的密集验证点到某个保留网格中心均严格小于20 m。"""

    region = np.asarray([[0.0, 0.0], [84.0, 0.0], [84.0, 56.0], [0.0, 56.0]])
    grid = np.asarray(fallback_clearance_grid(region, 28.0, 20.0, (0.0, 0.0)))
    samples = np.asarray(
        [(x, y) for x in np.linspace(0.0, 84.0, 22) for y in np.linspace(0.0, 56.0, 15)]
    )
    distances = np.linalg.norm(samples[:, None, :] - grid[None, :, :], axis=2)
    nearest = np.min(distances, axis=1)
    assert float(np.max(nearest)) < 20.0


def test_none_downweights_reachable_particles_and_radius_stays_fixed() -> None:
    """none必须排除当前位置可接收粒子；多次更新不能重抽固定R_j。"""

    belief = ChannelBelief.initialize(_small_settings(), 4)
    radii = belief.radii_m.copy()
    reachable = belief.exists & (
        np.linalg.norm(belief.points, axis=1) <= belief.radii_m
    )
    before = float(np.sum(belief.weights[reachable]))
    belief.update((0.0, 0.0), "no_signal")
    after = float(np.sum(belief.weights[reachable]))
    belief.update((100.0, 50.0), "no_signal")
    assert after < before
    assert np.array_equal(belief.radii_m, radii)


def test_single_channel_measure_does_not_change_other_entropy() -> None:
    """一次measure只更新目标频道，不能误减其他频道熵。"""

    state = Problem3State(_small_settings())
    other_weights = state.beliefs[2].weights.copy()
    state.apply_no_signal(1, (0.0, 0.0))
    assert np.array_equal(state.beliefs[2].weights, other_weights)


def test_measure_time_clear_channel_and_near_immediate_clear(tmp_path: Path) -> None:
    """移动、检测、切频计时齐全；near立即清除，clear保持频道。"""

    state = Problem3State(_small_settings())
    fake = FakeClient("near")
    logger = JsonlRunLogger(tmp_path / "actions.jsonl")
    executor = Problem3Executor(fake, state, logger, "test")
    executor.enter()
    executor.measure((3.0, 4.0), 2, "test near")
    logger.close()
    assert math.isclose(state.virtual_time_s, 12.0)
    assert state.counters.measure_count == 1
    assert state.counters.switch_count == 1
    assert state.counters.clear_success_count == 1
    assert fake.clear_calls == 1
    assert state.current_channel == 2


def test_absent_only_after_uncovered_set_is_empty() -> None:
    """一次none不能武断标absent；完整七边形保证扫描后才允许。"""

    settings = _small_settings(optimize_polygon=False)
    state = Problem3State(settings)
    state.apply_no_signal(1, (1000.0, 0.0))
    assert state.tracks[1].status == "unknown"
    plan = evaluate_polygon_plan(settings, 7, 1000.0, 0.0, False)
    for point in plan.points[1:]:
        state.apply_no_signal(1, point)
    assert not np.any(state.uncovered[1])
    assert state.tracks[1].status == "absent"


def test_mec_not_diameter_is_clearance_condition() -> None:
    """边长40的等边三角形直径为40，但最小覆盖圆半径大于20。"""

    triangle = np.asarray([[0.0, 0.0], [40.0, 0.0], [20.0, 20.0 * math.sqrt(3.0)]])
    circle = minimum_enclosing_circle(triangle)
    assert circle.radius > 20.0
    assert math.isclose(circle.radius, 40.0 / math.sqrt(3.0), rel_tol=1e-8)


def test_mpc_keeps_a_completable_guaranteed_route() -> None:
    """任意非覆盖动作前后，保留的正多边形骨架仍覆盖所有unknown的U_j。"""

    settings = _small_settings(optimize_polygon=False)
    state = Problem3State(settings)
    plan = evaluate_polygon_plan(settings, 7, 1000.0, 0.0, False)
    route = list(plan.points)
    assert guarantee_route_covers_uncovered(state, route)
    state.position = (321.0, -456.0)
    assert guarantee_route_covers_uncovered(state, route)


def test_replay_plot_contains_main_view_and_clearance_details(tmp_path: Path) -> None:
    """有事后真值时，每局必须同时生成全局轨迹图和独立清除特写。"""

    state = Problem3State(_small_settings())
    fake = FakeClient("near")
    action_log = tmp_path / "actions.jsonl"
    logger = JsonlRunLogger(action_log)
    executor = Problem3Executor(fake, state, logger, "test")
    executor.enter()
    executor.measure((3.0, 4.0), 2, "plot test")
    logger.close()
    truth = tmp_path / "truth.json"
    truth.write_text(
        json.dumps(
            {
                "sources": [
                    {"channel": 2, "x_m": 3.0, "y_m": 4.0,
                     "reception_radius_m": 1000.0}
                ]
            }
        ),
        encoding="utf-8",
    )
    main_output = tmp_path / "trajectory.png"
    detail_output = tmp_path / "details.png"
    timing_output = tmp_path / "time_breakdown.png"
    timing = summarize_action_times(action_log, state.virtual_time_s)
    paths = plot_run_replay(
        action_log,
        main_output,
        detail_output,
        truth,
        time_breakdown_s=timing,
        timing_output=timing_output,
    )
    assert main_output.stat().st_size > 10_000
    assert main_output.with_suffix(".pdf").stat().st_size > 5_000
    assert detail_output.stat().st_size > 5_000
    assert detail_output.with_suffix(".pdf").stat().st_size > 5_000
    assert timing_output.stat().st_size > 5_000
    assert timing_output.with_suffix(".pdf").stat().st_size > 5_000
    assert paths["clearance_detail_figure"] == str(detail_output)
    assert math.isclose(timing["movement"], 1.0)
    assert math.isclose(timing["channel_switch"], 1.0)
    assert math.isclose(timing["detection"], 5.0)
    assert math.isclose(timing["clearance"], 5.0)


def test_problem4_truth_counts_and_console_report(tmp_path: Path) -> None:
    """问题4报告应分别显示全向和定向源数量。"""

    truth = tmp_path / "problem4.json"
    truth.write_text(
        json.dumps(
            {
                "problem": 4,
                "sources": [
                    {"source_type": "omnidirectional"},
                    {"source_type": "directional"},
                    {"source_type": "directional"},
                ],
            }
        ),
        encoding="utf-8",
    )
    statistics = load_truth_statistics(truth)
    assert statistics["source_count"] == 3
    assert statistics["omnidirectional_source_count"] == 1
    assert statistics["directional_source_count"] == 2
    report = format_run_report(
        {
            **statistics,
            "strategy": "test",
            "total_virtual_time_s": 100.0,
            "cleared_count": 2,
            "time_breakdown_s": {"movement": 50.0},
        }
    )
    assert "全向 1，定向 2" in report
    assert "总清除数量：2" in report

def test_robust_finishes_station_scan_before_localization(tmp_path: Path) -> None:
    """发现信号后仍先完成同站其他频道，杜绝逐频道往返覆盖点。"""
    from unittest.mock import patch
    from problems.problem3 import strategies
    state = Problem3State(_small_settings())
    client = FakeClient('direction')
    logger = JsonlRunLogger(tmp_path / 'batch.jsonl')
    executor = Problem3Executor(client, state, logger, 'robust')
    def finish(executor, channel, *args, **kwargs):
        assert executor.state.counters.measure_count == len(state.settings.channels)
        executor.state.tracks[channel].status = 'cleared'
    try:
        executor.enter()
        with patch.object(strategies, 'localize_channel', side_effect=finish):
            RobustPolygonRollingStrategy(state.settings).run(executor)
    finally:
        logger.close()
    assert state.all_resolved()
