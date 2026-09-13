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
    search_polygon_plans,
)
from problems.problem3.main import format_run_brief
from problems.problem3.plotting import plot_run_replay
from problems.problem3.shared import (
    JsonlRunLogger,
    Problem3Executor,
    Problem3State,
    summarize_state,
)
from problems.problem3.shared import PlannedAction
from problems.problem3.strategies import BeliefMPCStrategy, guarantee_route_covers_uncovered
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
    paths = plot_run_replay(action_log, main_output, detail_output, truth)
    assert main_output.stat().st_size > 10_000
    assert detail_output.stat().st_size > 5_000
    assert paths["clearance_detail_figure"] == str(detail_output)


def test_rehearsal_summary_reports_exact_time_and_cleared_over_total() -> None:
    """有事后总数时，演练摘要必须显示具体耗时和准确的清除分数。"""

    state = Problem3State(_small_settings())
    state.virtual_time_s = 3723.125
    state.counters.clear_success_count = 10
    summary = summarize_state(state, "belief_mpc", {}, known_source_total=13)
    summary.update({"run_index": 1, "program_wall_time_s": 4.25})
    brief = format_run_brief(summary)
    assert summary["source_total_count"] == 13
    assert summary["cleared_over_total"] == "10/13"
    assert math.isclose(summary["clearance_ratio"], 10 / 13)
    assert "已清除/总数量：10/13" in brief
    assert "总虚拟耗时：3723.125 s (01:02:03.125)" in brief
    assert "虚拟耗时分解：" in brief
    assert "程序实际耗时：4.250 s" in brief


def test_bearing_likelihood_and_conditional_information_gain() -> None:
    """示向±1°须压低角外原子；一次none后的IG必须由条件后验重新计算。"""

    belief = ChannelBelief.initialize(_small_settings(), 7)
    prior_information = belief.information_gain_bits((0.0, 0.0))
    belief.update((0.0, 0.0), "no_signal")
    conditional_information = belief.information_gain_bits((700.0, 0.0))
    assert prior_information > 0.0
    assert conditional_information > 0.0
    virtual = ChannelBelief.initialize(_small_settings(), 8)
    vectors = virtual.points - np.asarray((0.0, 0.0))
    bearings = np.degrees(np.arctan2(vectors[:, 1], vectors[:, 0])) % 360.0
    received = virtual.exists & (np.linalg.norm(vectors, axis=1) <= virtual.radii_m)
    observed = float(bearings[np.flatnonzero(received)[0]])
    virtual.update((0.0, 0.0), "direction", observed)
    outside = np.abs((bearings - observed + 180.0) % 360.0 - 180.0) > 1.02
    assert float(np.sum(virtual.weights[outside & virtual.exists])) < 1e-6


def test_information_gain_matches_observation_entropy() -> None:
    """确定性固定R观测下，KL定义的互信息等于观测标签熵。"""

    belief = ChannelBelief.initialize(_small_settings(), 9)
    labels = belief.predicted_labels((250.0, -300.0))
    masses = np.bincount(labels, weights=belief.weights)
    positive = masses[masses > 0.0]
    expected = float(-np.sum(positive * np.log2(positive)))
    assert math.isclose(belief.information_gain_bits((250.0, -300.0)), expected)


def test_adaptive_grid_inherits_fixed_radius_and_reports_geometry() -> None:
    """空间细分不得生成新半径值，几何输出还须包含网格误差的保守膨胀。"""

    settings = _small_settings(particle_count_per_channel=120)
    belief = ChannelBelief.initialize(settings, 10)
    radii_before = set(belief.radii_m[belief.exists])
    belief.adaptive_refine(settings.fine_grid_size, 240)
    assert set(belief.radii_m[belief.exists]) <= radii_before
    metrics = belief.geometry_metrics()
    assert metrics["effective_grid_count"] > 0
    assert metrics["minimum_enclosing_radius_m"] >= 0.0
    assert "radius_marginal" in metrics


def test_batch_time_bounds_pruning_fallback_and_termination() -> None:
    """批量时间、上下界剪枝、安全回退和16源提前终止均使用显式规则。"""

    settings = _small_settings()
    state = Problem3State(settings)
    state.remaining_real_duration_s = 1200
    strategy = BeliefMPCStrategy(settings)
    batch = PlannedAction("batch_measure", (15.0, 20.0), (1, 3, 2), "test")
    # 移动5秒、三次检测15秒、1->3和3->2两次切频。
    assert math.isclose(strategy._action_time(state, batch), 22.0)
    candidates = [
        PlannedAction("measure", (0.0, 0.0), (1,), "keep", lower_bound_s=10.0),
        PlannedAction("measure", (0.0, 0.0), (2,), "prune", lower_bound_s=101.0),
        PlannedAction("clear", (0.0, 0.0), (3,), "protected", lower_bound_s=101.0),
    ]
    kept = strategy._prune_by_bounds(candidates, 100.0)
    assert [item.reason for item in kept] == ["keep", "protected"]
    assert not strategy._fallback_required(state, [candidates[0]])
    state.no_progress_steps = settings.fallback_no_progress_steps
    assert strategy._fallback_required(state, [candidates[0]])
    state.counters.clear_success_count = 16
    assert state.should_terminate()


def test_b1_unified_channel_state_and_confirmed_observations() -> None:
    """B1：direction/near均锁定存在概率，且所有兼容视图指向同一ChannelState。"""

    state = Problem3State(_small_settings())
    assert state.tracks[2] is state.channel_states[2]
    assert state.beliefs[2] is state.channel_states[2].joint_belief
    belief = state.beliefs[2]
    received = belief.exists & (np.linalg.norm(belief.points, axis=1) <= belief.radii_m)
    index = int(np.flatnonzero(received)[0])
    bearing = float(np.degrees(np.arctan2(belief.points[index, 1], belief.points[index, 0])) % 360.0)
    state.apply_direction(2, (0.0, 0.0), bearing)
    item = state.channel_states[2]
    assert item.status == "detected"
    assert math.isclose(item.existence_probability, 1.0, abs_tol=1e-12)
    assert item.support_region is not None
    state.apply_near(3, (0.0, 0.0))
    near = state.channel_states[3]
    assert near.status == "localized"
    assert math.isclose(near.existence_probability, 1.0, abs_tol=1e-12)
    assert near.pending_tasks == {"clear"}


def test_b1_clear_success_and_failure_update_one_channel_state() -> None:
    """B1：成功清空任务；失败保持存在并从粒子后验排除20m清除圆。"""

    state = Problem3State(_small_settings())
    state.apply_near(4, (0.0, 0.0))
    state.apply_clear(4, False)
    item = state.channel_states[4]
    assert item.status == "detected"
    assert math.isclose(item.existence_probability, 1.0, abs_tol=1e-12)
    assert not np.any(item.joint_belief.exists & (np.linalg.norm(item.joint_belief.points, axis=1) <= 20.0) & (item.joint_belief.weights > 0.0))
    state.apply_clear(4, True)
    assert item.status == "cleared"
    assert not item.pending_tasks


def test_b1_coarse_grid_never_proves_clearance() -> None:
    """B1：150m网格半对角线超过20m，不能凭粒子MEC设置localized。"""

    state = Problem3State(_small_settings(coarse_grid_size=150.0))
    item = state.channel_states[5]
    metrics = item.joint_belief.geometry_metrics()
    assert metrics["grid_resolution_m"] == 150.0
    assert 150.0 * math.sqrt(2.0) / 2.0 > state.settings.clearance_radius_m
    assert item.status == "unknown"
