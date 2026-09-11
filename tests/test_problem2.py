"""问题 2 接收概率及选点粗筛测试。"""

import numpy as np

from problems.problem1.model import BearingMeasurement
from problems.problem2.model import (
    build_posterior_grid,
    evaluate_candidate,
    reception_probability,
    select_second_point,
)


def test_reception_probability_piecewise_definition() -> None:
    """论文式 (17) 的端点与过渡区应严格符合分段定义。"""

    distances = np.array([0.0, 1_000.0, 1_250.0, 1_500.0, 1_700.0])
    expected = np.array([1.0, 1.0, 0.5, 0.0, 0.0])
    assert np.allclose(reception_probability(distances), expected)


def test_first_station_itself_fails_near_distance_screen() -> None:
    """第二检测点不能与第一检测点重合。"""

    measurement = BearingMeasurement((-900.0, -500.0), 30.0)
    posterior = build_posterior_grid(measurement, grid_step_m=90.0)
    score = evaluate_candidate(measurement.station, measurement, posterior)
    assert not score.feasible


def test_selected_point_belongs_to_reported_excellent_region() -> None:
    """最终点必须来自论文定义的 5% 近优集合，并输出至少一个区域边界。"""

    result = select_second_point(BearingMeasurement((-900.0, -500.0), 31.363757))
    assert result.selected in result.excellent_scores
    assert result.excellent_region_hulls
