"""问题 1 几何算法及题意反例测试。"""

import numpy as np

from problems.problem1.examples import validation_cases
from problems.problem1.model import localize
from radio_locator.geometry import (
    brute_force_diameter,
    diameter_circle_coverage,
    rotating_calipers_diameter,
)


def test_rotating_calipers_matches_brute_force() -> None:
    """三个代表案例中，线性算法必须与 O(m²) 枚举结果一致。"""

    for case in validation_cases():
        result = localize(case.measurements)
        brute = brute_force_diameter(result.polygon)
        assert np.isclose(result.diameter.distance, brute.distance, atol=1e-8)


def test_all_known_targets_lie_in_localization_region() -> None:
    """读数误差不超过 1° 时，已知真值应满足交汇区域的全部半平面约束。"""

    for case in validation_cases():
        result = localize(case.measurements)
        target = np.asarray(case.target)
        for start, end in zip(
            result.polygon,
            np.roll(result.polygon, -1, axis=0),
            strict=True,
        ):
            edge = end - start
            relative = target - start
            cross = edge[0] * relative[1] - edge[1] * relative[0]
            assert cross >= -1e-7


def test_counterexample_is_generated_by_three_measurements() -> None:
    """第三个案例必须由三站交会形成，并且直径圆确实不能覆盖交汇区域。"""

    case = validation_cases()[2]
    assert len(case.measurements) == 3
    result = localize(case.measurements)
    covers, _, _, excess = diameter_circle_coverage(
        result.polygon,
        rotating_calipers_diameter(result.polygon),
    )
    assert len(result.polygon) == 4
    assert not covers
    assert excess > 4.0
