"""现行策略注册表的回归测试。"""

from problems.problem3.config import STRATEGIES as PROBLEM3_STRATEGIES
from problems.problem4.config import PROBLEM4_STRATEGIES


def test_only_supported_problem3_strategies_are_registered() -> None:
    assert PROBLEM3_STRATEGIES == (
        "robust_polygon_rolling",
        "belief_mpc",
        "integrated_bearing_tour",
        "distance_optimized_bearing_tour",
        "safe_clear_route_aligned_tour",
        "dynamic_coverage_route_aligned_tour",
    )


def test_only_supported_problem4_strategies_are_registered() -> None:
    assert PROBLEM4_STRATEGIES == (
        "guaranteed_directional_lattice",
        "optimized_guaranteed_lattice",
        "legacy_outer_probe_fast",
    )


def test_removed_strategies_are_not_registered() -> None:
    removed = {
        "cooperative_bearing_tour",
        "route_aligned_bearing_tour",
        "problem4_fast",
    }
    assert removed.isdisjoint(PROBLEM3_STRATEGIES)
    assert removed.isdisjoint(PROBLEM4_STRATEGIES)
