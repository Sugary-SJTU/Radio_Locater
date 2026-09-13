"""同种子配对比较原联合巡回与选定的距离优化变体。"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from dataclasses import asdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]

from benchmark_problem3 import EngineClient  # noqa: E402
from problems.problem3.config import Problem3Settings  # noqa: E402
from problems.problem3.shared import Problem3Executor, Problem3State  # noqa: E402
from problems.problem3.strategies import (  # noqa: E402
    DistanceOptimizedBearingTourStrategy,
    IntegratedBearingTourStrategy,
    RouteAlignedBearingTourStrategy,
    SafeClearRouteAlignedTourStrategy,
    DynamicCoverageRouteAlignedTourStrategy,
)


class NullLogger:
    """基准测试只保留汇总，不写逐动作日志。"""

    def write(self, record: dict[str, object]) -> None:
        del record


def run_one(
    seed: int,
    variant: str,
    *,
    tour_polygon_sides: int | None = None,
    tour_polygon_radius_m: float | None = None,
) -> dict[str, object]:
    settings_values: dict[str, object] = {"seed": seed}
    if tour_polygon_sides is not None:
        key = (
            "route_aligned_polygon_sides"
            if variant in {
                "route_aligned_bearing_tour",
                "safe_clear_route_aligned_tour",
                "dynamic_coverage_route_aligned_tour",
            }
            else "tour_polygon_sides"
        )
        settings_values[key] = tour_polygon_sides
    if tour_polygon_radius_m is not None:
        key = (
            "route_aligned_polygon_radius_m"
            if variant in {
                "route_aligned_bearing_tour",
                "safe_clear_route_aligned_tour",
                "dynamic_coverage_route_aligned_tour",
            }
            else "tour_polygon_radius_m"
        )
        settings_values[key] = tour_polygon_radius_m
    settings = Problem3Settings(**settings_values)
    state = Problem3State(settings)
    client = EngineClient(seed)
    strategy_type = {
        "integrated_bearing_tour": IntegratedBearingTourStrategy,
        "distance_optimized_bearing_tour": DistanceOptimizedBearingTourStrategy,
        "route_aligned_bearing_tour": RouteAlignedBearingTourStrategy,
        "safe_clear_route_aligned_tour": SafeClearRouteAlignedTourStrategy,
        "dynamic_coverage_route_aligned_tour": DynamicCoverageRouteAlignedTourStrategy,
    }[variant]
    error: str | None = None
    started = time.monotonic()
    try:
        executor = Problem3Executor(client, state, NullLogger(), variant)
        executor.enter()
        strategy_type(settings).run(executor)
    except Exception as exception:  # 单个坏场景必须保留，不能中断整组配对。
        error = f"{type(exception).__name__}: {exception}"
    return {
        "seed": seed,
        "variant": variant,
        "source_count": len(client.engine.scenario.sources),
        "time_s": state.virtual_time_s,
        "resolved": state.all_resolved(),
        "error": error,
        "wall_time_s": time.monotonic() - started,
        **asdict(state.counters),
    }


def summarize(rows: list[dict[str, object]]) -> dict[str, object]:
    return {
        "runs": len(rows),
        "resolved": sum(bool(row["resolved"]) for row in rows),
        "errors": sum(row["error"] is not None for row in rows),
        "mean_time_s": statistics.mean(float(row["time_s"]) for row in rows),
        "mean_movement_distance_m": statistics.mean(
            float(row["movement_distance_m"]) for row in rows
        ),
        "mean_measure_count": statistics.mean(
            int(row["measure_count"]) for row in rows
        ),
        "mean_wall_time_s": statistics.mean(
            float(row["wall_time_s"]) for row in rows
        ),
        "clear_failures": sum(int(row["clear_failure_count"]) for row in rows),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--start", type=int, default=0)
    parser.add_argument("--count", type=int, default=60)
    parser.add_argument(
        "--baseline",
        choices=(
            "integrated_bearing_tour",
            "route_aligned_bearing_tour",
            "safe_clear_route_aligned_tour",
        ),
        default="integrated_bearing_tour",
    )
    parser.add_argument(
        "--candidate",
        choices=(
            "distance_optimized_bearing_tour",
            "route_aligned_bearing_tour",
            "safe_clear_route_aligned_tour",
            "dynamic_coverage_route_aligned_tour",
        ),
        default="distance_optimized_bearing_tour",
    )
    parser.add_argument("--candidate-tour-sides", type=int)
    parser.add_argument("--candidate-tour-radius", type=float)
    arguments = parser.parse_args()
    if arguments.start < 0 or arguments.count <= 0:
        raise ValueError("start must be nonnegative and count must be positive")
    if (arguments.candidate_tour_sides is None) != (
        arguments.candidate_tour_radius is None
    ):
        raise ValueError("candidate tour sides and radius must be provided together")
    if arguments.candidate_tour_sides is not None and (
        arguments.candidate_tour_sides < 3
        or arguments.candidate_tour_radius <= 0.0
    ):
        raise ValueError("candidate polygon requires at least 3 sides and positive radius")

    variants = (arguments.baseline, arguments.candidate)
    rows: list[dict[str, object]] = []
    for seed in range(arguments.start, arguments.start + arguments.count):
        for variant in variants:
            candidate_options = (
                {
                    "tour_polygon_sides": arguments.candidate_tour_sides,
                    "tour_polygon_radius_m": arguments.candidate_tour_radius,
                }
                if variant == arguments.candidate
                else {}
            )
            row = run_one(seed, variant, **candidate_options)
            rows.append(row)
            print(
                f"seed={seed} variant={variant} time={row['time_s']:.3f} "
                f"distance={row['movement_distance_m']:.3f} "
                f"resolved={row['resolved']}",
                flush=True,
            )

    grouped = {
        variant: [row for row in rows if row["variant"] == variant]
        for variant in variants
    }
    paired = []
    for seed in range(arguments.start, arguments.start + arguments.count):
        original = next(
            row for row in grouped[variants[0]] if row["seed"] == seed
        )
        optimized = next(
            row for row in grouped[variants[1]] if row["seed"] == seed
        )
        paired.append(
            {
                "seed": seed,
                "time_saved_s": float(original["time_s"])
                - float(optimized["time_s"]),
                "distance_saved_m": float(original["movement_distance_m"])
                - float(optimized["movement_distance_m"]),
            }
        )
    mean_time_saved = statistics.mean(row["time_saved_s"] for row in paired)
    mean_distance_saved = statistics.mean(
        row["distance_saved_m"] for row in paired
    )
    original_summary = summarize(grouped[variants[0]])
    optimized_summary = summarize(grouped[variants[1]])
    document = {
        "seed_start": arguments.start,
        "seed_count": arguments.count,
        "baseline": arguments.baseline,
        "candidate_tour_sides": arguments.candidate_tour_sides,
        "candidate_tour_radius_m": arguments.candidate_tour_radius,
        "summary": {
            variants[0]: original_summary,
            variants[1]: optimized_summary,
        },
        "paired_summary": {
            "optimized_faster": sum(row["time_saved_s"] > 1e-9 for row in paired),
            "ties": sum(abs(row["time_saved_s"]) <= 1e-9 for row in paired),
            "optimized_slower": sum(row["time_saved_s"] < -1e-9 for row in paired),
            "mean_time_saved_s": mean_time_saved,
            "median_time_saved_s": statistics.median(
                row["time_saved_s"] for row in paired
            ),
            "best_time_saved_s": max(row["time_saved_s"] for row in paired),
            "worst_time_saved_s": min(row["time_saved_s"] for row in paired),
            "mean_time_reduction_percent": 100.0
            * mean_time_saved
            / float(original_summary["mean_time_s"]),
            "mean_distance_saved_m": mean_distance_saved,
            "mean_distance_reduction_percent": 100.0
            * mean_distance_saved
            / float(original_summary["mean_movement_distance_m"]),
        },
        "paired": paired,
        "runs": rows,
    }
    parameter_suffix = (
        f"_n{arguments.candidate_tour_sides}_r{arguments.candidate_tour_radius:g}"
        if arguments.candidate_tour_sides is not None
        and arguments.candidate_tour_radius is not None
        else ""
    )
    output = (
        ROOT
        / "res"
        / "tables"
        / "problem3"
        / f"{arguments.candidate}{parameter_suffix}_comparison_{arguments.start}_{arguments.count}.json"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(document, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(document["summary"], ensure_ascii=False, indent=2))
    print(json.dumps(document["paired_summary"], ensure_ascii=False, indent=2))
    print(f"output={output}")


if __name__ == "__main__":
    main()
