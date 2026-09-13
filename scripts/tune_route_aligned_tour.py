"""以共同种子筛选 route-aligned 覆盖多边形边数与半径。"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]

from benchmark_distance_tour import run_one, summarize  # noqa: E402
from problems.problem3.config import Problem3Settings  # noqa: E402
from problems.problem3.coverage import evaluate_polygon_plan  # noqa: E402


COARSE_CONFIGURATIONS = (
    (6, 1125.0), (6, 1150.0), (6, 1175.0), (6, 1200.0),
    (7, 1000.0), (7, 1025.0), (7, 1050.0), (7, 1075.0),
    (8, 950.0), (8, 975.0), (8, 1000.0),
    (9, 925.0), (9, 950.0),
)
FINE_CONFIGURATIONS = tuple((7, float(radius)) for radius in range(1025, 1076, 5))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--start", type=int, default=0)
    parser.add_argument("--count", type=int, default=30)
    parser.add_argument("--stage", choices=("coarse", "fine"), default="coarse")
    arguments = parser.parse_args()
    if arguments.start < 0 or arguments.count <= 0:
        raise ValueError("start must be nonnegative and count must be positive")

    configurations: list[dict[str, object]] = []
    all_rows: list[dict[str, object]] = []
    grid = FINE_CONFIGURATIONS if arguments.stage == "fine" else COARSE_CONFIGURATIONS
    for sides, radius in grid:
        settings = Problem3Settings(
            route_aligned_polygon_sides=sides,
            route_aligned_polygon_radius_m=radius,
        )
        plan = evaluate_polygon_plan(settings, sides, radius, 0.0, True)
        if not plan.coverage.valid:
            continue
        rows = [
            run_one(
                seed,
                "route_aligned_bearing_tour",
                tour_polygon_sides=sides,
                tour_polygon_radius_m=radius,
            )
            for seed in range(arguments.start, arguments.start + arguments.count)
        ]
        for row in rows:
            row["tour_polygon_sides"] = sides
            row["tour_polygon_radius_m"] = radius
        result = {
            "tour_polygon_sides": sides,
            "tour_polygon_radius_m": radius,
            "coverage_worst_distance_m": plan.coverage.analytic_max_distance_m,
            "base_route_length_m": plan.route_length_m,
            "estimated_base_time_s": plan.estimated_base_time_s,
            **summarize(rows),
        }
        configurations.append(result)
        all_rows.extend(rows)
        print(json.dumps(result, ensure_ascii=False), flush=True)

    configurations.sort(
        key=lambda item: (
            int(item["errors"]),
            -int(item["resolved"]),
            float(item["mean_time_s"]),
        )
    )
    document = {
        "seed_start": arguments.start,
        "seed_count": arguments.count,
        "stage": arguments.stage,
        "ranking": configurations,
        "runs": all_rows,
    }
    output = (
        ROOT / "res" / "tables" / "problem3"
        / f"route_aligned_polygon_{arguments.stage}_tuning_{arguments.start}_{arguments.count}.json"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(document, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"output={output}")


if __name__ == "__main__":
    main()
