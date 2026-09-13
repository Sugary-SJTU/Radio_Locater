"""同种子比较问题四保证策略变体，并生成实测耗时占比图。"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]

from config.constants import (
    CHANNEL_SWITCH_TIME_S,
    CLEARANCE_FAILURE_TIME_S,
    CLEARANCE_SUCCESS_TIME_S,
    MEASUREMENT_TIME_S,
    ROBOT_SPEED_MPS,
)
from config.paths import PROBLEM4_FIGURES_DIR
from problems.problem3.shared import Problem3Executor
from problems.problem4.config import Problem4Settings
from problems.problem4.strategies import (
    GuaranteedDirectionalLatticeStrategy,
    LegacyOuterProbeFastStrategy,
    OptimizedGuaranteedLatticeStrategy,
    Problem4State,
)
from radio_locator.client import ActionExchange
from radio_locator.local_simulator import SimulatorEngine, SimulatorScenario
from scripts.plot_strategy_time_comparison import TimeBreakdown, _plot_percent_bars


class EngineClient:
    """只暴露正式四端点行为，不向策略提供场景真值。"""

    def __init__(self, seed: int) -> None:
        self.engine = SimulatorEngine(
            SimulatorScenario.generate(4, seed), "benchmark", monotonic_s=lambda: 0.0
        )
        self.counter = 0

    def check_connection(self) -> None:
        return None

    def _call(self, path: str, position=None, channel=None) -> ActionExchange:
        self.counter += 1
        payload = {
            "arena_id": "default",
            "robot_id": "benchmark",
            "request_id": str(self.counter),
        }
        if position is not None:
            payload.update(
                position={"x": position[0], "y": position[1]}, channel=channel
            )
        status, response = self.engine.process(path, payload)
        if status != 200 or response.get("accepted") is not True:
            raise RuntimeError(response)
        return ActionExchange(path, payload, status, response)

    def enter(self):
        return self._call("/enter")

    def measure(self, position, channel):
        return self._call("/measure", position, channel)

    def clear(self, position, channel):
        return self._call("/clear", position, channel)


class NullLogger:
    def write(self, _record: dict) -> None:
        return None


def run_case(
    settings: Problem4Settings,
    seed: int,
    strategy_type=GuaranteedDirectionalLatticeStrategy,
) -> dict:
    settings = replace(settings, seed=seed)
    state = Problem4State(settings)
    client = EngineClient(seed)
    executor = Problem3Executor(client, state, NullLogger(), "p4-benchmark")
    executor.enter()
    result = strategy_type(settings).run(executor)
    counters = state.counters
    breakdown = TimeBreakdown(
        counters.movement_distance_m / ROBOT_SPEED_MPS,
        counters.measure_count * MEASUREMENT_TIME_S,
        counters.switch_count * CHANNEL_SWITCH_TIME_S,
        counters.clear_success_count * CLEARANCE_SUCCESS_TIME_S
        + counters.clear_failure_count * CLEARANCE_FAILURE_TIME_S,
    )
    directional = sum(
        source.source_type == "directional" for source in client.engine.scenario.sources
    )
    return {
        "seed": seed,
        "complete": state.all_resolved(),
        "all_sources_cleared": counters.clear_success_count
        == len(client.engine.scenario.sources),
        "source_clearance_ratio": counters.clear_success_count
        / len(client.engine.scenario.sources),
        "source_count": len(client.engine.scenario.sources),
        "directional_source_count": directional,
        "omnidirectional_source_count": len(client.engine.scenario.sources)
        - directional,
        "total_time_s": state.virtual_time_s,
        "station_count": result.plan["station_count"],
        "movement_distance_m": counters.movement_distance_m,
        **asdict(counters),
        "time_breakdown": breakdown.as_dict(),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed-start", type=int, default=0)
    parser.add_argument("--seed-count", type=int, default=10)
    parser.add_argument(
        "--only",
        choices=(
            "shifted_triangular_lattice",
            "concentric_ring_no_insertion",
            "concentric_ring_joint",
            "optimized_guaranteed_lattice",
            "legacy_outer_probe_fast",
        ),
        help="只审计指定变体，适合扩大随机种子样本",
    )
    args = parser.parse_args()
    seeds = range(args.seed_start, args.seed_start + args.seed_count)
    base = Problem4Settings()
    variants = {
        "shifted_triangular_lattice": (
            GuaranteedDirectionalLatticeStrategy,
            replace(
                base,
                directional_mesh_layout="triangular_lattice",
                directional_grid_offset_y_m=base.directional_grid_spacing_m
                * math.sqrt(3.0)
                / 4.0,
                route_clear_insertion_limit_m=300.0,
            ),
        ),
        "concentric_ring_no_insertion": (
            GuaranteedDirectionalLatticeStrategy,
            replace(
                base,
                directional_mesh_layout="concentric_ring",
                route_clear_insertion_limit_m=0.0,
            ),
        ),
        "concentric_ring_joint": (
            GuaranteedDirectionalLatticeStrategy,
            replace(
                base,
                directional_mesh_layout="concentric_ring",
                route_clear_insertion_limit_m=300.0,
            ),
        ),
        "optimized_guaranteed_lattice": (OptimizedGuaranteedLatticeStrategy, base),
        "legacy_outer_probe_fast": (LegacyOuterProbeFastStrategy, base),
    }
    if args.only:
        variants = {args.only: variants[args.only]}
    document = {
        "data_kind": "local_simulator_measurement",
        "problem": 4,
        "seeds": list(seeds),
        "priority": "completion rate first, mean virtual time second",
        "variants": {},
    }
    mean_breakdowns: list[TimeBreakdown] = []
    for name, (strategy_type, settings) in variants.items():
        samples = [run_case(settings, seed, strategy_type) for seed in seeds]
        mean = {
            "completion_rate": float(np.mean([row["complete"] for row in samples])),
            "all_sources_cleared_rate": float(
                np.mean([row["all_sources_cleared"] for row in samples])
            ),
            "mean_source_clearance_ratio": float(
                np.mean([row["source_clearance_ratio"] for row in samples])
            ),
            "total_time_s": float(np.mean([row["total_time_s"] for row in samples])),
            "median_time_s": float(np.median([row["total_time_s"] for row in samples])),
            "p90_time_s": float(
                np.percentile([row["total_time_s"] for row in samples], 90)
            ),
            "movement_distance_m": float(
                np.mean([row["movement_distance_m"] for row in samples])
            ),
            "measure_count": float(np.mean([row["measure_count"] for row in samples])),
            "clear_failure_count": float(
                np.mean([row["clear_failure_count"] for row in samples])
            ),
        }
        fields = TimeBreakdown.__dataclass_fields__
        mean_breakdown = TimeBreakdown(
            *(
                float(np.mean([row["time_breakdown"][field] for row in samples]))
                for field in fields
            )
        )
        mean["time_breakdown"] = mean_breakdown.as_dict()
        mean_breakdowns.append(mean_breakdown)
        document["variants"][name] = {
            "settings": asdict(settings),
            "mean": mean,
            "samples": samples,
        }
        print(name, json.dumps(mean, ensure_ascii=False), flush=True)

    output_name = (
        f"{args.only}_audit_{args.seed_start}_{args.seed_count}.json"
        if args.only
        else "problem4_strategy_comparison.json"
    )
    output = ROOT / "res/tables/problem4" / output_name
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(document, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    if not args.only:
        _plot_percent_bars(
            [
                "平移三角格",
                "同心环\n无顺路清除",
                "同心环\n联合插入",
                "优化保证格\n联合路线",
                "父项目\n高速有限定位",
                "外侧补测\n安全兜底",
            ],
            mean_breakdowns,
            PROBLEM4_FIGURES_DIR / "problem4_strategy_time_percent.png",
        )


if __name__ == "__main__":
    main()
