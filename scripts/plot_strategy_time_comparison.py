"""绘制问题三实测与问题四方案阶段的动作耗时结构对比图。

问题三统计来自同一批本地模拟器随机案例的真实动作计数；问题四目前没有可运行的
完整自动策略，故只比较几何扫描方案在最坏情形下的动作成本模型。两类数据分别落盘，
避免把方案估算误作仿真结果。
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]

from radio_locator.runtime import prepare_matplotlib_config

prepare_matplotlib_config()

import matplotlib.pyplot as plt
import numpy as np

from config.constants import (
    CHANNEL_SWITCH_TIME_S,
    CLEARANCE_FAILURE_TIME_S,
    CLEARANCE_SUCCESS_TIME_S,
    MEASUREMENT_TIME_S,
    ROBOT_SPEED_MPS,
    SOURCE_COUNT_MAX,
)
from config.paths import PROBLEM3_FIGURES_DIR, PROBLEM4_FIGURES_DIR, paired_pdf_path
from problems.problem1.plotting import configure_chinese_font, style_figure_for_paper
from problems.problem3.config import Problem3Settings
from problems.problem3.coverage import (
    optimize_remaining_route_multistart,
    regular_polygon_points,
    route_length,
)
from problems.problem3.shared import Problem3Executor, Problem3State
from problems.problem3.strategies import (
    BeliefMPCStrategy,
    IntegratedBearingTourStrategy,
    RobustPolygonRollingStrategy,
)
from scripts.benchmark_problem3 import EngineClient


COMPONENTS = ("移动", "检测", "换频", "清除/其他")
COLORS = ("#4C78A8", "#F2A541", "#59A14F", "#B07AA1")


class _NullLogger:
    """基准图只需最终计数，不在 ``res/logs`` 写入逐动作日志。"""

    def write(self, _record: dict) -> None:
        return None


@dataclass(frozen=True)
class TimeBreakdown:
    movement_s: float
    measurement_s: float
    switching_s: float
    clearance_other_s: float

    @property
    def total_s(self) -> float:
        return sum(asdict(self).values())

    def percentages(self) -> list[float]:
        total = self.total_s
        return [100.0 * value / total for value in asdict(self).values()]

    def as_dict(self) -> dict[str, float]:
        return {**asdict(self), "total_s": self.total_s, "percentages": self.percentages()}


def _breakdown_from_state(state: Problem3State) -> TimeBreakdown:
    counters = state.counters
    movement = counters.movement_distance_m / ROBOT_SPEED_MPS
    measurement = counters.measure_count * MEASUREMENT_TIME_S
    switching = counters.switch_count * CHANNEL_SWITCH_TIME_S
    clearance = (
        counters.clear_success_count * CLEARANCE_SUCCESS_TIME_S
        + counters.clear_failure_count * CLEARANCE_FAILURE_TIME_S
    )
    # 保留任何未来新增动作的虚拟耗时，保证结构图与模拟器总时间严格可核对。
    other = max(0.0, state.virtual_time_s - movement - measurement - switching - clearance)
    return TimeBreakdown(movement, measurement, switching, clearance + other)


def run_problem3(strategy: type, seed: int) -> TimeBreakdown:
    """在固定种子本地场景中执行一个策略，返回实际动作耗时分解。"""

    settings = Problem3Settings(seed=seed)
    client = EngineClient(seed)
    state = Problem3State(settings)
    executor = Problem3Executor(client, state, _NullLogger(), strategy.name)
    executor.enter()
    strategy(settings).run(executor)
    if not state.all_resolved():
        raise RuntimeError(f"{strategy.name} seed={seed} 未完成全部频道")
    return _breakdown_from_state(state)


def _mean_breakdown(samples: list[TimeBreakdown]) -> TimeBreakdown:
    return TimeBreakdown(*(
        float(np.mean([getattr(sample, field) for sample in samples]))
        for field in TimeBreakdown.__dataclass_fields__
    ))


def collect_problem3(seeds: range, progress: bool = False) -> dict:
    strategies: dict[str, type] = {
        "robust_polygon_rolling": RobustPolygonRollingStrategy,
        "belief_mpc": BeliefMPCStrategy,
        "integrated_bearing_tour": IntegratedBearingTourStrategy,
    }
    records: dict[str, dict] = {}
    for name, strategy in strategies.items():
        samples = [run_problem3(strategy, seed) for seed in seeds]
        mean = _mean_breakdown(samples)
        records[name] = {
            "mean": mean.as_dict(),
            "samples": [sample.as_dict() for sample in samples],
        }
        if progress:
            print(f"{name}: {mean.total_s:.1f}s", flush=True)
    return {
        "data_kind": "local_simulator_measurement",
        "scenario": "problem3",
        "seeds": list(seeds),
        "strategy_results": records,
    }


def _lattice_points(spacing_m: float = 1_000.0, radius_m: float = 2_800.0) -> list[tuple[float, float]]:
    """生成覆盖半径1800m、有效接收半径1000m的三角网格候选测站。"""

    vertical = spacing_m * np.sqrt(3.0) / 2.0
    points: list[tuple[float, float]] = []
    for row, y in enumerate(np.arange(-radius_m, radius_m + 1e-9, vertical)):
        offset = 0.5 * spacing_m if row % 2 else 0.0
        for x in np.arange(-radius_m, radius_m + 1e-9, spacing_m):
            point = (float(x + offset), float(y))
            if np.hypot(*point) <= radius_m + 1e-9:
                points.append(point)
    # 原点必须是首个检测点；其余点保持唯一。
    unique = {(round(x, 8), round(y, 8)): (x, y) for x, y in points}
    unique[(0.0, 0.0)] = (0.0, 0.0)
    return list(unique.values())


def _model_breakdown(points: list[tuple[float, float]], label: str) -> dict:
    route = optimize_remaining_route_multistart((0.0, 0.0), points)
    distance = route_length(tuple(route), (0.0, 0.0))
    station_count = len(points)
    measure_count = station_count * 20
    switch_count = station_count * 19
    breakdown = TimeBreakdown(
        movement_s=distance / ROBOT_SPEED_MPS,
        measurement_s=measure_count * MEASUREMENT_TIME_S,
        switching_s=switch_count * CHANNEL_SWITCH_TIME_S,
        clearance_other_s=SOURCE_COUNT_MAX * CLEARANCE_SUCCESS_TIME_S,
    )
    return {
        "label": label,
        "station_count": station_count,
        "route_length_m": distance,
        "measure_count": measure_count,
        "switch_count": switch_count,
        "assumed_clear_success_count": SOURCE_COUNT_MAX,
        "breakdown": breakdown.as_dict(),
    }


def model_problem4() -> dict:
    """构建问题四尚未实现策略的、可复核的最坏情形动作成本模型。"""

    core = list(regular_polygon_points(7, 1_050.0, scan_origin=True))
    outer = list(regular_polygon_points(6, 2_250.0, rotation_deg=30.0))
    lattice = _lattice_points()
    candidates = [
        _model_breakdown(core, "七边形直接复用\n（无定向保证）"),
        _model_breakdown(core + outer, "七边形+外侧补测\n（经验方案）"),
        _model_breakdown(lattice, "三角网格扫描\n（保证方案）"),
    ]
    return {
        "data_kind": "analytical_action_cost_estimate_not_simulation",
        "scenario": "problem4",
        "assumptions": {
            "scan": "每个测站扫描20个频道；每站按19次换频计。",
            "clearance": "按题设最多16个干扰源均清除成功计。",
            "movement": "起点为原点；采用多起点开放路径启发式估算，不含返航。",
            "scope": "仅比较候选发现路线的动作成本，不代表检出率或清除成功率。",
        },
        "candidate_strategies": candidates,
    }


def _plot_percent_bars(labels: list[str], breakdowns: list[TimeBreakdown], output: Path) -> None:
    configure_chinese_font()
    figure, axis = plt.subplots(figsize=(8.4, 5.2))
    x = np.arange(len(labels))
    bottom = np.zeros(len(labels))
    matrix = np.asarray([item.percentages() for item in breakdowns])
    for index, (component, color) in enumerate(zip(COMPONENTS, COLORS, strict=True)):
        axis.bar(
            x, matrix[:, index], bottom=bottom, width=0.64, label=component,
            color=color, edgecolor="black", linewidth=0.85,
        )
        bottom += matrix[:, index]
    # 大段直接标在柱内；狭窄段合并为柱顶的紧凑注记，避免细小区段文字重叠。
    for bar_index, percentages in enumerate(matrix):
        cumulative = 0.0
        small_labels: list[str] = []
        short_names = ("移", "测", "换", "清")
        for component_index, percentage in enumerate(percentages):
            center = cumulative + percentage / 2.0
            label = f"{percentage:.1f}%"
            if percentage >= 5.0:
                axis.text(
                    x[bar_index], center, label, ha="center", va="center",
                    fontsize=9.0, fontweight="semibold",
                    color="white" if component_index in {0, 2, 3} else "#111111",
                    zorder=5,
                )
            else:
                small_labels.append(f"{short_names[component_index]} {label}")
            cumulative += percentage
        if small_labels:
            axis.text(
                x[bar_index], 102.0, " · ".join(small_labels),
                ha="center", va="center", fontsize=7.6, color="#222222", zorder=6,
            )
    axis.set_ylim(0.0, 104.0)
    axis.set_ylabel("耗时占比 / %")
    axis.set_xticks(x, labels)
    axis.set_yticks(np.arange(0, 101, 20))
    axis.grid(axis="y", color="#C9C9C9", linewidth=0.65, alpha=0.75)
    axis.set_axisbelow(True)
    for spine in axis.spines.values():
        spine.set_color("black")
        spine.set_linewidth(1.0)
    # 图例放在图框内上方，避免较长的第四题方案名称在紧凑裁切时挤掉图例。
    figure.subplots_adjust(top=0.80)
    figure.legend(
        *axis.get_legend_handles_labels(), ncol=4, loc="upper center",
        bbox_to_anchor=(0.5, 0.975), frameon=False,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    style_figure_for_paper(figure)
    figure.savefig(output, dpi=300, bbox_inches="tight", facecolor="white", edgecolor="black")
    pdf = paired_pdf_path(output)
    pdf.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(pdf, bbox_inches="tight", facecolor="white", edgecolor="black")
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed-start", type=int, default=0)
    parser.add_argument("--seed-count", type=int, default=20)
    parser.add_argument("--progress", action="store_true", help="逐策略输出进度")
    parser.add_argument("--render-only", action="store_true", help="只按现有统计表重绘图片")
    args = parser.parse_args()
    if args.seed_count <= 0:
        raise ValueError("seed-count must be positive")

    p3_table = ROOT / "res/tables/problem3/strategy_time_percent_comparison.json"
    p4_table = ROOT / "res/tables/problem4/candidate_strategy_time_percent_model.json"
    if args.render_only:
        problem3 = json.loads(p3_table.read_text(encoding="utf-8"))
        problem4 = json.loads(p4_table.read_text(encoding="utf-8"))
    else:
        seeds = range(args.seed_start, args.seed_start + args.seed_count)
        problem3 = collect_problem3(seeds, progress=args.progress)
        problem4 = model_problem4()
        p3_table.parent.mkdir(parents=True, exist_ok=True)
        p4_table.parent.mkdir(parents=True, exist_ok=True)
        p3_table.write_text(json.dumps(problem3, ensure_ascii=False, indent=2), encoding="utf-8")
        p4_table.write_text(json.dumps(problem4, ensure_ascii=False, indent=2), encoding="utf-8")

    p3_records = problem3["strategy_results"]
    _plot_percent_bars(
        ["鲁棒多边形\n滚动", "信念MPC", "联合示向\n巡回"],
        [
            TimeBreakdown(**{key: p3_records[name]["mean"][key] for key in TimeBreakdown.__dataclass_fields__})
            for name in p3_records
        ],
        PROBLEM3_FIGURES_DIR / "problem3_strategy_time_percent.png",
    )
    _plot_percent_bars(
        [item["label"] for item in problem4["candidate_strategies"]],
        [
            TimeBreakdown(**{key: item["breakdown"][key] for key in TimeBreakdown.__dataclass_fields__})
            for item in problem4["candidate_strategies"]
        ],
        PROBLEM4_FIGURES_DIR / "problem4_candidate_strategy_time_percent.png",
    )


if __name__ == "__main__":
    main()
