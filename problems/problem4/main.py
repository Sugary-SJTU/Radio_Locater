"""问题四混合全向/定向源策略的正式运行入口。"""

from __future__ import annotations

import json
import time
from dataclasses import asdict, replace
from datetime import datetime
from pathlib import Path
from typing import Any

from config.paths import PROBLEM4_TRAJECTORY_FIGURES_DIR, paired_pdf_path
from problems.problem3.main import (
    format_run_report,
    load_truth_statistics,
    summarize_action_times,
)
from problems.problem3.shared import JsonlRunLogger, Problem3Executor, summarize_state
from problems.problem4.config import (
    ACTION_LOG_DIR,
    GUARANTEED_DIRECTIONAL_LATTICE,
    LEGACY_OUTER_PROBE_FAST,
    OPTIMIZED_GUARANTEED_LATTICE,
    PARENT_FAST_STRATEGY,
    SUMMARY_DIR,
    Problem4Settings,
)
from problems.problem4.strategies import (
    GuaranteedDirectionalLatticeStrategy,
    LegacyOuterProbeFastStrategy,
    OptimizedGuaranteedLatticeStrategy,
    ParentProblem4FastStrategy,
    Problem4State,
)
from radio_locator.client import SimulatorClient


def _write_json(path: Path, document: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document, ensure_ascii=False, indent=2), encoding="utf-8")


def run_once(
    *, host: str, port: int, robot_id: str, timeout_s: float,
    settings: Problem4Settings, run_index: int = 1,
    strategy_name: str = GUARANTEED_DIRECTIONAL_LATTICE,
    truth_file: Path | None = None,
) -> dict[str, Any]:
    """执行一局问题四，保存动作日志、统计、轨迹与耗时图。"""

    if not robot_id:
        raise ValueError("robot_id is required; set --robot-id or CUMCM_ROBOT_ID")
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    strategy_types = {
        GUARANTEED_DIRECTIONAL_LATTICE: GuaranteedDirectionalLatticeStrategy,
        OPTIMIZED_GUARANTEED_LATTICE: OptimizedGuaranteedLatticeStrategy,
        PARENT_FAST_STRATEGY: ParentProblem4FastStrategy,
        LEGACY_OUTER_PROBE_FAST: LegacyOuterProbeFastStrategy,
    }
    if strategy_name not in strategy_types:
        raise ValueError(f"unknown problem4 strategy: {strategy_name}")
    stem = f"{strategy_name}_seed{settings.seed}_run{run_index}_{stamp}"
    action_log = ACTION_LOG_DIR / f"{stem}.jsonl"
    summary_path = SUMMARY_DIR / f"{stem}.json"
    logger = JsonlRunLogger(action_log)
    state = Problem4State(settings)
    client = SimulatorClient(
        base_url=f"http://{host}:{port}", robot_id=robot_id, timeout_s=timeout_s,
        request_prefix=f"p4-{strategy_name}-s{settings.seed}-r{run_index}",
    )
    executor = Problem3Executor(client, state, logger, strategy_name)
    start = time.monotonic()
    try:
        executor.enter()
        result = strategy_types[strategy_name](settings).run(executor)
        executor.exit()
    finally:
        logger.close()

    summary = summarize_state(state, strategy_name, result.plan)
    truth = load_truth_statistics(truth_file)
    truth["problem_number"] = 4
    timing = summarize_action_times(action_log, float(summary["total_virtual_time_s"]))
    source_count = truth["source_count"]
    cleared_count = int(summary["cleared_count"])
    if source_count:
        summary["clearance_ratio"] = cleared_count / source_count
        summary["clearance_ratio_denominator"] = "postrun truth source count"
    summary.update(truth)
    summary.update(
        {
            "run_index": run_index,
            "seed": settings.seed,
            "host": host,
            "port": port,
            "program_wall_time_s": time.monotonic() - start,
            "time_breakdown_s": timing,
            "movement_time_s": timing["movement"],
            "channel_switch_time_s": timing["channel_switch"],
            "detection_time_s": timing["detection"],
            "clearance_time_s": timing["clearance"],
            "other_time_s": timing["other"],
            "average_time_per_source_s": (
                float(summary["total_virtual_time_s"]) / source_count if source_count else None
            ),
            "average_time_per_cleared_source_s": (
                float(summary["total_virtual_time_s"]) / cleared_count if cleared_count else None
            ),
            "action_log": str(action_log),
            "summary_file": str(summary_path),
            "settings": asdict(settings),
            "postrun_truth_file": str(truth_file) if truth_file else None,
        }
    )
    figure_directory = PROBLEM4_TRAJECTORY_FIGURES_DIR / stem
    pdf_directory = paired_pdf_path(figure_directory / "placeholder.png").parent
    try:
        from problems.problem3.plotting import plot_run_replay

        figures = plot_run_replay(
            action_log, figure_directory / "trajectory.png",
            figure_directory / "clearance_details.png", truth_file,
            time_breakdown_s=timing,
            timing_output=figure_directory / "time_breakdown.png",
        )
        plotting_error = None
    except Exception as error:
        figures = {}
        plotting_error = f"{type(error).__name__}: {error}"
    summary.update(
        {
            "figure_directory_png": str(figure_directory),
            "figure_directory_pdf": str(pdf_directory),
            "plotting_error": plotting_error,
            **figures,
        }
    )
    _write_json(summary_path, summary)
    return summary


def run_problem4(
    *, host: str, port: int, robot_id: str, timeout_s: float,
    settings: Problem4Settings, runs: int = 1,
    strategy: str = GUARANTEED_DIRECTIONAL_LATTICE,
    truth_files: list[Path] | None = None,
) -> list[dict[str, Any]]:
    """连续运行若干局；每局种子递增，且不在策略执行期间读取真值。"""

    if runs <= 0:
        raise ValueError("runs must be positive")
    truths = truth_files or []
    return [
        run_once(
            host=host, port=port, robot_id=robot_id, timeout_s=timeout_s,
            settings=replace(settings, seed=settings.seed + index), run_index=index + 1,
            strategy_name=strategy,
            truth_file=truths[index] if index < len(truths) else None,
        )
        for index in range(runs)
    ]


__all__ = ["format_run_report", "run_once", "run_problem4"]
