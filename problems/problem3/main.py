"""问题 3 的端口运行入口、结果落盘和正式三次测试编排。

本模块只通过 :class:`SimulatorClient` 调用附件规定的四个端点。连接失败或动作异常
会直接终止当前局，不会切换到本地真值、伪响应，也不会自动重发可能已执行的动作。
"""

from __future__ import annotations

import csv
import json
import time
from dataclasses import asdict, fields, replace
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml

from problems.problem3.config import (
    ACTION_LOG_DIR,
    FORMAL_RESULTS_TABLE,
    ROBUST_STRATEGY,
    STRATEGIES,
    SUMMARY_DIR,
    TRAJECTORY_FIGURE_DIR,
    Problem3Settings,
)
from problems.problem3.shared import (
    JsonlRunLogger,
    Problem3Executor,
    Problem3State,
    summarize_state,
)
from problems.problem3.strategies import (
    BeliefMPCStrategy, RobustPolygonRollingStrategy,
    CooperativeBearingTourStrategy, IntegratedBearingTourStrategy,
)
from radio_locator.client import AmbiguousActionError, SimulatorClient


def load_settings(path: Path | None, overrides: dict[str, Any]) -> Problem3Settings:
    """读取 YAML 算法参数，并应用命令行中显式给出的覆盖值。"""

    values: dict[str, Any] = {}
    if path is not None:
        with path.open("r", encoding="utf-8") as file:
            document = yaml.safe_load(file) or {}
        if not isinstance(document, dict):
            raise ValueError("problem3 config root must be a mapping")
        values.update(document.get("algorithm", document))
    values.update({key: value for key, value in overrides.items() if value is not None})
    allowed = {item.name for item in fields(Problem3Settings)}
    unknown = sorted(set(values) - allowed)
    if unknown:
        raise ValueError(f"unknown problem3 configuration keys: {unknown}")
    tuple_fields = {
        "channels",
        "polygon_side_candidates",
        "polygon_radius_candidates_m",
        "polygon_rotation_candidates_deg",
        "scan_origin_candidates",
        "start_position",
    }
    for key in tuple_fields & values.keys():
        values[key] = tuple(values[key])
    settings = Problem3Settings(**values)
    _validate_settings(settings)
    return settings


def _validate_settings(settings: Problem3Settings) -> None:
    """在连接模拟器前检查会改变动作含义的配置。"""

    if settings.tour_endgame_mode not in {"legacy", "exact", "probe"}:
        raise ValueError("tour_endgame_mode must be legacy, exact or probe")
    if settings.tour_channel_order not in {"legacy", "alternating"}:
        raise ValueError("tour_channel_order must be legacy or alternating")
    if settings.tour_polygon_sides < 3 or settings.tour_polygon_radius_m <= 0:
        raise ValueError("tour polygon requires at least three sides and positive radius")
    if not 0 < settings.tour_target_radius_m < float("inf"):
        raise ValueError("tour target radius must be finite and positive")
    if settings.guaranteed_radius_m != 1_000.0:
        raise ValueError("guaranteed_radius_m is fixed by the problem at 1000 m")
    if settings.start_position != (0.0, 0.0) or settings.start_channel != 1:
        raise ValueError("problem3 must start at origin with receiver channel 1")
    if not 20 <= settings.candidate_action_limit <= 50:
        raise ValueError("candidate_action_limit must be between 20 and 50")
    if not 2 <= settings.horizon <= 4:
        raise ValueError("horizon must be between 2 and 4")
    if not 10 <= settings.beam_width <= 30:
        raise ValueError("beam_width must be between 10 and 30")
    if settings.coverage_grid_step_m <= 0 or settings.coverage_validation_step_m <= 0:
        raise ValueError("coverage grid steps must be positive")
    if (
        settings.robust_polygon_sides < 3
        or settings.robust_polygon_radius_m <= 0
    ):
        raise ValueError("robust polygon must have at least three sides and positive radius")
    if not 0.0 < settings.robust_clear_radius_m <= settings.clearance_radius_m:
        raise ValueError("robust_clear_radius_m must lie in (0, clearance_radius_m]")
    fallback_cover_radius = settings.fallback_grid_step_m * 2**0.5 / 2.0
    if fallback_cover_radius >= settings.clearance_radius_m:
        raise ValueError("fallback grid cells are too large for the 20 m clearance radius")


def _write_json(path: Path, data: dict[str, Any]) -> None:
    """以 UTF-8 缩进 JSON 保存人工可审阅结果。"""

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def _write_plan_search(path: Path, plans: tuple[Any, ...]) -> None:
    """保存所有正多边形候选，便于核对参数搜索没有只试七边形。"""

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(
            [
                "sides", "radius_m", "rotation_deg", "scan_origin",
                "route_length_m", "measure_count", "switch_count",
                "estimated_base_time_s", "analytic_max_distance_m",
                "numerical_max_distance_m", "coverage_ratio", "worst_x",
                "worst_y", "achieved_margin_m", "valid",
            ]
        )
        for plan in plans:
            metric = plan.coverage
            writer.writerow(
                [
                    plan.sides, plan.radius_m, plan.rotation_deg, plan.scan_origin,
                    plan.route_length_m, plan.measure_count, plan.switch_count,
                    plan.estimated_base_time_s, metric.analytic_max_distance_m,
                    metric.numerical_max_distance_m, metric.coverage_ratio,
                    *metric.worst_point, metric.achieved_margin_m, metric.valid,
                ]
            )


def _append_formal_result(path: Path, summary: dict[str, Any]) -> None:
    """按附件正式测试表关键信息追加一行，不改名模拟器原始日志。"""

    path.parent.mkdir(parents=True, exist_ok=True)
    exists = path.exists()
    columns = [
        "run_index", "case_code", "strategy", "seed", "cleared_count",
        "clearance_ratio", "total_virtual_time_s", "program_wall_time_s",
        "action_log", "summary_file", "official_log_original_names",
        "trajectory_figure", "clearance_detail_figure",
    ]
    with path.open("a", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=columns)
        if not exists:
            writer.writeheader()
        row = {key: summary.get(key) for key in columns}
        row["official_log_original_names"] = ";".join(
            summary["official_log_original_names"]
        )
        writer.writerow(row)


def run_once(
    *, strategy_name: str, host: str, port: int, robot_id: str,
    timeout_s: float, settings: Problem3Settings, run_index: int,
    case_code: str | None, official_log_names: list[str], formal: bool,
    truth_file: Path | None = None,
) -> dict[str, Any]:
    """连接一局模拟器、运行策略并保存动作日志和汇总。"""

    if strategy_name not in STRATEGIES:
        raise ValueError(f"unknown strategy: {strategy_name}")
    if not robot_id:
        raise ValueError("robot_id is required; set --robot-id or CUMCM_ROBOT_ID")
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    stem = f"{strategy_name}_seed{settings.seed}_run{run_index}_{stamp}"
    action_log = ACTION_LOG_DIR / f"{stem}.jsonl"
    summary_path = SUMMARY_DIR / f"{stem}.json"
    plan_path = SUMMARY_DIR / f"{stem}_polygon_search.csv"
    logger = JsonlRunLogger(action_log)
    state = Problem3State(settings)
    client = SimulatorClient(
        base_url=f"http://{host}:{port}", robot_id=robot_id, timeout_s=timeout_s,
        request_prefix=f"p3-{strategy_name}-s{settings.seed}-r{run_index}",
    )
    executor = Problem3Executor(client, state, logger, strategy_name)
    strategy_types = {
        "robust_polygon_rolling": RobustPolygonRollingStrategy,
        "belief_mpc": BeliefMPCStrategy,
        "cooperative_bearing_tour": CooperativeBearingTourStrategy,
        "integrated_bearing_tour": IntegratedBearingTourStrategy,
    }
    strategy = strategy_types[strategy_name](settings)
    start = time.monotonic()
    try:
        executor.enter()
        result = strategy.run(executor)
        executor.exit()
        plotting_error: str | None = None
        try:
            from problems.problem3.plotting import plot_run_replay

            figure_paths = plot_run_replay(
                action_log,
                TRAJECTORY_FIGURE_DIR / f"{stem}_trajectory.png",
                TRAJECTORY_FIGURE_DIR / f"{stem}_clearance_details.png",
                truth_file,
            )
        except Exception as error:
            # 模拟动作已经完整结束，绘图异常不能把一次有效正式测试伪装成运行失败。
            figure_paths = {
                "trajectory_figure": None,
                "clearance_detail_figure": None,
            }
            plotting_error = f"{type(error).__name__}: {error}"
        summary = summarize_state(
            result.state, strategy_name, result.selected_plan.as_dict(),
            official_log_names,
        )
        _write_plan_search(plan_path, result.compared_plans)
        summary.update(
            {
                "run_index": run_index, "case_code": case_code,
                "seed": settings.seed, "host": host, "port": port,
                "program_wall_time_s": time.monotonic() - start,
                "action_log": str(action_log), "summary_file": str(summary_path),
                "polygon_search_file": str(plan_path), "settings": asdict(settings),
                "postrun_truth_file": str(truth_file) if truth_file is not None else None,
                "plotting_error": plotting_error,
                **figure_paths,
            }
        )
        _write_json(summary_path, summary)
        if formal:
            _append_formal_result(FORMAL_RESULTS_TABLE, summary)
        return summary
    except AmbiguousActionError:
        # 请求是否已执行未知，绝不能继续发新动作或自动exit。
        raise
    finally:
        logger.close()


def run_problem3(
    *, strategy: str, host: str, port: int, robot_id: str, timeout_s: float,
    settings: Problem3Settings, runs: int = 1,
    case_codes: list[str] | None = None, official_logs: list[Path] | None = None,
    truth_files: list[Path] | None = None, formal: bool = False,
    next_run_wait_s: float = 0.0,
) -> list[dict[str, Any]]:
    """连续执行1或3局；正式模式强制3局并保留原始日志文件名。"""

    if formal and runs != 3:
        raise ValueError("formal mode requires exactly three runs")
    if runs < 1:
        raise ValueError("runs must be positive")
    codes = case_codes or []
    logs = official_logs or []
    truths = truth_files or []
    summaries: list[dict[str, Any]] = []
    for index in range(1, runs + 1):
        if index > 1 and next_run_wait_s > 0:
            # 下一局由附件GUI启动；这里只等待端口重新可用，不执行模拟动作。
            deadline = time.monotonic() + next_run_wait_s
            probe = SimulatorClient(
                base_url=f"http://{host}:{port}", robot_id=robot_id,
                timeout_s=timeout_s,
            )
            while True:
                try:
                    probe.check_connection()
                    break
                except Exception:
                    if time.monotonic() >= deadline:
                        raise TimeoutError("next simulator run did not become available")
                    time.sleep(min(1.0, max(deadline - time.monotonic(), 0.0)))
        summary = run_once(
            strategy_name=strategy, host=host, port=port, robot_id=robot_id,
            timeout_s=timeout_s,
            settings=replace(settings, seed=settings.seed + index - 1),
            run_index=index,
            case_code=codes[index - 1] if index <= len(codes) else None,
            official_log_names=[logs[index - 1].name] if index <= len(logs) else [],
            formal=formal,
            truth_file=truths[index - 1] if index <= len(truths) else None,
        )
        summaries.append(summary)
    return summaries


__all__ = ["load_settings", "run_once", "run_problem3"]
