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

from config.paths import (
    PROBLEM3_TRAJECTORY_FIGURES_DIR,
    PROBLEM4_TRAJECTORY_FIGURES_DIR,
    paired_pdf_path,
)
from problems.problem3.config import (
    ACTION_LOG_DIR,
    DISTANCE_TOUR_STRATEGY,
    DYNAMIC_COVERAGE_TOUR_STRATEGY,
    FORMAL_RESULTS_TABLE,
    MPC_STRATEGY,
    ROBUST_STRATEGY,
    SAFE_CLEAR_TOUR_STRATEGY,
    STRATEGIES,
    SUMMARY_DIR,
    TOUR_STRATEGY,
    Problem3Settings,
)
from problems.problem3.shared import (
    JsonlRunLogger,
    Problem3Executor,
    Problem3State,
    summarize_state,
)
from problems.problem3.strategies import (
    BeliefMPCStrategy,
    DistanceOptimizedBearingTourStrategy,
    DynamicCoverageRouteAlignedTourStrategy,
    IntegratedBearingTourStrategy,
    RobustPolygonRollingStrategy,
    SafeClearRouteAlignedTourStrategy,
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
        raise ValueError(
            "tour polygon requires at least three sides and positive radius"
        )
    if (
        settings.route_aligned_polygon_sides < 3
        or settings.route_aligned_polygon_radius_m <= 0
    ):
        raise ValueError(
            "route-aligned polygon requires at least three sides and positive radius"
        )
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
    if settings.robust_polygon_sides < 3 or settings.robust_polygon_radius_m <= 0:
        raise ValueError(
            "robust polygon must have at least three sides and positive radius"
        )
    if not 0.0 < settings.robust_clear_radius_m <= settings.clearance_radius_m:
        raise ValueError("robust_clear_radius_m must lie in (0, clearance_radius_m]")
    fallback_cover_radius = settings.fallback_grid_step_m * 2**0.5 / 2.0
    if fallback_cover_radius >= settings.clearance_radius_m:
        raise ValueError(
            "fallback grid cells are too large for the 20 m clearance radius"
        )


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
                "sides",
                "radius_m",
                "rotation_deg",
                "scan_origin",
                "route_length_m",
                "measure_count",
                "switch_count",
                "estimated_base_time_s",
                "analytic_max_distance_m",
                "numerical_max_distance_m",
                "coverage_ratio",
                "worst_x",
                "worst_y",
                "achieved_margin_m",
                "valid",
            ]
        )
        for plan in plans:
            metric = plan.coverage
            writer.writerow(
                [
                    plan.sides,
                    plan.radius_m,
                    plan.rotation_deg,
                    plan.scan_origin,
                    plan.route_length_m,
                    plan.measure_count,
                    plan.switch_count,
                    plan.estimated_base_time_s,
                    metric.analytic_max_distance_m,
                    metric.numerical_max_distance_m,
                    metric.coverage_ratio,
                    *metric.worst_point,
                    metric.achieved_margin_m,
                    metric.valid,
                ]
            )


def summarize_action_times(
    action_log: Path,
    total_virtual_time_s: float,
) -> dict[str, float]:
    """从动作日志汇总互斥耗时类别，供结果表、控制台和柱状图共用。"""

    totals = {
        "movement": 0.0,
        "channel_switch": 0.0,
        "detection": 0.0,
        "clearance": 0.0,
    }
    with action_log.open("r", encoding="utf-8") as file:
        for line in file:
            if not line.strip():
                continue
            record = json.loads(line)
            breakdown = record.get("time_breakdown_s") or {}
            totals["movement"] += float(breakdown.get("movement", 0.0))
            totals["channel_switch"] += float(breakdown.get("channel_switch", 0.0))
            operation = float(breakdown.get("measure_or_clear", 0.0))
            if record.get("action_type") == "measure":
                totals["detection"] += operation
            elif record.get("action_type") == "clear":
                totals["clearance"] += operation
    accounted = sum(totals.values())
    totals["other"] = max(0.0, float(total_virtual_time_s) - accounted)
    totals["total"] = float(total_virtual_time_s)
    return totals


def load_truth_statistics(truth_file: Path | None) -> dict[str, Any]:
    """读取事后本地真值中的题号和源类型；正式策略运行期间不得调用。"""

    unknown: dict[str, Any] = {
        "problem_number": 3,
        "source_count": None,
        "omnidirectional_source_count": None,
        "directional_source_count": None,
        "truth_metadata_error": None,
    }
    if truth_file is None:
        return unknown
    try:
        document = json.loads(truth_file.read_text(encoding="utf-8"))
        sources = document.get("sources")
        if not isinstance(sources, list):
            raise ValueError("truth file does not contain a sources list")
        problem_number = int(document.get("problem", 3))
        if problem_number not in (3, 4):
            raise ValueError("truth file problem must be 3 or 4")
        directional = sum(
            source.get("source_type") == "directional" for source in sources
        )
        omnidirectional = sum(
            source.get("source_type") == "omnidirectional" for source in sources
        )
        return {
            "problem_number": problem_number,
            "source_count": len(sources),
            "omnidirectional_source_count": omnidirectional,
            "directional_source_count": directional,
            "truth_metadata_error": None,
        }
    except (OSError, TypeError, ValueError, json.JSONDecodeError) as error:
        unknown["truth_metadata_error"] = f"{type(error).__name__}: {error}"
        return unknown


def format_run_report(summary: dict[str, Any]) -> str:
    """把单局核心数量和耗时整理为便于终端阅读的中文报告。"""

    def seconds(value: Any) -> str:
        return "未知" if value is None else f"{float(value):.2f} s"

    source_count = summary.get("source_count")
    source_text = (
        "未知（未提供事后真值）" if source_count is None else str(source_count)
    )
    lines = [
        f"\n===== 问题 {summary.get('problem_number', 3)} 单次测试汇总 =====",
        f"策略：{summary.get('strategy', 'unknown')}",
        f"总耗时：{seconds(summary.get('total_virtual_time_s'))}",
        f"总清除数量：{summary.get('cleared_count', 0)}",
        f"干扰源数量：{source_text}",
    ]
    if summary.get("problem_number") == 4 and source_count is not None:
        lines.append(
            "源类型："
            f"全向 {summary.get('omnidirectional_source_count', 0)}，"
            f"定向 {summary.get('directional_source_count', 0)}"
        )
    lines.extend(
        [
            f"平均单个源耗时：{seconds(summary.get('average_time_per_source_s'))}",
            "平均每个已清除源耗时："
            f"{seconds(summary.get('average_time_per_cleared_source_s'))}",
            "平均定位至清除耗时："
            f"{seconds(summary.get('average_localize_clear_time_s'))}",
            f"累计移动距离：{float(summary.get('total_movement_distance_m', 0.0)):.2f} m",
            (
                "动作次数："
                f"检测 {summary.get('measure_count', 0)}，"
                f"换频 {summary.get('switch_count', 0)}，"
                f"清除成功 {summary.get('clear_success_count', 0)}，"
                f"清除失败 {summary.get('clear_failure_count', 0)}"
            ),
        ]
    )
    timing = summary.get("time_breakdown_s") or {}
    lines.append(
        "耗时分解："
        f"移动 {seconds(timing.get('movement'))}；"
        f"换频 {seconds(timing.get('channel_switch'))}；"
        f"检测 {seconds(timing.get('detection'))}；"
        f"清除 {seconds(timing.get('clearance'))}；"
        f"其他 {seconds(timing.get('other'))}"
    )
    if summary.get("figure_directory_png"):
        lines.append(f"PNG 图像目录：{summary['figure_directory_png']}")
        lines.append(f"PDF 图像目录：{summary['figure_directory_pdf']}")
    if summary.get("summary_file"):
        lines.append(f"完整汇总文件：{summary['summary_file']}")
    lines.append("=" * 35)
    return "\n".join(lines)


def _append_formal_result(path: Path, summary: dict[str, Any]) -> None:
    """按附件正式测试表关键信息追加一行，不改名模拟器原始日志。"""

    path.parent.mkdir(parents=True, exist_ok=True)
    exists = path.exists()
    columns = [
        "run_index",
        "case_code",
        "strategy",
        "seed",
        "problem_number",
        "source_count",
        "omnidirectional_source_count",
        "directional_source_count",
        "cleared_count",
        "clearance_ratio",
        "total_virtual_time_s",
        "program_wall_time_s",
        "average_time_per_source_s",
        "average_time_per_cleared_source_s",
        "movement_time_s",
        "channel_switch_time_s",
        "detection_time_s",
        "clearance_time_s",
        "other_time_s",
        "action_log",
        "summary_file",
        "official_log_original_names",
        "figure_directory_png",
        "figure_directory_pdf",
        "trajectory_figure",
        "clearance_detail_figure",
        "time_breakdown_figure",
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
    *,
    strategy_name: str,
    host: str,
    port: int,
    robot_id: str,
    timeout_s: float,
    settings: Problem3Settings,
    run_index: int,
    case_code: str | None,
    official_log_names: list[str],
    formal: bool,
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
        base_url=f"http://{host}:{port}",
        robot_id=robot_id,
        timeout_s=timeout_s,
        request_prefix=f"p3-{strategy_name}-s{settings.seed}-r{run_index}",
    )
    executor = Problem3Executor(client, state, logger, strategy_name)
    strategy_types = {
        ROBUST_STRATEGY: RobustPolygonRollingStrategy,
        MPC_STRATEGY: BeliefMPCStrategy,
        TOUR_STRATEGY: IntegratedBearingTourStrategy,
        DISTANCE_TOUR_STRATEGY: DistanceOptimizedBearingTourStrategy,
        SAFE_CLEAR_TOUR_STRATEGY: SafeClearRouteAlignedTourStrategy,
        DYNAMIC_COVERAGE_TOUR_STRATEGY: DynamicCoverageRouteAlignedTourStrategy,
    }
    strategy = strategy_types[strategy_name](settings)
    start = time.monotonic()
    try:
        executor.enter()
        result = strategy.run(executor)
        executor.exit()
        summary = summarize_state(
            result.state,
            strategy_name,
            result.selected_plan.as_dict(),
            official_log_names,
        )
        truth_statistics = load_truth_statistics(truth_file)
        timing = summarize_action_times(
            action_log,
            float(summary["total_virtual_time_s"]),
        )
        source_count = truth_statistics["source_count"]
        cleared_count = int(summary["cleared_count"])
        if source_count:
            summary["clearance_ratio"] = cleared_count / source_count
            summary["clearance_ratio_denominator"] = "postrun truth source count"
        summary.update(truth_statistics)
        summary.update(
            {
                "time_breakdown_s": timing,
                "movement_time_s": timing["movement"],
                "channel_switch_time_s": timing["channel_switch"],
                "detection_time_s": timing["detection"],
                "clearance_time_s": timing["clearance"],
                "other_time_s": timing["other"],
                "average_time_per_source_s": (
                    float(summary["total_virtual_time_s"]) / source_count
                    if source_count
                    else None
                ),
                "average_time_per_cleared_source_s": (
                    float(summary["total_virtual_time_s"]) / cleared_count
                    if cleared_count
                    else None
                ),
            }
        )
        figure_root = (
            PROBLEM4_TRAJECTORY_FIGURES_DIR
            if truth_statistics["problem_number"] == 4
            else PROBLEM3_TRAJECTORY_FIGURES_DIR
        )
        figure_directory = figure_root / stem
        pdf_figure_directory = paired_pdf_path(
            figure_directory / "placeholder.png"
        ).parent
        plotting_error: str | None = None
        try:
            from problems.problem3.plotting import plot_run_replay

            figure_paths = plot_run_replay(
                action_log,
                figure_directory / "trajectory.png",
                figure_directory / "clearance_details.png",
                truth_file,
                time_breakdown_s=timing,
                timing_output=figure_directory / "time_breakdown.png",
            )
        except Exception as error:
            # 模拟动作已经完整结束，绘图异常不能把一次有效正式测试伪装成运行失败。
            figure_paths = {
                "trajectory_figure": None,
                "trajectory_figure_png": None,
                "trajectory_figure_pdf": None,
                "clearance_detail_figure": None,
                "clearance_detail_figure_png": None,
                "clearance_detail_figure_pdf": None,
                "time_breakdown_figure": None,
                "time_breakdown_figure_png": None,
                "time_breakdown_figure_pdf": None,
            }
            plotting_error = f"{type(error).__name__}: {error}"
        _write_plan_search(plan_path, result.compared_plans)
        summary.update(
            {
                "run_index": run_index,
                "case_code": case_code,
                "seed": settings.seed,
                "host": host,
                "port": port,
                "program_wall_time_s": time.monotonic() - start,
                "action_log": str(action_log),
                "summary_file": str(summary_path),
                "polygon_search_file": str(plan_path),
                "settings": asdict(settings),
                "postrun_truth_file": str(truth_file)
                if truth_file is not None
                else None,
                "plotting_error": plotting_error,
                "figure_directory_png": str(figure_directory),
                "figure_directory_pdf": str(pdf_figure_directory),
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
    *,
    strategy: str,
    host: str,
    port: int,
    robot_id: str,
    timeout_s: float,
    settings: Problem3Settings,
    runs: int = 1,
    case_codes: list[str] | None = None,
    official_logs: list[Path] | None = None,
    truth_files: list[Path] | None = None,
    formal: bool = False,
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
                base_url=f"http://{host}:{port}",
                robot_id=robot_id,
                timeout_s=timeout_s,
            )
            while True:
                try:
                    probe.check_connection()
                    break
                except Exception:
                    if time.monotonic() >= deadline:
                        raise TimeoutError(
                            "next simulator run did not become available"
                        )
                    time.sleep(min(1.0, max(deadline - time.monotonic(), 0.0)))
        summary = run_once(
            strategy_name=strategy,
            host=host,
            port=port,
            robot_id=robot_id,
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


__all__ = [
    "format_run_report",
    "load_settings",
    "load_truth_statistics",
    "run_once",
    "run_problem3",
    "summarize_action_times",
]
