"""项目命令行入口。

负责运行问题 1、2 数值程序，或启动问题 3、4 共用的附件兼容本地模拟器。
"""

import argparse
import json
import os
from pathlib import Path
from urllib.parse import urlsplit


def main() -> None:
    """解析命令行，并转发至数值问题或本地模拟器入口。"""

    # prog 与 pyproject.toml 中注册的命令名一致，确保帮助信息不会随启动方式改变。
    parser = argparse.ArgumentParser(
        prog="radio-locator",
        description="2026 CUMCM B 题无线电干扰源定位与清除",
    )
    subparsers = parser.add_subparsers(dest="problem")
    subparsers.add_parser("problem1", help="运行问题 1 验证并生成图表")
    problem2_parser = subparsers.add_parser(
        "problem2", help="指定第一检测点和示向度，搜索第二检测点"
    )
    problem2_parser.add_argument(
        "--x", type=float, default=-900.0, help="第一检测点 x 坐标 / m"
    )
    problem2_parser.add_argument(
        "--y", type=float, default=-500.0, help="第一检测点 y 坐标 / m"
    )
    problem2_parser.add_argument(
        "--bearing",
        type=float,
        default=31.363757,
        help="第一检测点示向度 / °（正东为 0°，逆时针为正）",
    )
    problem3_parser = subparsers.add_parser(
        "problem3", help="连接附件模拟器运行问题3在线搜索、定位与清除策略"
    )
    problem3_parser.add_argument(
        "--strategy",
        choices=(
            "robust_polygon_rolling",
            "belief_mpc",
            "cooperative_bearing_tour",
            "integrated_bearing_tour",
            "distance_optimized_bearing_tour",
            "route_aligned_bearing_tour",
            "safe_clear_route_aligned_tour",
            "dynamic_coverage_route_aligned_tour",
        ),
        required=True,
        help="问题3策略",
    )
    default_url = urlsplit(os.getenv("CUMCM_BASE_URL", "http://127.0.0.1:2026"))
    problem3_parser.add_argument("--host", default=default_url.hostname or "127.0.0.1")
    problem3_parser.add_argument("--port", type=int, default=default_url.port or 2026)
    problem3_parser.add_argument(
        "--robot-id", default=os.getenv("CUMCM_ROBOT_ID", ""), help="参赛队号/本地demo标识"
    )
    problem3_parser.add_argument(
        "--timeout", type=float, default=float(os.getenv("CUMCM_HTTP_TIMEOUT_S", "5"))
    )
    problem3_parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).resolve().parents[2] / "config" / "problem3.yaml",
        help="算法YAML配置（默认使用项目内配置，与启动目录无关）",
    )
    problem3_parser.add_argument("--endgame", dest="tour_endgame_mode", choices=("legacy", "exact", "probe"), help="小规模路线与末段粗定位处理模式")
    problem3_parser.add_argument("--tour-radius", dest="tour_polygon_radius_m", type=float, help="联合巡回覆盖多边形半径/m")
    problem3_parser.add_argument(
        "--route-aligned-sides",
        dest="route_aligned_polygon_sides",
        type=int,
        help="路线对齐策略的保证覆盖多边形边数",
    )
    problem3_parser.add_argument(
        "--route-aligned-radius",
        dest="route_aligned_polygon_radius_m",
        type=float,
        help="路线对齐策略的保证覆盖多边形半径/m",
    )
    problem3_parser.add_argument("--channel-order", dest="tour_channel_order", choices=("legacy", "alternating"), help="共享测站频道顺序")
    problem3_parser.add_argument("--seed", type=int)
    problem3_parser.add_argument("--grid-step", dest="coverage_grid_step_m", type=float)
    problem3_parser.add_argument(
        "--validation-step", dest="coverage_validation_step_m", type=float
    )
    problem3_parser.add_argument(
        "--particles", dest="particle_count_per_channel", type=int
    )
    problem3_parser.add_argument(
        "--candidate-limit", dest="candidate_action_limit", type=int
    )
    problem3_parser.add_argument("--horizon", type=int)
    problem3_parser.add_argument("--beam-width", dest="beam_width", type=int)
    problem3_parser.add_argument("--p0", type=float)
    problem3_parser.add_argument("--g0", type=float)
    problem3_parser.add_argument("--polygon-sides", dest="polygon_sides", type=int)
    problem3_parser.add_argument(
        "--polygon-radius", dest="polygon_radius_m", type=float
    )
    problem3_parser.add_argument(
        "--polygon-rotation", dest="polygon_rotation_deg", type=float
    )
    problem3_parser.add_argument(
        "--robustness-margin", dest="robustness_margin_m", type=float
    )
    problem3_parser.add_argument(
        "--scan-origin", action=argparse.BooleanOptionalAction, default=None
    )
    problem3_parser.add_argument(
        "--fixed-polygon", action="store_true", help="使用指定n=7/rho=1000等参数而不搜索"
    )
    problem3_parser.add_argument("--runs", type=int, choices=(1, 3), default=1)
    problem3_parser.add_argument(
        "--formal", action="store_true", help="正式模式：要求连续三局并追加正式汇总CSV"
    )
    problem3_parser.add_argument(
        "--case-code", action="append", default=[], help="每局案例编号，可重复三次"
    )
    problem3_parser.add_argument(
        "--official-log",
        action="append",
        type=Path,
        default=[],
        help="模拟器导出的原始日志，可重复三次；仅记录原文件名，不改名",
    )
    problem3_parser.add_argument(
        "--truth-file",
        action="append",
        type=Path,
        default=[],
        help="可选事后真值JSON，可重复三次；仅用于运行结束后作图",
    )
    problem3_parser.add_argument(
        "--print-json",
        action="store_true",
        help="在简明收尾报告后额外输出完整 JSON；默认只写入汇总文件",
    )
    problem3_parser.add_argument(
        "--next-run-wait", type=float, default=0.0, help="等待GUI启动下一局的秒数"
    )
    problem4_parser = subparsers.add_parser(
        "problem4", help="运行全向/定向混合源的保证发现、定位与清除策略"
    )
    problem4_parser.add_argument(
        "--strategy", choices=(
            "guaranteed_directional_lattice", "optimized_guaranteed_lattice",
            "problem4_fast", "legacy_outer_probe_fast",
        ),
        default="guaranteed_directional_lattice",
    )
    problem4_parser.add_argument("--host", default=default_url.hostname or "127.0.0.1")
    problem4_parser.add_argument("--port", type=int, default=default_url.port or 2026)
    problem4_parser.add_argument("--robot-id", default=os.getenv("CUMCM_ROBOT_ID", ""))
    problem4_parser.add_argument(
        "--timeout", type=float, default=float(os.getenv("CUMCM_HTTP_TIMEOUT_S", "5"))
    )
    problem4_parser.add_argument("--seed", type=int, default=1)
    problem4_parser.add_argument("--grid-spacing", type=float, default=1_000.0)
    problem4_parser.add_argument("--bearing-limit", type=int, default=5)
    problem4_parser.add_argument("--clear-insertion", type=float, default=300.0)
    problem4_parser.add_argument("--runs", type=int, default=1)
    problem4_parser.add_argument("--truth-file", action="append", type=Path, default=[])
    problem4_parser.add_argument("--print-json", action="store_true")
    simulator_parser = subparsers.add_parser(
        "simulator", help="启动与附件接口兼容的问题 3/4 本地模拟器"
    )
    simulator_parser.add_argument(
        "--problem",
        dest="simulation_problem",
        type=int,
        choices=(3, 4),
        required=True,
        help="仿真题号",
    )
    simulator_parser.add_argument(
        "--seed", type=int, default=2026, help="案例随机种子"
    )
    simulator_parser.add_argument(
        "--robot-id", default="demo", help="接口逐字节校验的本地机器狗标识"
    )
    simulator_parser.add_argument(
        "--host", default="127.0.0.1", help="监听地址，默认仅本机回环"
    )
    simulator_parser.add_argument(
        "--port", type=int, default=2026, help="监听端口"
    )
    simulator_parser.add_argument(
        "--truth-output",
        type=Path,
        help="可选案例真值 JSON 路径；默认写入 res/simulator",
    )
    arguments = parser.parse_args()

    # 延迟导入避免仅查看 --help 时加载 Matplotlib 等较重的绘图库。
    if arguments.problem == "problem1":
        from problems.problem1.main import main as run_problem1

        run_problem1()
    elif arguments.problem == "problem2":
        from problems.problem2.main import main as run_problem2

        run_problem2(arguments.x, arguments.y, arguments.bearing)
    elif arguments.problem == "problem3":
        from problems.problem3.main import (
            format_run_report,
            load_settings,
            run_problem3,
        )

        override_names = (
            "tour_polygon_radius_m", "route_aligned_polygon_sides",
            "route_aligned_polygon_radius_m", "tour_channel_order", "tour_endgame_mode",
            "seed", "coverage_grid_step_m", "coverage_validation_step_m",
            "particle_count_per_channel", "candidate_action_limit", "horizon",
            "beam_width", "p0", "g0", "polygon_sides", "polygon_radius_m",
            "polygon_rotation_deg", "robustness_margin_m", "scan_origin",
        )
        overrides = {name: getattr(arguments, name) for name in override_names}
        if arguments.fixed_polygon:
            overrides["optimize_polygon"] = False
        settings = load_settings(arguments.config, overrides)
        summaries = run_problem3(
            strategy=arguments.strategy,
            host=arguments.host,
            port=arguments.port,
            robot_id=arguments.robot_id,
            timeout_s=arguments.timeout,
            settings=settings,
            runs=arguments.runs,
            case_codes=arguments.case_code,
            official_logs=arguments.official_log,
            truth_files=arguments.truth_file,
            formal=arguments.formal,
            next_run_wait_s=arguments.next_run_wait,
        )
        for summary in summaries:
            print(format_run_report(summary))
        if arguments.print_json:
            print("\n完整 JSON 汇总：")
            print(json.dumps(summaries, ensure_ascii=False, indent=2))
    elif arguments.problem == "problem4":
        from problems.problem4.config import Problem4Settings
        from problems.problem4.main import format_run_report, run_problem4

        spacing = arguments.grid_spacing
        settings = Problem4Settings(
            seed=arguments.seed,
            directional_grid_spacing_m=spacing,
            directional_grid_offset_y_m=spacing * 3**0.5 / 4.0,
            opportunistic_bearing_limit=arguments.bearing_limit,
            route_clear_insertion_limit_m=arguments.clear_insertion,
        )
        summaries = run_problem4(
            host=arguments.host,
            port=arguments.port,
            robot_id=arguments.robot_id,
            timeout_s=arguments.timeout,
            settings=settings,
            strategy=arguments.strategy,
            runs=arguments.runs,
            truth_files=arguments.truth_file,
        )
        for summary in summaries:
            print(format_run_report(summary))
        if arguments.print_json:
            print("\n完整 JSON 汇总：")
            print(json.dumps(summaries, ensure_ascii=False, indent=2))
    elif arguments.problem == "simulator":
        from config.paths import SIMULATOR_CASES_DIR
        from radio_locator.local_simulator import run_local_simulator

        truth_output = arguments.truth_output or (
            SIMULATOR_CASES_DIR
            / f"problem{arguments.simulation_problem}_seed{arguments.seed}.json"
        )
        run_local_simulator(
            problem=arguments.simulation_problem,
            seed=arguments.seed,
            robot_id=arguments.robot_id,
            host=arguments.host,
            port=arguments.port,
            truth_output=truth_output,
        )
    else:
        parser.print_help()
