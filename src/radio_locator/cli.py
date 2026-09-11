"""项目命令行入口。

当前只负责展示项目帮助，不会运行算法或连接模拟器。问题 1 至问题 4 的子命令将在
相应算法完成后添加，避免在基础骨架中提供不可用的命令。
"""

import argparse


def main() -> None:
    """解析命令行，并将 problem1/problem2 转发给对应实现。"""

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
    arguments = parser.parse_args()

    # 延迟导入避免仅查看 --help 时加载 Matplotlib 等较重的绘图库。
    if arguments.problem == "problem1":
        from problems.problem1.main import main as run_problem1

        run_problem1()
    elif arguments.problem == "problem2":
        from problems.problem2.main import main as run_problem2

        run_problem2(arguments.x, arguments.y, arguments.bearing)
    else:
        parser.print_help()
