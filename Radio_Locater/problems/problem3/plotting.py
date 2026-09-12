"""问题3单局运行轨迹和清除误差特写绘图。

绘图只在策略退出后读取程序动作日志及可选本地真值，不参与选点。主图保留全局运动
结构，所有容易重叠的逐次行为另在时间轴中展开；清除细节独立成图，避免遮盖主图。
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Circle

from config.constants import ARENA_RADIUS_M, CLEARANCE_RADIUS_M
from problems.problem1.plotting import configure_chinese_font


RESULT_STYLE: dict[str, dict[str, Any]] = {
    "no_signal": {"color": "#9AA0A6", "marker": "o", "label": "检测：无信号"},
    "direction": {"color": "#F28E2B", "marker": "^", "label": "检测：示向度"},
    "near": {"color": "#B455D4", "marker": "D", "label": "检测：near"},
    "clear_success": {"color": "#2CA25F", "marker": "P", "label": "清除成功"},
    "clear_failure": {"color": "#D62728", "marker": "X", "label": "清除失败"},
}


@dataclass(frozen=True, slots=True)
class ReplayEvent:
    """动作日志中一次可绘制的检测或清除行为。"""

    index: int
    virtual_time_s: float
    position: tuple[float, float]
    channel: int
    action_type: str
    result: str


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    """逐行读取JSONL，并在坏行处给出明确行号。"""

    records: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as file:
        for line_number, line in enumerate(file, 1):
            if not line.strip():
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError as error:
                raise ValueError(f"invalid JSONL at {path}:{line_number}") from error
            records.append(record)
    if not records:
        raise ValueError(f"empty action log: {path}")
    return records


def _event_result(record: dict[str, Any]) -> str | None:
    """把附件原始结果映射为固定绘图颜色类别。"""

    action = record.get("action_type")
    response = record.get("raw_response", {})
    if action == "measure":
        result = response.get("measure_result")
        return result if result in {"no_signal", "direction", "near"} else None
    if action == "clear":
        return (
            "clear_success"
            if response.get("clear_result") == "success"
            else "clear_failure"
        )
    return None


def load_replay(
    action_log: Path,
) -> tuple[list[tuple[float, float]], list[ReplayEvent], str]:
    """提取包含重复停留点的完整轨迹和所有检测/清除事件。"""

    records = _read_jsonl(action_log)
    trajectory: list[tuple[float, float]] = []
    events: list[ReplayEvent] = []
    strategy = str(records[0].get("strategy", "unknown"))
    for record in records:
        position_data = record.get("position")
        if not isinstance(position_data, dict):
            continue
        position = (float(position_data["x"]), float(position_data["y"]))
        trajectory.append(position)
        result = _event_result(record)
        channels = record.get("target_channels") or []
        if result is not None and channels:
            events.append(
                ReplayEvent(
                    len(events) + 1,
                    float(record["virtual_time_s"]),
                    position,
                    int(channels[0]),
                    str(record["action_type"]),
                    result,
                )
            )
    return trajectory, events, strategy


def load_sources(truth_file: Path | None) -> list[dict[str, Any]]:
    """读取可选的事后真值；策略运行阶段不得调用本函数。"""

    if truth_file is None:
        return []
    document = json.loads(truth_file.read_text(encoding="utf-8"))
    sources = document.get("sources")
    if not isinstance(sources, list):
        raise ValueError("truth file does not contain a sources list")
    return sources


def _save(figure: plt.Figure, path: Path) -> None:
    """以论文插图分辨率保存白底图像。"""

    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=220, bbox_inches="tight", facecolor="white")
    plt.close(figure)


def _draw_direction_arrows(axis: plt.Axes, points: np.ndarray) -> None:
    """沿长移动段稀疏绘制箭头，既表达方向又不铺满轨迹。"""

    if len(points) < 2:
        return
    displacement = np.diff(points, axis=0)
    moving = np.flatnonzero(np.linalg.norm(displacement, axis=1) > 1e-6)
    if not len(moving):
        return
    stride = max(1, math.ceil(len(moving) / 18))
    for index in moving[::stride]:
        start = points[index]
        delta = displacement[index]
        axis.annotate(
            "",
            xy=start + 0.58 * delta,
            xytext=start + 0.42 * delta,
            arrowprops={"arrowstyle": "-|>", "color": "#2468A2", "lw": 0.9},
            zorder=3,
        )


def plot_run_trajectory(
    action_log: Path,
    output: Path,
    truth_file: Path | None = None,
) -> None:
    """绘制全局轨迹、源位置、空间事件和不重叠的逐动作时间轴。"""

    configure_chinese_font()
    trajectory, events, strategy = load_replay(action_log)
    sources = load_sources(truth_file)
    points = np.asarray(trajectory, dtype=float)
    figure = plt.figure(figsize=(14.2, 8.3), constrained_layout=True)
    grid = figure.add_gridspec(1, 2, width_ratios=(3.25, 1.0))
    axis = figure.add_subplot(grid[0, 0])
    timeline = figure.add_subplot(grid[0, 1])

    axis.add_patch(
        Circle(
            (0.0, 0.0), ARENA_RADIUS_M, fill=False, linestyle="--",
            linewidth=1.3, edgecolor="#59636F", label="目标圆域",
        )
    )
    axis.plot(
        points[:, 0], points[:, 1], color="#2878B5", linewidth=1.25,
        alpha=0.72, zorder=1, label="机器狗运动轨迹",
    )
    _draw_direction_arrows(axis, points)
    axis.scatter(
        points[0, 0], points[0, 1], marker="s", s=58, color="#111111",
        edgecolor="white", linewidth=0.7, zorder=7, label="起点",
    )
    axis.scatter(
        points[-1, 0], points[-1, 1], marker="o", s=82, facecolor="none",
        edgecolor="#00A6A6", linewidth=1.7, zorder=7, label="结束位置",
    )
    if sources:
        source_points = np.asarray(
            [[float(source["x_m"]), float(source["y_m"])] for source in sources]
        )
        axis.scatter(
            source_points[:, 0], source_points[:, 1], marker="*", s=105,
            color="#C51B33", edgecolor="white", linewidth=0.55, zorder=6,
            label=f"干扰源真值（{len(sources)}个）",
        )
    else:
        axis.text(
            0.02, 0.98, "未提供可读真值：仅绘制策略可见轨迹",
            transform=axis.transAxes, ha="left", va="top", fontsize=9,
            bbox={"boxstyle": "round,pad=0.3", "fc": "white", "ec": "#888888"},
        )

    for result, style in RESULT_STYLE.items():
        selected = [event for event in events if event.result == result]
        if not selected:
            continue
        event_points = np.asarray([event.position for event in selected])
        size = 14 if result == "no_signal" else 34
        axis.scatter(
            event_points[:, 0], event_points[:, 1], s=size,
            marker=style["marker"], color=style["color"],
            alpha=0.52 if result == "no_signal" else 0.9,
            edgecolor="white", linewidth=0.35, zorder=4,
        )

    axis.set_aspect("equal")
    extent = max(ARENA_RADIUS_M * 1.06, float(np.max(np.abs(points))) * 1.04)
    axis.set_xlim(-extent, extent)
    axis.set_ylim(-extent, extent)
    axis.set_xlabel("x / m（正东）")
    axis.set_ylabel("y / m（正北）")
    axis.grid(alpha=0.16)
    axis.set_title(f"问题3单局运动复盘：{strategy}")

    for result, style in RESULT_STYLE.items():
        selected = [event for event in events if event.result == result]
        if selected:
            timeline.scatter(
                [event.index for event in selected],
                [event.virtual_time_s / 3600.0 for event in selected],
                marker=style["marker"], color=style["color"], s=28,
                label=f"{style['label']} ({len(selected)})",
            )
    timeline.plot(
        [event.index for event in events],
        [event.virtual_time_s / 3600.0 for event in events],
        color="#B8C2CC", linewidth=0.8, zorder=0,
    )
    timeline.set_xlabel("检测/清除行为序号")
    timeline.set_ylabel("累计虚拟时间 / h")
    timeline.set_title("全部行为与结果")
    timeline.grid(alpha=0.18)
    timeline.legend(loc="upper left", fontsize=7.7, framealpha=0.92)

    handles = [
        Line2D([0], [0], color="#2878B5", lw=1.5, label="机器狗运动轨迹"),
        Line2D([0], [0], marker="s", color="none", markerfacecolor="#111111",
               markersize=7, label="起点"),
        Line2D([0], [0], marker="o", color="none", markerfacecolor="none",
               markeredgecolor="#00A6A6", markersize=8, label="结束位置"),
    ]
    if sources:
        handles.append(
            Line2D([0], [0], marker="*", color="none", markerfacecolor="#C51B33",
                   markersize=11, label=f"干扰源真值（{len(sources)}个）")
        )
    handles.extend(
        Line2D(
            [0], [0], marker=style["marker"], color="none",
            markerfacecolor=style["color"], markersize=7, label=style["label"],
        )
        for result, style in RESULT_STYLE.items()
        if any(event.result == result for event in events)
    )
    axis.legend(
        handles=handles, loc="lower center", bbox_to_anchor=(0.5, -0.16),
        ncol=4, fontsize=8.2, framealpha=0.95,
    )
    _save(figure, output)


def plot_clearance_details(
    action_log: Path,
    output: Path,
    truth_file: Path,
) -> bool:
    """按频道绘制清除点特写；返回False表示没有可画的清除事件。"""

    configure_chinese_font()
    _, events, strategy = load_replay(action_log)
    sources = load_sources(truth_file)
    source_by_channel = {int(source["channel"]): source for source in sources}
    clear_events = [event for event in events if event.action_type == "clear"]
    channels = sorted({event.channel for event in clear_events if event.channel in source_by_channel})
    if not channels:
        return False
    columns = min(4, len(channels))
    rows = math.ceil(len(channels) / columns)
    figure, axes = plt.subplots(rows, columns, figsize=(3.15 * columns, 3.0 * rows))
    axes_array = np.atleast_1d(axes).ravel()
    for axis, channel in zip(axes_array, channels, strict=False):
        source = source_by_channel[channel]
        truth = np.asarray([float(source["x_m"]), float(source["y_m"])])
        attempts = [event for event in clear_events if event.channel == channel]
        attempt_points = np.asarray([event.position for event in attempts])
        axis.add_patch(
            Circle(
                truth, CLEARANCE_RADIUS_M, facecolor="#8FD19E", edgecolor="#267A43",
                alpha=0.18, linewidth=1.0,
            )
        )
        axis.scatter(*truth, marker="*", s=115, color="#C51B33", zorder=5)
        for attempt in attempts:
            style = RESULT_STYLE[attempt.result]
            axis.scatter(
                *attempt.position, marker=style["marker"], s=65,
                color=style["color"], edgecolor="white", linewidth=0.5, zorder=6,
            )
            axis.plot(
                [truth[0], attempt.position[0]], [truth[1], attempt.position[1]],
                color=style["color"], linestyle=":", linewidth=1.0,
            )
        last_error = float(np.linalg.norm(attempt_points[-1] - truth))
        radius = max(28.0, float(np.max(np.linalg.norm(attempt_points - truth, axis=1))) + 8.0)
        axis.set_xlim(truth[0] - radius, truth[0] + radius)
        axis.set_ylim(truth[1] - radius, truth[1] + radius)
        axis.set_aspect("equal")
        axis.set_title(f"频道 {channel}：末次误差 {last_error:.2f} m", fontsize=9)
        axis.axis("off")
        bar_y = truth[1] - 0.80 * radius
        axis.plot([truth[0] - 5.0, truth[0] + 5.0], [bar_y, bar_y], color="#222222", lw=2)
        axis.text(truth[0], bar_y + 1.5, "10 m", ha="center", va="bottom", fontsize=7)
    for axis in axes_array[len(channels):]:
        axis.axis("off")
    handles = [
        Line2D([0], [0], marker="*", color="none", markerfacecolor="#C51B33",
               markersize=11, label="干扰源真值"),
        Line2D([0], [0], marker="P", color="none", markerfacecolor="#2CA25F",
               markersize=8, label="清除成功点"),
        Line2D([0], [0], marker="X", color="none", markerfacecolor="#D62728",
               markersize=8, label="清除失败点"),
        Line2D([0], [0], color="#267A43", lw=6, alpha=0.25,
               label="真值周围20 m可清除区"),
    ]
    figure.legend(handles=handles, loc="lower center", ncol=4, fontsize=8)
    figure.suptitle(f"问题3清除位置特写：{strategy}", fontsize=13)
    figure.subplots_adjust(top=0.92, bottom=0.08, hspace=0.22, wspace=0.10)
    _save(figure, output)
    return True


def plot_run_replay(
    action_log: Path,
    trajectory_output: Path,
    clearance_output: Path,
    truth_file: Path | None = None,
) -> dict[str, str | None]:
    """生成一次模拟的主运动图和可选清除特写图。"""

    plot_run_trajectory(action_log, trajectory_output, truth_file)
    clearance_path: str | None = None
    if truth_file is not None and plot_clearance_details(
        action_log, clearance_output, truth_file
    ):
        clearance_path = str(clearance_output)
    return {
        "trajectory_figure": str(trajectory_output),
        "clearance_detail_figure": clearance_path,
    }
