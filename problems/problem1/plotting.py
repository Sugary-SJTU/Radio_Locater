"""问题 1 的论文图与验证图绘制函数。"""

from __future__ import annotations

from math import atan2, degrees
from pathlib import Path
from typing import Sequence

from radio_locator.runtime import prepare_matplotlib_config

prepare_matplotlib_config()

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Polygon, Wedge
from mpl_toolkits.axes_grid1.inset_locator import inset_axes, mark_inset

from problems.problem1.config import ANGLE_ERROR_DEG, DOMAIN_RADIUS_M
from problems.problem1.examples import LocalizationCase
from problems.problem1.model import LocalizationResult, localize
from radio_locator.geometry import rotating_calipers_diameter


def configure_chinese_font() -> None:
    """设置当前环境已有的中文字体，并保证负号正常显示。"""

    plt.rcParams["font.sans-serif"] = [
        # Windows 10/11 自带字体优先，后两项兼容 Conda/Linux 环境。
        "Microsoft YaHei",
        "SimHei",
        "Source Han Sans CN",
        "Noto Sans CJK SC",
        "DejaVu Sans",
    ]
    plt.rcParams["axes.unicode_minus"] = False


def _save_figure(figure: plt.Figure, output: Path) -> None:
    """创建父目录并以适合论文插图的分辨率保存。"""

    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=220, bbox_inches="tight", facecolor="white")
    plt.close(figure)


def _draw_arena(axis: plt.Axes) -> None:
    """绘制半径 1800 m 的目标圆域。"""

    axis.add_patch(
        Circle(
            (0.0, 0.0),
            DOMAIN_RADIUS_M,
            fill=False,
            linestyle="--",
            linewidth=1.4,
            edgecolor="#586174",
            label="目标区域边界",
        )
    )


def _draw_measurements(
    axis: plt.Axes,
    case: LocalizationCase,
    ray_length: float = 3_200.0,
) -> None:
    """绘制检测点、误差扇形填充和两条细实线边界，不绘制示向度中心线。"""

    colors = ("#2878B5", "#9AC9DB", "#F28E2B", "#59A14F")
    for index, measurement in enumerate(case.measurements):
        x, y = measurement.station
        color = colors[index % len(colors)]
        start_angle = measurement.bearing_deg - ANGLE_ERROR_DEG
        axis.add_patch(
            Wedge(
                (x, y),
                ray_length,
                start_angle,
                measurement.bearing_deg + ANGLE_ERROR_DEG,
                color=color,
                alpha=0.12,
            )
        )
        for boundary_angle in (
            measurement.bearing_deg - ANGLE_ERROR_DEG,
            measurement.bearing_deg + ANGLE_ERROR_DEG,
        ):
            angle = np.radians(boundary_angle)
            axis.plot(
                [x, x + ray_length * np.cos(angle)],
                [y, y + ray_length * np.sin(angle)],
                color=color,
                linewidth=0.85,
                linestyle="-",
                alpha=0.95,
            )
        axis.scatter(
            x,
            y,
            s=45,
            color=color,
            edgecolor="white",
            zorder=5,
            label="检测点" if index == 0 else None,
        )
        axis.annotate(
            f"$S_{index + 1}$",
            (x, y),
            xytext=(6, 6),
            textcoords="offset points",
        )


def _draw_local_wedge_boundaries(
    axis: plt.Axes,
    case: LocalizationCase,
    ray_length: float = 4_000.0,
) -> None:
    """在局部特写中以虚射线延拓所有 ±1° 扇形边界。"""

    colors = ("#2878B5", "#9AC9DB", "#F28E2B", "#59A14F")
    for index, measurement in enumerate(case.measurements):
        x, y = measurement.station
        color = colors[index % len(colors)]
        for boundary_angle in (
            measurement.bearing_deg - ANGLE_ERROR_DEG,
            measurement.bearing_deg + ANGLE_ERROR_DEG,
        ):
            angle = np.radians(boundary_angle)
            axis.plot(
                [x, x + ray_length * np.cos(angle)],
                [y, y + ray_length * np.sin(angle)],
                color=color,
                linewidth=0.9,
                linestyle="--",
                alpha=0.85,
                zorder=1,
            )


def _wedge_boundary_handle(linestyle: str) -> Line2D:
    """返回图例中统一表示多组扇形边界的线对象。"""

    return Line2D(
        [0],
        [0],
        color="#586174",
        linewidth=1.0,
        linestyle=linestyle,
        label="±1° 扇形边界",
    )


def plot_three_station_intersection(
    case: LocalizationCase,
    result: LocalizationResult,
    output: Path,
) -> None:
    """绘制三个检测点形成的扇形交汇定位区域。"""

    configure_chinese_font()
    figure, axis = plt.subplots(figsize=(8.0, 7.2))
    _draw_arena(axis)
    _draw_measurements(axis, case)
    axis.add_patch(
        Polygon(
            result.polygon,
            closed=True,
            facecolor="#E15759",
            edgecolor="#B22222",
            alpha=0.45,
            linewidth=2.0,
            label="三站交汇定位区域",
        )
    )
    axis.plot(
        [result.diameter.first[0], result.diameter.second[0]],
        [result.diameter.first[1], result.diameter.second[1]],
        color="#7A1FA2",
        linewidth=2.2,
        marker="o",
        label=f"区域直径 D={result.diameter.distance:.1f} m",
    )
    axis.scatter(
        *case.target,
        marker="*",
        s=170,
        color="#D62728",
        label="验证真值 G",
    )
    axis.set_title("三个检测点的示向度误差带及交汇定位区域")
    axis.set_xlabel("x / m（正东）")
    axis.set_ylabel("y / m（正北）")
    axis.set_aspect("equal")
    axis.set_xlim(-1950, 1950)
    axis.set_ylim(-1950, 1950)
    axis.grid(alpha=0.2)
    handles, labels = axis.get_legend_handles_labels()
    axis.legend(
        [_wedge_boundary_handle("-"), *handles],
        ["±1° 扇形边界", *labels],
        loc="upper right",
        fontsize=9,
    )

    # 全局圆域尺度远大于定位多边形，因此添加局部特写而不牺牲检测点布局信息。
    detail = inset_axes(
        axis,
        width="43%",
        height="43%",
        loc="lower right",
        borderpad=1.2,
    )
    detail.fill(
        result.polygon[:, 0],
        result.polygon[:, 1],
        color="#E15759",
        alpha=0.45,
    )
    _draw_local_wedge_boundaries(detail, case)
    detail.add_patch(
        Circle(
            result.circle_center,
            result.circle_radius_m,
            fill=False,
            edgecolor="#E15759",
            linewidth=1.5,
            label="直径圆",
            zorder=3,
        )
    )
    detail.plot(
        [result.diameter.first[0], result.diameter.second[0]],
        [result.diameter.first[1], result.diameter.second[1]],
        color="#7A1FA2",
        linewidth=2.0,
        marker="o",
    )
    detail.scatter(*case.target, marker="*", s=90, color="#D62728")
    center = np.mean(result.polygon, axis=0)
    span = max(float(np.ptp(result.polygon[:, 0])), float(np.ptp(result.polygon[:, 1])))
    half_width = max(0.7 * span, 25.0)
    detail.set_xlim(center[0] - half_width, center[0] + half_width)
    detail.set_ylim(center[1] - half_width, center[1] + half_width)
    detail.set_aspect("equal")
    detail.set_title("交汇区域特写", fontsize=9)
    detail.grid(alpha=0.2)
    detail.tick_params(labelsize=7)
    mark_inset(axis, detail, loc1=2, loc2=4, fc="none", ec="#777777", lw=0.8)
    _save_figure(figure, output)


def plot_rotating_calipers(output: Path) -> None:
    """用两幅解析图展示对踵点推进和最终直径端点。"""

    configure_chinese_font()
    polygon = np.array(
        [
            [-4.0, -1.0],
            [-2.0, -3.0],
            [2.0, -2.5],
            [4.2, 0.2],
            [2.2, 3.2],
            [-2.2, 2.6],
        ],
        dtype=float,
    )
    diameter = rotating_calipers_diameter(polygon)
    figure, axes = plt.subplots(1, 2, figsize=(12.0, 5.2))
    closed = np.vstack((polygon, polygon[0]))
    for axis in axes:
        axis.fill(polygon[:, 0], polygon[:, 1], color="#9AC9DB", alpha=0.35)
        axis.plot(closed[:, 0], closed[:, 1], color="#2878B5", marker="o")
        for index, point in enumerate(polygon):
            axis.annotate(
                f"$P_{index}$",
                point,
                xytext=(5, 5),
                textcoords="offset points",
            )
        axis.set_aspect("equal")
        # 本图解释算法关系而非数值位置，去除坐标轴以突出卡壳线和对踵点。
        axis.axis("off")

    edge_start, edge_end = polygon[0], polygon[1]
    edge = edge_end - edge_start
    relative = polygon - edge_start
    areas = np.abs(edge[0] * relative[:, 1] - edge[1] * relative[:, 0])
    opposite = polygon[int(np.argmax(areas))]
    normal = np.array([-edge[1], edge[0]]) / np.linalg.norm(edge)
    tangent = edge / np.linalg.norm(edge)
    for point, style in ((edge_start, "-"), (opposite, "--")):
        line = np.vstack((point - 7 * tangent, point + 7 * tangent))
        axes[0].plot(
            line[:, 0],
            line[:, 1],
            style,
            color="#E15759",
            linewidth=1.8,
        )
    projection = edge_start + np.dot(opposite - edge_start, tangent) * tangent
    axes[0].annotate(
        "",
        xy=opposite,
        xytext=projection,
        arrowprops={"arrowstyle": "<->", "color": "#7A1FA2", "lw": 1.8},
    )
    label_point = (opposite + projection) / 2 + 0.2 * normal
    axes[0].text(*label_point, "$h(i,j)$", color="#7A1FA2")
    axes[0].set_title("(a) 平行卡壳与对踵点：面积增大则推进 j")

    axes[1].plot(
        [diameter.first[0], diameter.second[0]],
        [diameter.first[1], diameter.second[1]],
        color="#E15759",
        linewidth=2.8,
        marker="o",
    )
    midpoint = (diameter.first + diameter.second) / 2
    delta = diameter.second - diameter.first
    angle = degrees(atan2(float(delta[1]), float(delta[0])))
    axes[1].annotate(
        f"最大顶点距 D={diameter.distance:.2f}\n方向角约 {angle:.1f}°",
        midpoint,
        xytext=(12, 18),
        textcoords="offset points",
        arrowprops={"arrowstyle": "->", "color": "#333333"},
    )
    axes[1].set_title("(b) 卡壳旋转一周后取得凸多边形直径")
    figure.suptitle("旋转卡壳算法解析图例", fontsize=15)
    figure.tight_layout()
    _save_figure(figure, output)


def plot_validation_cases(cases: Sequence[LocalizationCase], output: Path) -> None:
    """上排绘制检测点全局布局，下排放大相应定位区域。"""

    configure_chinese_font()
    figure, axes = plt.subplots(
        2,
        len(cases),
        figsize=(15.0, 9.2),
    )
    for column, case in enumerate(cases):
        result = localize(case.measurements)
        overview = axes[0, column]
        detail = axes[1, column]

        _draw_arena(overview)
        _draw_measurements(overview, case)
        overview.add_patch(
            Polygon(
                result.polygon,
                closed=True,
                facecolor="#E15759",
                edgecolor="#B22222",
                alpha=0.45,
                label="交汇定位区域",
            )
        )
        overview.scatter(
            *case.target,
            marker="*",
            color="#D62728",
            s=100,
            label="验证真值 G",
        )
        overview.plot(
            [result.diameter.first[0], result.diameter.second[0]],
            [result.diameter.first[1], result.diameter.second[1]],
            color="#7A1FA2",
            linewidth=1.8,
            label="区域直径",
        )
        overview.set_title(f"{case.name}：检测点全局布局")
        overview.set_xlim(-1900, 1900)
        overview.set_ylim(-1900, 1900)
        overview.set_aspect("equal")
        overview.grid(alpha=0.15)
        overview.set_xlabel("x / m")
        if column == 0:
            overview.set_ylabel("y / m")
        handles, labels = overview.get_legend_handles_labels()
        overview.legend(
            [_wedge_boundary_handle("-"), *handles],
            ["±1° 扇形边界", *labels],
            fontsize=6.8,
            loc="lower left",
            ncol=2,
        )

        _draw_local_wedge_boundaries(detail, case)
        detail.fill(
            result.polygon[:, 0],
            result.polygon[:, 1],
            color="#E15759",
            edgecolor="#B22222",
            alpha=0.45,
        )
        detail.plot(
            [result.diameter.first[0], result.diameter.second[0]],
            [result.diameter.first[1], result.diameter.second[1]],
            color="#7A1FA2",
            linewidth=2.0,
            marker="o",
            label="区域直径",
        )
        detail.add_patch(
            Circle(
                result.circle_center,
                result.circle_radius_m,
                fill=False,
                edgecolor="#E15759",
                linewidth=1.5,
                label="直径圆",
                zorder=3,
            )
        )
        detail.scatter(
            *case.target,
            marker="*",
            color="#D62728",
            s=100,
            label="验证真值 G",
            zorder=4,
        )
        for index, point in enumerate(result.polygon):
            detail.annotate(
                f"P{index}",
                point,
                xytext=(4, 4),
                textcoords="offset points",
                fontsize=8,
            )
        center = np.mean(result.polygon, axis=0)
        span = max(
            float(np.ptp(result.polygon[:, 0])),
            float(np.ptp(result.polygon[:, 1])),
        )
        half_width = max(0.65 * span, 20.0)
        detail.set_xlim(center[0] - half_width, center[0] + half_width)
        detail.set_ylim(center[1] - half_width, center[1] + half_width)
        detail.set_title(
            f"交汇区域特写：面积={result.area_m2:.0f} m²，"
            f"D={result.diameter.distance:.1f} m"
        )
        detail.set_aspect("equal")
        detail.grid(alpha=0.2)
        detail.set_xlabel("x / m")
        if column == 0:
            detail.set_ylabel("y / m")
        handles, labels = detail.get_legend_handles_labels()
        detail.legend(
            [_wedge_boundary_handle("--"), *handles],
            ["扇形边界虚射线", *labels],
            fontsize=6.8,
            loc="best",
            ncol=2,
        )
    figure.suptitle("问题 1：不同检测点几何布局的算法验证", fontsize=14)
    figure.tight_layout()
    _save_figure(figure, output)


def plot_counterexample(case: LocalizationCase, output: Path) -> None:
    """绘制由三次示向度交会真实形成的直径圆反例。

    左图给出检测点、示向度和圆域中的交汇位置，右图放大交汇四边形、直径圆及
    圆外顶点；图内同时列出可复现输入坐标与示向度。
    """

    configure_chinese_font()
    result = localize(case.measurements)
    polygon = result.polygon
    diameter = result.diameter
    center = result.circle_center
    radius = result.circle_radius_m
    excess = result.maximum_circle_excess_m

    figure, axes = plt.subplots(1, 2, figsize=(13.0, 6.0))
    overview, detail = axes
    _draw_arena(overview)
    _draw_measurements(overview, case)
    overview.add_patch(
        Polygon(
            polygon,
            closed=True,
            facecolor="#E15759",
            edgecolor="#B22222",
            alpha=0.65,
            label="交汇四边形",
        )
    )
    overview.scatter(*case.target, marker="*", s=130, color="#D62728", label="验证真值 G")
    input_text = "\n".join(
        f"S{i}=({m.station[0]:.0f}, {m.station[1]:.0f}),  θ{i}={m.bearing_deg:.3f}°"
        for i, m in enumerate(case.measurements, 1)
    )
    overview.text(
        0.02,
        0.02,
        input_text,
        transform=overview.transAxes,
        fontsize=9,
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.9},
    )
    overview.set_title("(a) 三个检测点及其 ±1° 示向度交会")
    overview.set_xlabel("x / m")
    overview.set_ylabel("y / m")
    overview.set_xlim(-1_900, 1_900)
    overview.set_ylim(-1_900, 1_900)
    overview.set_aspect("equal")
    overview.grid(alpha=0.2)
    handles, labels = overview.get_legend_handles_labels()
    overview.legend(
        [_wedge_boundary_handle("-"), *handles],
        ["±1° 扇形边界", *labels],
        fontsize=8,
        loc="upper left",
    )

    detail.add_patch(
        Polygon(
            polygon,
            closed=True,
            facecolor="#9AC9DB",
            edgecolor="#2878B5",
            alpha=0.55,
        )
    )
    detail.add_patch(
        Circle(
            center,
            radius,
            fill=False,
            edgecolor="#E15759",
            linewidth=2.2,
            label="直径圆",
        )
    )
    _draw_local_wedge_boundaries(detail, case)
    detail.plot(
        [diameter.first[0], diameter.second[0]],
        [diameter.first[1], diameter.second[1]],
        color="#7A1FA2",
        linewidth=2.4,
        marker="o",
        label="选取的一对直径端点",
    )
    distances = np.linalg.norm(polygon - center, axis=1)
    outside_index = int(np.argmax(distances - radius))
    for index, point in enumerate(polygon):
        detail.annotate(
            f"P{index}\n({point[0]:.2f}, {point[1]:.2f})",
            point,
            xytext=(6, 6),
            textcoords="offset points",
            fontsize=8,
        )
    detail.scatter(
        *polygon[outside_index],
        facecolors="none",
        edgecolors="#D62728",
        s=170,
        linewidths=2.0,
        label="圆外顶点",
    )
    padding = 25.0
    detail.set_xlim(
        float(np.min(polygon[:, 0]) - padding),
        float(np.max(polygon[:, 0]) + padding),
    )
    detail.set_ylim(
        float(np.min(polygon[:, 1]) - padding),
        float(np.max(polygon[:, 1]) + padding),
    )
    detail.set_title(
        "(b) 交汇区域局部放大\n"
        f"D={diameter.distance:.3f} m，圆半径={radius:.3f} m，"
        f"圆外超出={excess:.3f} m"
    )
    detail.set_xlabel("x / m")
    detail.set_ylabel("y / m")
    detail.set_aspect("equal")
    detail.grid(alpha=0.2)
    handles, labels = detail.get_legend_handles_labels()
    detail.legend(
        [_wedge_boundary_handle("--"), *handles],
        ["扇形边界虚射线", *labels],
        fontsize=8,
    )
    figure.suptitle("由实际示向度交会形成的“直径圆不能覆盖定位区域”反例", fontsize=14)
    figure.tight_layout()
    _save_figure(figure, output)
