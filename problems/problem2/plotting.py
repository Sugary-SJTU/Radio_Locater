"""问题 2 的粗筛原理图和第二检测点选择结果图。"""

from __future__ import annotations

from pathlib import Path

from radio_locator.runtime import prepare_matplotlib_config

prepare_matplotlib_config()

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Annulus, Circle, Polygon

from problems.problem1.plotting import configure_chinese_font
from problems.problem2.config import (
    DOMAIN_RADIUS_M,
    GUARANTEED_RECEPTION_RADIUS_M,
    MAX_RECEPTION_RADIUS_M,
    MIN_DETECTION_PROBABILITY,
    MIN_GEOMETRY_SCORE,
    MIN_SECOND_POINT_DISTANCE_M,
)
from problems.problem2.model import (
    SecondRegionDiameterSample,
    SelectionResult,
    reception_probability,
)


def _save_figure(figure: plt.Figure, output: Path) -> None:
    """创建结果目录并保存高分辨率白底图片。"""

    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=220, bbox_inches="tight", facecolor="white")
    plt.close(figure)


def plot_screening_principles(output: Path) -> None:
    """分别解释粗筛使用的四项原则，并突出 1000--1500 m 过渡区。"""

    configure_chinese_font()
    figure, axes = plt.subplots(2, 2, figsize=(12.0, 10.0))

    # 原理一：第二点必须离开第一点 5 m 禁区，避免完全重复检测环境。
    first = np.array([0.0, 0.0])
    axes[0, 0].add_patch(
        Circle(first, 5.0, color="#E15759", alpha=0.30, label="5 m 禁止区")
    )
    axes[0, 0].scatter(*first, color="#2878B5", s=70, label="第一检测点 $M_1$")
    axes[0, 0].scatter(8.0, 5.0, color="#59A14F", s=70, label="允许的 $M_2$")
    axes[0, 0].annotate("距离必须大于 5 m", (4.0, 2.5), fontsize=10)
    axes[0, 0].set_xlim(-12, 15)
    axes[0, 0].set_ylim(-12, 15)
    axes[0, 0].set_aspect("equal")
    axes[0, 0].set_title("(a) 近距排除原则")
    axes[0, 0].legend(fontsize=9)
    axes[0, 0].axis("off")

    # 原理二：显示论文的分段接收概率；过渡区用斜线填充，与必然区明显区分。
    distances = np.linspace(0.0, 1_700.0, 500)
    probabilities = reception_probability(distances)
    axes[0, 1].plot(distances, probabilities, color="#2878B5", linewidth=2.3)
    axes[0, 1].axvspan(
        0,
        GUARANTEED_RECEPTION_RADIUS_M,
        color="#59A14F",
        alpha=0.18,
        label="0–1000 m：必然接收",
    )
    transition = axes[0, 1].axvspan(
        GUARANTEED_RECEPTION_RADIUS_M,
        MAX_RECEPTION_RADIUS_M,
        facecolor="#F28E2B",
        alpha=0.18,
        hatch="///",
        edgecolor="#F28E2B",
        label="1000–1500 m：概率过渡区",
    )
    transition.set_linewidth(0.0)
    axes[0, 1].axvline(1_500, color="#E15759", linestyle="--")
    axes[0, 1].set_xlabel("检测距离 d / m")
    axes[0, 1].set_ylabel("接收概率 p(d)")
    axes[0, 1].set_ylim(-0.05, 1.08)
    axes[0, 1].set_title("(b) 未知接收半径下的检测概率")
    axes[0, 1].legend(fontsize=9)

    # 原理三：并列画出一个通过和一个被淘汰的候选点，明确展示集合相交条件。
    first_region = np.array(
        [[-350.0, -80.0], [900.0, 160.0], [1_150.0, 520.0], [-250.0, 120.0]]
    )
    feasible_candidate = np.array([950.0, -800.0])
    rejected_candidate = np.array([-2_000.0, -1_300.0])
    witness = np.array([900.0, 160.0])
    nearest = np.array([-350.0, -80.0])
    axes[1, 0].add_patch(
        Polygon(first_region, color="#9AC9DB", alpha=0.5, label="首次定位区域 $S_1$")
    )
    axes[1, 0].add_patch(
        Circle(
            feasible_candidate,
            1_500.0,
            fill=False,
            linestyle="--",
            linewidth=2.0,
            edgecolor="#59A14F",
            label="通过：1500 m 圆与 $S_1$ 相交",
        )
    )
    axes[1, 0].add_patch(
        Circle(
            rejected_candidate,
            1_500.0,
            fill=False,
            linestyle=":",
            linewidth=2.0,
            edgecolor="#9B9B9B",
            label="淘汰：1500 m 圆与 $S_1$ 不相交",
        )
    )
    axes[1, 0].scatter(
        *feasible_candidate,
        color="#59A14F",
        s=55,
        marker="o",
    )
    axes[1, 0].scatter(
        *rejected_candidate,
        color="#9B9B9B",
        s=65,
        marker="x",
    )
    axes[1, 0].scatter(*witness, color="#D62728", s=35, zorder=4)
    axes[1, 0].annotate(
        "$d(M_2^{(a)},X_k)<1500$ m\n存在可接收位置，保留",
        xy=witness,
        xytext=(1_250, 900),
        arrowprops={"arrowstyle": "->", "color": "#59A14F"},
        color="#26733A",
        fontsize=9,
    )
    axes[1, 0].annotate(
        "$d(M_2^{(b)},S_1)>1500$ m\n不存在可接收位置，淘汰",
        xy=nearest,
        xytext=(-2_800, 450),
        arrowprops={"arrowstyle": "->", "color": "#777777"},
        color="#555555",
        fontsize=9,
    )
    axes[1, 0].text(
        feasible_candidate[0] + 80,
        feasible_candidate[1] - 100,
        "$M_2^{(a)}$",
        color="#26733A",
    )
    axes[1, 0].text(
        rejected_candidate[0] - 220,
        rejected_candidate[1] - 130,
        "$M_2^{(b)}$",
        color="#666666",
    )
    axes[1, 0].set_xlim(-3_300, 2_600)
    axes[1, 0].set_ylim(-2_700, 2_000)
    axes[1, 0].set_aspect("equal")
    axes[1, 0].set_title("(c) 1500 m 可达性：保留相交候选、淘汰不相交候选")
    axes[1, 0].legend(fontsize=7.5, loc="lower right")
    axes[1, 0].axis("off")

    # 原理四：用定量曲线同时展示退化交会、最优正交和论文阈值对应区间。
    gamma = np.linspace(0.0, 180.0, 721)
    geometry_quality = np.abs(np.sin(np.radians(gamma)))
    axes[1, 1].plot(gamma, geometry_quality, color="#7A1FA2", linewidth=2.3)
    axes[1, 1].axhline(
        MIN_GEOMETRY_SCORE,
        color="#E15759",
        linestyle="--",
        linewidth=1.5,
        label=f"筛选阈值 $g_0={MIN_GEOMETRY_SCORE:.1f}$",
    )
    axes[1, 1].axvspan(
        30.0,
        150.0,
        color="#59A14F",
        alpha=0.16,
        label="合格角度区间 30°–150°",
    )
    axes[1, 1].scatter(90.0, 1.0, color="#D62728", s=45, zorder=4)
    axes[1, 1].annotate(
        "90°：正交交会最优",
        xy=(90.0, 1.0),
        xytext=(108.0, 0.90),
        arrowprops={"arrowstyle": "->", "color": "#D62728"},
        fontsize=9,
    )
    axes[1, 1].text(4.0, 0.08, "近 0°：射线近同向\n定位区域狭长")
    axes[1, 1].text(137.0, 0.08, "近 180°：射线近反向\n同样发生退化")
    axes[1, 1].set_xlim(0.0, 180.0)
    axes[1, 1].set_ylim(0.0, 1.08)
    axes[1, 1].set_xticks(np.arange(0.0, 181.0, 30.0))
    axes[1, 1].set_xlabel("交会角 γ / °")
    axes[1, 1].set_ylabel("单点几何质量 |sin γ|")
    axes[1, 1].set_title("(d) 交会角质量：避免近 0° 或 180° 的退化几何")
    axes[1, 1].legend(fontsize=8, loc="lower center")
    axes[1, 1].grid(alpha=0.2)

    # 分段概率图和交会角质量图依赖定量坐标，几何集合示意图不显示坐标轴。
    axes[0, 1].grid(alpha=0.2)
    figure.suptitle("问题 2：第二检测点粗筛的四项原理", fontsize=16)
    figure.tight_layout()
    _save_figure(figure, output)


def plot_candidate_region(
    result: SelectionResult,
    output: Path,
) -> None:
    """绘制粗筛候选点、首次后验、细化结果和最终第二检测点。"""

    configure_chinese_font()
    figure, axis = plt.subplots(figsize=(9.2, 8.0))
    axis.add_patch(
        Circle(
            (0.0, 0.0),
            DOMAIN_RADIUS_M,
            fill=False,
            linestyle="--",
            edgecolor="#586174",
            linewidth=1.4,
            label="_nolegend_",
        )
    )
    axis.add_patch(
        Polygon(
            result.posterior.first_region,
            closed=True,
            facecolor="#9AC9DB",
            edgecolor="#2878B5",
            alpha=0.20,
            label="示向度可行区域 $S_1$",
        )
    )
    posterior_size = (
        3 + 12 * result.posterior.weights / np.max(result.posterior.weights)
    )
    axis.scatter(
        result.posterior.points[:, 0],
        result.posterior.points[:, 1],
        s=posterior_size,
        color="#B4230D",
        alpha=0.38,
        linewidths=0,
        label="位置离散后验",
    )

    rejected = np.asarray(
        [score.point for score in result.coarse_scores if not score.feasible]
    )
    feasible = [score for score in result.coarse_scores if score.feasible]
    feasible_points = np.asarray([score.point for score in feasible])
    feasible_values = np.asarray([score.proxy_score for score in feasible])
    if len(rejected):
        axis.scatter(
            rejected[:, 0],
            rejected[:, 1],
            s=3,
            color="#B8B8B8",
            alpha=0.30,
            label="_nolegend_",
        )
    scatter = axis.scatter(
        feasible_points[:, 0],
        feasible_points[:, 1],
        c=feasible_values,
        cmap="viridis",
        s=11,
        label="粗筛候选区域",
        zorder=3,
    )
    figure.colorbar(scatter, ax=axis, shrink=0.72, label="粗筛代理得分")

    fine_points = np.asarray([score.point for score in result.fine_scores])
    axis.scatter(
        fine_points[:, 0],
        fine_points[:, 1],
        facecolors="none",
        edgecolors="#7A1FA2",
        s=13,
        linewidths=0.55,
        label="_nolegend_",
    )
    # 论文式 (57) 的 5% 近优集合可能含多个连通分量；半透明凸包只概括离散
    # 细网格点的范围，实际点仍保存在 CSV 中供人工核查。
    excellent_label_used = False
    for hull in result.excellent_region_hulls:
        label = "5% 近优区域" if not excellent_label_used else "_nolegend_"
        if len(hull) >= 3:
            axis.add_patch(
                Polygon(
                    hull,
                    closed=True,
                    facecolor="#C9A0DC",
                    edgecolor="#7A1FA2",
                    linewidth=1.8,
                    alpha=0.25,
                    label=label,
                    zorder=4,
                )
            )
        else:
            axis.plot(
                hull[:, 0],
                hull[:, 1],
                color="#7A1FA2",
                linewidth=3.0,
                marker="o",
                label=label,
                zorder=4,
            )
        excellent_label_used = True
    first = result.first_measurement.station
    selected = result.selected.point
    maximum_score_point = result.maximum_score.point
    axis.scatter(
        *first,
        marker="s",
        s=60,
        color="#1F4E79",
        label="第一检测点 $M_1$",
    )
    axis.scatter(
        *maximum_score_point,
        marker="*",
        s=145,
        color="#F2B134",
        edgecolor="#7A4E00",
        linewidth=0.8,
        label="得分最大点",
        zorder=6,
    )
    axis.annotate(
        "目标函数最大",
        xy=maximum_score_point,
        xytext=(maximum_score_point[0] + 130.0, maximum_score_point[1] + 120.0),
        arrowprops={"arrowstyle": "->", "color": "#7A4E00"},
        color="#7A4E00",
        fontsize=8.5,
    )
    axis.scatter(
        *selected,
        marker="X",
        s=90,
        color="#E15759",
        edgecolor="white",
        label="式 (59) 最终点 $M_2^*$",
        zorder=6,
    )
    axis.plot(
        [first[0], selected[0]],
        [first[1], selected[1]],
        color="#E15759",
        linestyle=":",
        linewidth=1.6,
    )

    # 对固定候选点 M2，所有位置假设 X_k 按其到 M2 的距离落入不同接收层。
    # 因此接收区以选定检测点为圆心，而不是以某个预设干扰源真值为圆心。
    reception_center = np.asarray(selected, dtype=float)
    axis.add_patch(
        Circle(
            reception_center,
            GUARANTEED_RECEPTION_RADIUS_M,
            facecolor="#59A14F",
            edgecolor="#2F7F3F",
            alpha=0.08,
            linewidth=1.2,
            label="_nolegend_",
        )
    )
    axis.add_patch(
        Annulus(
            reception_center,
            MAX_RECEPTION_RADIUS_M,
            width=MAX_RECEPTION_RADIUS_M - GUARANTEED_RECEPTION_RADIUS_M,
            facecolor="none",
            edgecolor="#F28E2B",
            hatch="///",
            linewidth=1.1,
            alpha=0.65,
            label="_nolegend_",
        )
    )
    axis.annotate(
        "相对 $M_2^*$：0–1000 m\n所有 $X_k$ 均可接收",
        xy=reception_center + np.array([780.0, 0.0]),
        xytext=(350.0, 1_000.0),
        arrowprops={"arrowstyle": "->", "color": "#2F7F3F"},
        fontsize=8,
        color="#26733A",
    )
    axis.annotate(
        "相对 $M_2^*$：1000–1500 m\n各 $X_k$ 按 $p(d_{2k})$ 加权",
        xy=reception_center + np.array([0.0, 1_250.0]),
        xytext=(300.0, 1_600.0),
        arrowprops={"arrowstyle": "->", "color": "#F28E2B"},
        fontsize=8,
        color="#B86400",
    )
    axis.text(
        200.0,
        -1_500.0,
        "$P_{det}(M_2)=\\sum_k \\pi_k q_{2k}(M_2)$\n"
        "$G_\\gamma(M_2)="
        "\\sum_k \\pi_k q_{2k}|\\sin\\gamma_k|/P_{det}$",
        fontsize=8.5,
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.85},
    )
    axis.add_patch(
        Circle(
            first,
            MIN_SECOND_POINT_DISTANCE_M,
            color="#E15759",
            alpha=0.35,
        )
    )
    gain = result.selected.expected_log_diameter_gain
    axis.set_title(
        "问题 2：粗筛—细化得到的第二检测点候选区域\n"
        f"式 (59) 最终点={selected}，期望对数直径增益={gain:.3f} bit"
    )
    axis.set_xlabel("x / m（正东）")
    axis.set_ylabel("y / m（正北）")
    axis.set_xlim(-2_050, 2_050)
    axis.set_ylim(-2_050, 2_050)
    axis.set_aspect("equal")
    axis.grid(alpha=0.18)
    axis.legend(loc="lower left", fontsize=8, ncol=2)
    _save_figure(figure, output)


def plot_selected_m2_diameter_histogram(
    samples: tuple[SecondRegionDiameterSample, ...],
    selected_point: tuple[float, float],
    output: Path,
) -> None:
    """绘制固定 M2 下，S1 密网格形成的 S2 直径柱状图和区间饼图。"""

    configure_chinese_font()
    diameters = np.asarray([sample.diameter_m for sample in samples], dtype=float)
    raw_weights = np.asarray(
        [sample.integration_weight for sample in samples], dtype=float
    )
    weights = raw_weights / np.sum(raw_weights)
    bin_count = min(16, max(8, int(np.sqrt(len(samples)))))
    counts, edges = np.histogram(diameters, bins=bin_count, weights=weights)
    centers = (edges[:-1] + edges[1:]) / 2.0
    widths = np.diff(edges)
    weighted_mean = float(np.dot(weights, diameters))
    source_count = len({sample.source_point for sample in samples})

    figure, (axis, pie_axis) = plt.subplots(
        1,
        2,
        figsize=(13.2, 5.8),
        gridspec_kw={"width_ratios": [1.65, 1.0]},
    )
    axis.bar(
        centers,
        counts,
        width=0.88 * widths,
        color="#4C78A8",
        edgecolor="white",
        linewidth=0.8,
        label="检测条件下的加权概率质量",
    )
    axis.axvline(
        weighted_mean,
        color="#E15759",
        linestyle="--",
        linewidth=2.0,
        label=f"加权平均直径 {weighted_mean:.2f} m",
    )
    axis.set_title("(a) $D_2$ 加权柱状分布")
    axis.set_xlabel("第二次测向后多边形直径 $D_2$ / m")
    axis.set_ylabel("归一化概率质量")
    axis.grid(axis="y", alpha=0.22)
    axis.legend()

    # 饼图按直径数值区间汇总，而不是给每条记录单独画扇区。分界点覆盖定位中常见
    # 的小、中、大直径，并将极端长尾单列，方便比较各尺度所占概率质量。
    pie_edges = np.array([0.0, 40.0, 60.0, 80.0, 120.0, np.inf])
    pie_labels = ["<40 m", "40–60 m", "60–80 m", "80–120 m", "≥120 m"]
    pie_values = np.array(
        [
            np.sum(
                weights[
                    (diameters >= lower)
                    & (diameters < upper)
                ]
            )
            for lower, upper in zip(pie_edges[:-1], pie_edges[1:], strict=True)
        ]
    )
    nonzero = pie_values > 1e-12
    pie_axis.pie(
        pie_values[nonzero],
        labels=np.asarray(pie_labels)[nonzero],
        autopct="%1.1f%%",
        startangle=90,
        counterclock=False,
        colors=["#4C78A8", "#72B7B2", "#F2CF5B", "#F28E2B", "#E15759"],
        wedgeprops={"edgecolor": "white", "linewidth": 1.0},
        textprops={"fontsize": 9},
    )
    pie_axis.set_title("(b) 按直径大小分组的概率占比")
    figure.suptitle(
        "选定 $M_2$ 后定位多边形直径分布\n"
        f"$M_2={selected_point}$；S1 干扰源网格点 {source_count} 个，"
        f"含测向误差共 {len(samples)} 条记录",
        fontsize=15,
    )
    figure.tight_layout()
    _save_figure(figure, output)
