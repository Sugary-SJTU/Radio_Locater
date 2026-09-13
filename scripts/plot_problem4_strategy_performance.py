"""从现有问题四基准表绘制总耗时与实际全源清除率联合图。"""

from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]

from radio_locator.runtime import prepare_matplotlib_config

prepare_matplotlib_config()

import matplotlib.pyplot as plt
import numpy as np

from config.paths import PROBLEM4_FIGURES_DIR, paired_pdf_path
from problems.problem1.plotting import configure_chinese_font, style_figure_for_paper


def main() -> None:
    source = ROOT / "res/tables/problem4/problem4_strategy_comparison.json"
    document = json.loads(source.read_text(encoding="utf-8"))
    names = [
        "shifted_triangular_lattice",
        "concentric_ring_no_insertion",
        "concentric_ring_joint",
        "legacy_outer_probe_fast",
    ]
    labels = [
        "平移三角格",
        "同心环\n无顺路清除",
        "同心环\n联合插入",
        "外侧补测\n安全兜底",
    ]
    means = [document["variants"][name]["mean"] for name in names]
    times = np.asarray([item["total_time_s"] for item in means])
    success = np.asarray([item["all_sources_cleared_rate"] * 100.0 for item in means])

    configure_chinese_font()
    figure, axis = plt.subplots(figsize=(10.2, 5.3))
    x = np.arange(len(labels))
    bars = axis.bar(
        x,
        times,
        width=0.62,
        color=("#8CB9D9", "#6EA6D7", "#3E7CB1", "#C98773"),
        edgecolor="black",
        linewidth=0.9,
        label="平均总耗时",
    )
    axis.bar_label(
        bars, labels=[f"{value:.0f} s" for value in times], padding=4, fontsize=8.5
    )
    axis.set_ylabel("平均总耗时 / s")
    axis.set_xticks(x, labels)
    axis.set_ylim(0.0, max(times) * 1.18)
    axis.grid(axis="y", color="#D0D0D0", linewidth=0.65, alpha=0.75)
    axis.set_axisbelow(True)

    rate_axis = axis.twinx()
    rate_axis.plot(
        x,
        success,
        color="#9C2F2F",
        marker="D",
        markersize=5.5,
        linewidth=1.6,
        label="实际全源清除率",
    )
    for x_value, value in zip(x, success, strict=True):
        rate_axis.annotate(
            f"{value:.0f}%",
            (x_value, value),
            xytext=(0, 8),
            textcoords="offset points",
            ha="center",
            color="#822626",
            fontsize=8.5,
        )
    rate_axis.set_ylabel("实际全源清除率 / %", color="#822626")
    rate_axis.tick_params(axis="y", labelcolor="#822626")
    rate_axis.set_ylim(0.0, 112.0)

    handles1, labels1 = axis.get_legend_handles_labels()
    handles2, labels2 = rate_axis.get_legend_handles_labels()
    figure.subplots_adjust(top=0.82)
    figure.legend(
        handles1 + handles2,
        labels1 + labels2,
        ncol=2,
        loc="upper center",
        bbox_to_anchor=(0.5, 0.965),
        frameon=False,
    )
    for current_axis in (axis, rate_axis):
        for spine in current_axis.spines.values():
            spine.set_color("black")
            spine.set_linewidth(1.0)
    style_figure_for_paper(figure)
    output = PROBLEM4_FIGURES_DIR / "problem4_strategy_time_and_clearance.png"
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(
        output, dpi=300, bbox_inches="tight", facecolor="white", edgecolor="black"
    )
    pdf = paired_pdf_path(output)
    pdf.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(pdf, bbox_inches="tight", facecolor="white", edgecolor="black")
    plt.close(figure)


if __name__ == "__main__":
    main()
