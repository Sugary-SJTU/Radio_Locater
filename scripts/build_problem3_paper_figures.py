"""生成问题三论文正文使用的两张图。"""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager
from matplotlib.patches import Circle


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "res" / "figures" / "problem3" / "paper"
OUT.mkdir(parents=True, exist_ok=True)


def configure_style() -> None:
    font_candidates = [
        Path(r"C:\Windows\Fonts\msyh.ttc"),
        Path(r"C:\Windows\Fonts\simhei.ttf"),
    ]
    for path in font_candidates:
        if path.exists():
            font_manager.fontManager.addfont(str(path))
            plt.rcParams["font.family"] = font_manager.FontProperties(fname=str(path)).get_name()
            break
    plt.rcParams.update(
        {
            "axes.unicode_minus": False,
            "font.size": 10.5,
            "axes.linewidth": 0.8,
            "figure.facecolor": "white",
            "savefig.facecolor": "white",
        }
    )


def coverage_figure() -> None:
    arena_radius = 1800.0
    station_radius = 1150.0
    guaranteed_radius = 1000.0
    angles = np.arange(6) * np.pi / 3
    vertices = np.c_[station_radius * np.cos(angles), station_radius * np.sin(angles)]
    stations = np.vstack(([0.0, 0.0], vertices))

    fig, ax = plt.subplots(figsize=(7.2, 6.6))
    ax.add_patch(Circle((0, 0), arena_radius, fill=False, lw=2.0, color="#172A46", label="目标区域边界（R=1800 m）"))
    for idx, (x, y) in enumerate(stations):
        ax.add_patch(
            Circle(
                (x, y),
                guaranteed_radius,
                facecolor="#4C9BE8",
                edgecolor="#2673B8",
                lw=0.8,
                alpha=0.10,
            )
        )
        ax.scatter(x, y, s=34, color="#C43D3D", zorder=4)
        ax.text(x + 38, y + 38, f"$P_{idx}$", color="#7A2020", fontsize=10)

    closed_vertices = np.vstack((vertices, vertices[0]))
    ax.plot(closed_vertices[:, 0], closed_vertices[:, 1], "--", lw=1.25, color="#C43D3D", label="六边形访问骨架（a=1150 m）")

    boundary_angle = np.pi / 6
    critical = arena_radius * np.array([np.cos(boundary_angle), np.sin(boundary_angle)])
    ax.scatter(*critical, marker="*", s=95, color="#F2A900", edgecolor="#7A5200", zorder=5)
    ax.plot([vertices[0, 0], critical[0]], [vertices[0, 1], critical[1]], ":", color="#7A5200", lw=1.1)
    ax.text(critical[0] - 535, critical[1] + 70, "边界最不利方位\n距最近测站 988.51 m", color="#6B4900", fontsize=9.5)

    ax.set_aspect("equal")
    ax.set_xlim(-2050, 2050)
    ax.set_ylim(-2050, 2050)
    ax.set_xlabel("$x$/m")
    ax.set_ylabel("$y$/m")
    ax.grid(lw=0.45, alpha=0.28)
    ax.legend(loc="lower center", bbox_to_anchor=(0.5, -0.17), ncol=2, frameon=False)
    fig.tight_layout()
    fig.savefig(OUT / "problem3_coverage_geometry.png", dpi=320, bbox_inches="tight")
    fig.savefig(OUT / "problem3_coverage_geometry.pdf", bbox_inches="tight")
    plt.close(fig)


def ablation_figure() -> None:
    radius_labels = ["1200", "1175", "1150", "1130"]
    radius_means = [3800.28, 3752.47, 3699.96, 3701.27]
    modes = ["legacy", "exact", "probe"]
    mode_means = [3717.74, 3697.15, 3528.87]

    fig, axes = plt.subplots(1, 2, figsize=(9.0, 3.7))
    colors = ["#AFC7E8", "#7EA8D8", "#2E78B7", "#5B91C3"]
    bars = axes[0].bar(radius_labels, radius_means, color=colors, width=0.68)
    axes[0].set_title("六边形半径消融（legacy）")
    axes[0].set_xlabel("六边形半径/m")
    axes[0].set_ylabel("平均虚拟时间/s")
    axes[0].set_ylim(3400, 3900)
    axes[0].bar_label(bars, fmt="%.2f", padding=3, fontsize=9)

    mode_colors = ["#AFC7E8", "#7EA8D8", "#E58A3A"]
    bars = axes[1].bar(modes, mode_means, color=mode_colors, width=0.68)
    axes[1].set_title("末段规则消融（a=1150 m）")
    axes[1].set_xlabel("末段调度模式")
    axes[1].set_ylabel("平均虚拟时间/s")
    axes[1].set_ylim(3400, 3800)
    axes[1].bar_label(bars, fmt="%.2f", padding=3, fontsize=9)
    axes[1].annotate(
        "较 legacy 降低 5.08%",
        xy=(2, mode_means[2]),
        xytext=(1.1, 3590),
        arrowprops={"arrowstyle": "->", "lw": 1.0, "color": "#9B4F0F"},
        color="#9B4F0F",
        fontsize=9.5,
    )

    for ax in axes:
        ax.spines[["top", "right"]].set_visible(False)
        ax.grid(axis="y", lw=0.45, alpha=0.25)
        ax.set_axisbelow(True)
    fig.tight_layout(w_pad=2.4)
    fig.savefig(OUT / "problem3_ablation.png", dpi=320, bbox_inches="tight")
    fig.savefig(OUT / "problem3_ablation.pdf", bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    configure_style()
    coverage_figure()
    ablation_figure()

