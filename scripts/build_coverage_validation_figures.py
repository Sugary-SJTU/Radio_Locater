"""生成论文用的全域覆盖几何验证图。"""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager
from matplotlib.lines import Line2D
from matplotlib.patches import Circle


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "res" / "figures" / "problem3" / "paper"
OUT.mkdir(parents=True, exist_ok=True)

ARENA_R = 1800.0
DETECT_R = 1000.0


def set_style() -> None:
    for path in (Path(r"C:\Windows\Fonts\msyh.ttc"), Path(r"C:\Windows\Fonts\simhei.ttf")):
        if path.exists():
            font_manager.fontManager.addfont(str(path))
            plt.rcParams["font.family"] = font_manager.FontProperties(fname=str(path)).get_name()
            break
    plt.rcParams.update(
        {
            "axes.unicode_minus": False,
            "font.size": 13,
            "axes.linewidth": 0.8,
            "figure.facecolor": "white",
            "savefig.facecolor": "white",
        }
    )


def stations(n: int, radius: float) -> np.ndarray:
    theta = np.arange(n) * 2 * np.pi / n
    outer = np.c_[radius * np.cos(theta), radius * np.sin(theta)]
    return np.vstack(([0.0, 0.0], outer))


def base_plot(ax, pts: np.ndarray, polygon_color: str) -> None:
    ax.add_patch(Circle((0, 0), ARENA_R, fill=False, lw=2.1, color="#203B60", zorder=4))
    for x, y in pts:
        ax.add_patch(
            Circle(
                (x, y),
                DETECT_R,
                facecolor="#78AEE3",
                edgecolor="#4A8CCB",
                lw=0.75,
                alpha=0.10,
                zorder=1,
            )
        )
    outer = pts[1:]
    closed = np.vstack((outer, outer[0]))
    ax.plot(closed[:, 0], closed[:, 1], "--", lw=1.35, color=polygon_color, zorder=3)
    ax.scatter(pts[:, 0], pts[:, 1], s=33, color=polygon_color, zorder=5)
    for i, (x, y) in enumerate(pts):
        offset_x = 40 if x >= -1 else -110
        offset_y = 45 if y >= -1 else -95
        ax.text(x + offset_x, y + offset_y, f"$P_{i}$", color=polygon_color, fontsize=12.5, zorder=6)
    ax.set_aspect("equal")
    ax.set_xlim(-2025, 2025)
    ax.set_ylim(-2025, 2025)
    ax.set_xlabel("$x$/m")
    ax.set_ylabel("$y$/m")
    ax.set_xticks([-1800, -900, 0, 900, 1800])
    ax.set_yticks([-1800, -900, 0, 900, 1800])
    ax.grid(lw=0.45, alpha=0.24)


def save(fig, stem: str) -> None:
    fig.savefig(OUT / f"{stem}.png", dpi=360, bbox_inches="tight")
    fig.savefig(OUT / f"{stem}.pdf", bbox_inches="tight")
    fig.savefig(OUT / f"{stem}.svg", bbox_inches="tight")
    plt.close(fig)


def draw_hexagon() -> None:
    a = 1130.0
    pts = stations(6, a)
    fig, ax = plt.subplots(figsize=(7.1, 6.6))
    base_plot(ax, pts, "#C64444")

    theta = np.pi / 6
    critical = ARENA_R * np.array([np.cos(theta), np.sin(theta)])
    ax.scatter(*critical, marker="*", s=105, color="#E79A18", edgecolor="#875700", zorder=7)
    ax.plot(
        [pts[1, 0], critical[0]],
        [pts[1, 1], critical[1]],
        ":",
        lw=1.2,
        color="#875700",
        zorder=6,
    )
    ax.annotate(
        "$d_{\max}=996.95$ m\n$<1000$ m",
        xy=critical,
        xytext=(820, 1180),
        ha="left",
        va="center",
        color="#714900",
        arrowprops={"arrowstyle": "->", "lw": 1.0, "color": "#875700"},
        fontsize=13,
        zorder=8,
    )
    ax.text(
        0,
        -1935,
        "结论：全域覆盖",
        ha="center",
        va="center",
        fontsize=14.5,
        color="#206A45",
        fontweight="bold",
    )
    ax.set_title("中心＋正六边形（$a=1130$ m）", pad=10)
    handles = [
        Line2D([0], [0], color="#203B60", lw=2.1, label="目标区域  $R=1800$ m"),
        Line2D([0], [0], marker="o", color="none", markerfacecolor="#78AEE3", markeredgecolor="#4A8CCB", label="保证检测半径  $r=1000$ m"),
    ]
    ax.legend(handles=handles, loc="lower center", bbox_to_anchor=(0.5, -0.165), ncol=2, frameon=False, fontsize=11.5)
    fig.tight_layout()
    save(fig, "hexagon-1130-coverage")


def draw_pentagon_failure() -> None:
    n = 5
    optimal_a = ARENA_R * np.cos(np.pi / n)
    pts = stations(n, optimal_a)
    fig, ax = plt.subplots(figsize=(7.1, 6.6))

    # 将测距超过1000 m的区域浅红标出，直观显示五个边界缺口。
    grid = np.linspace(-ARENA_R, ARENA_R, 520)
    xx, yy = np.meshgrid(grid, grid)
    inside = xx**2 + yy**2 <= ARENA_R**2
    dist = np.full_like(xx, np.inf, dtype=float)
    for sx, sy in pts:
        dist = np.minimum(dist, np.hypot(xx - sx, yy - sy))
    uncovered = np.where(inside & (dist > DETECT_R), 1.0, np.nan)
    ax.contourf(xx, yy, uncovered, levels=[0.5, 1.5], colors=["#E45A5A"], alpha=0.27, zorder=0)

    base_plot(ax, pts, "#B33B3B")

    theta = np.pi / n
    critical = ARENA_R * np.array([np.cos(theta), np.sin(theta)])
    ax.scatter(*critical, marker="X", s=88, color="#D43D3D", edgecolor="#741F1F", zorder=8)
    ax.plot(
        [pts[1, 0], critical[0]],
        [pts[1, 1], critical[1]],
        ":",
        lw=1.2,
        color="#741F1F",
        zorder=6,
    )
    ax.annotate(
        "最佳仍有\n$d^*_{\max}=1058.01$ m $>1000$ m",
        xy=critical,
        xytext=(690, 1570),
        ha="left",
        va="center",
        color="#8D2626",
        arrowprops={"arrowstyle": "->", "lw": 1.0, "color": "#8D2626"},
        fontsize=12.5,
        zorder=9,
    )
    ax.text(
        0,
        -1935,
        "结论：边界存在漏检区",
        ha="center",
        va="center",
        fontsize=14.5,
        color="#A52E2E",
        fontweight="bold",
    )
    ax.set_title("中心＋正五边形（最优 $a=1456.23$ m）", pad=10)
    handles = [
        Line2D([0], [0], color="#203B60", lw=2.1, label="目标区域  $R=1800$ m"),
        Line2D([0], [0], marker="o", color="none", markerfacecolor="#78AEE3", markeredgecolor="#4A8CCB", label="检测圆  $r=1000$ m"),
        Line2D([0], [0], marker="s", color="none", markerfacecolor="#E45A5A", alpha=0.45, label="漏检区"),
    ]
    ax.legend(handles=handles, loc="lower center", bbox_to_anchor=(0.5, -0.165), ncol=3, frameon=False, fontsize=10.5)
    fig.tight_layout()
    save(fig, "pentagon-coverage-failure")


if __name__ == "__main__":
    set_style()
    draw_hexagon()
    draw_pentagon_failure()
