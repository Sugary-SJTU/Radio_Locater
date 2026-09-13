"""问题 2 的第二检测点选择与论文插图生成入口。"""

from __future__ import annotations

import csv
import json
from math import isfinite

import numpy as np

from config.paths import paired_pdf_path
from problems.problem1.model import BearingMeasurement
from problems.problem2.config import (
    DIAMETER_HISTOGRAM,
    DIAMETER_SAMPLE_GRID_STEP_M,
    DIAMETER_SAMPLE_TABLE,
    INTERSECTION_ANGLE_CONSTRAINT_FIGURE,
    M1_POSTERIOR_GRID_FIGURE,
    NEAR_OPTIMAL_RELATIVE_GAP,
    REGION_SUMMARY,
    RESULT_FIGURE,
    RESULT_TABLE,
    SCREENING_FIGURES,
)
from problems.problem2.model import (
    CandidateScore,
    SelectionResult,
    build_posterior_grid,
    sample_selected_second_region_diameters,
    select_second_point,
)

# 可复现实验输入只有第一检测点和实际读到的示向度，不设干扰源真实位置。
DEFAULT_FIRST_X_M = -900.0
DEFAULT_FIRST_Y_M = -500.0
DEFAULT_FIRST_BEARING_DEG = 31.363757


def main(
    first_x_m: float = DEFAULT_FIRST_X_M,
    first_y_m: float = DEFAULT_FIRST_Y_M,
    first_bearing_deg: float = DEFAULT_FIRST_BEARING_DEG,
) -> SelectionResult:
    """按指定的第一检测点及其示向度完成第二检测点选址。"""

    if not all(isfinite(value) for value in (first_x_m, first_y_m, first_bearing_deg)):
        raise ValueError("first point coordinates and bearing must be finite")
    first_measurement = BearingMeasurement(
        station=(float(first_x_m), float(first_y_m)),
        bearing_deg=float(first_bearing_deg % 360.0),
    )

    # 延迟加载绘图模块，使数值模型和测试在无图形环境中仍可独立导入。
    from problems.problem2.plotting import (
        plot_candidate_region,
        plot_first_detection_posterior_grid,
        plot_intersection_angle_constraints,
        plot_screening_principles,
        plot_selected_m2_diameter_histogram,
    )

    result = select_second_point(first_measurement)
    excellent_points = {score.point for score in result.excellent_scores}
    rows: list[dict[str, object]] = []

    def append_score(phase: str, score: CandidateScore) -> None:
        """将 CandidateScore 转为适合人工检查的扁平 CSV 行。"""

        rows.append(
            {
                "phase": phase,
                "x_m": f"{score.point[0]:.6f}",
                "y_m": f"{score.point[1]:.6f}",
                "reachable": score.reachable,
                "detection_probability": f"{score.detection_probability:.9f}",
                "geometry_score": f"{score.geometry_score:.9f}",
                "passes_coarse_screen": score.feasible,
                "proxy_score": f"{score.proxy_score:.9f}",
                "expected_log_diameter_gain": (
                    ""
                    if score.expected_log_diameter_gain is None
                    else f"{score.expected_log_diameter_gain:.9f}"
                ),
                # 同一坐标可能同时出现在粗、细网格中；只在最终细化行标记一次。
                "selected": (
                    phase == "fine" and score.point == result.selected.point
                ),
                "in_excellent_region": (
                    phase == "fine" and score.point in excellent_points
                ),
            }
        )

    for score in result.coarse_scores:
        append_score("coarse", score)
    for score in result.fine_scores:
        append_score("fine", score)

    RESULT_TABLE.parent.mkdir(parents=True, exist_ok=True)
    with RESULT_TABLE.open("w", encoding="utf-8-sig", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    dense_diameter_posterior = build_posterior_grid(
        first_measurement,
        grid_step_m=DIAMETER_SAMPLE_GRID_STEP_M,
    )
    diameter_samples = sample_selected_second_region_diameters(
        result,
        posterior=dense_diameter_posterior,
    )
    with DIAMETER_SAMPLE_TABLE.open(
        "w", encoding="utf-8-sig", newline=""
    ) as file:
        writer = csv.DictWriter(
            file,
            fieldnames=[
                "source_x_m",
                "source_y_m",
                "source_posterior_weight",
                "conditional_detection_probability",
                "bearing_error_deg",
                "second_bearing_deg",
                "polygon_diameter_m",
                "integration_weight",
            ],
        )
        writer.writeheader()
        for sample in diameter_samples:
            writer.writerow(
                {
                    "source_x_m": f"{sample.source_point[0]:.6f}",
                    "source_y_m": f"{sample.source_point[1]:.6f}",
                    "source_posterior_weight": (
                        f"{sample.source_posterior_weight:.12f}"
                    ),
                    "conditional_detection_probability": (
                        f"{sample.conditional_detection_probability:.12f}"
                    ),
                    "bearing_error_deg": f"{sample.bearing_error_deg:.6f}",
                    "second_bearing_deg": f"{sample.second_bearing_deg:.9f}",
                    "polygon_diameter_m": f"{sample.diameter_m:.9f}",
                    "integration_weight": f"{sample.integration_weight:.12f}",
                }
            )

    excellent_array = np.asarray(
        [score.point for score in result.excellent_scores], dtype=float
    )
    maximum_score_point = result.maximum_score
    summary = {
        "file_function": "记录指定第一检测点输入、5%近优区域坐标范围及最佳第二检测点。",
        "first_measurement": {
            "x_m": first_measurement.station[0],
            "y_m": first_measurement.station[1],
            "bearing_deg": first_measurement.bearing_deg,
        },
        "excellent_definition": (
            f"expected_gain >= {1.0 - NEAR_OPTIMAL_RELATIVE_GAP:.2f} * maximum_gain"
        ),
        "excellent_point_count": len(result.excellent_scores),
        "coarse_refinement_centers_m": [
            list(score.point) for score in result.coarse_refinement_centers
        ],
        "overall_bounds_m": {
            "x_min": float(np.min(excellent_array[:, 0])),
            "x_max": float(np.max(excellent_array[:, 0])),
            "y_min": float(np.min(excellent_array[:, 1])),
            "y_max": float(np.max(excellent_array[:, 1])),
        },
        "components": [
            {
                "component": index,
                "bounds_m": {
                    "x_min": float(np.min(hull[:, 0])),
                    "x_max": float(np.max(hull[:, 0])),
                    "y_min": float(np.min(hull[:, 1])),
                    "y_max": float(np.max(hull[:, 1])),
                },
                "hull_vertices_m": hull.tolist(),
            }
            for index, hull in enumerate(result.excellent_region_hulls, start=1)
        ],
        "maximum_score_point": {
            "x_m": maximum_score_point.point[0],
            "y_m": maximum_score_point.point[1],
            "expected_log_diameter_gain_bit": (
                maximum_score_point.expected_log_diameter_gain
            ),
        },
        "selected_second_point": {
            "x_m": result.selected.point[0],
            "y_m": result.selected.point[1],
            "detection_probability": result.selected.detection_probability,
            "geometry_score": result.selected.geometry_score,
            "expected_log_diameter_gain_bit": (
                result.selected.expected_log_diameter_gain
            ),
        },
    }
    with REGION_SUMMARY.open("w", encoding="utf-8") as file:
        json.dump(summary, file, ensure_ascii=False, indent=2)

    plot_screening_principles(SCREENING_FIGURES)
    plot_first_detection_posterior_grid(
        first_measurement, result.posterior, M1_POSTERIOR_GRID_FIGURE
    )
    plot_intersection_angle_constraints(INTERSECTION_ANGLE_CONSTRAINT_FIGURE)
    plot_candidate_region(result, RESULT_FIGURE)
    plot_selected_m2_diameter_histogram(
        diameter_samples,
        result.selected.point,
        DIAMETER_HISTOGRAM,
    )
    bounds = summary["overall_bounds_m"]
    print(f"第一检测点：{first_measurement.station}")
    print(f"第一示向度：{first_measurement.bearing_deg:.6f}°")
    print(
        "5% 近优区域总包围范围："
        f"x=[{bounds['x_min']:.1f}, {bounds['x_max']:.1f}] m，"
        f"y=[{bounds['y_min']:.1f}, {bounds['y_max']:.1f}] m；"
        f"共 {len(result.excellent_region_hulls)} 个连通分量、"
        f"{len(result.excellent_scores)} 个细网格点"
    )
    print(
        "细网格得分最大点："
        f"{maximum_score_point.point}，"
        f"增益={maximum_score_point.expected_log_diameter_gain:.4f} bit"
    )
    print(f"论文式 (59) 最终第二检测点：{result.selected.point}")
    print(
        "选定点指标："
        f"Pdet={result.selected.detection_probability:.4f}, "
        f"Gγ={result.selected.geometry_score:.4f}, "
        f"期望对数直径增益={result.selected.expected_log_diameter_gain:.4f} bit"
    )
    print(f"问题 2 候选表：{RESULT_TABLE}")
    print(f"问题 2 优良区域：{REGION_SUMMARY}")
    print(
        f"直径分布密网格：{len(dense_diameter_posterior.points)} 个干扰源位置，"
        f"共 {len(diameter_samples)} 条位置—误差记录"
    )
    print(f"固定 M2 的多边形直径表：{DIAMETER_SAMPLE_TABLE}")
    labelled_outputs = [
        ("固定 M2 的直径柱状图", DIAMETER_HISTOGRAM),
        *(
            (f"问题 2 粗筛原理图 {index}", output)
            for index, output in enumerate(SCREENING_FIGURES, start=1)
        ),
        ("M1 检测区域离散采样图", M1_POSTERIOR_GRID_FIGURE),
        ("交会角约束示意图", INTERSECTION_ANGLE_CONSTRAINT_FIGURE),
        ("问题 2 结果图", RESULT_FIGURE),
    ]
    for label, output in labelled_outputs:
        print(f"{label} PNG：{output}")
        print(f"{label} PDF：{paired_pdf_path(output)}")
    return result


if __name__ == "__main__":
    main()
