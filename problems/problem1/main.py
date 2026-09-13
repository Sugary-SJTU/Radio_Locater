"""问题 1 的算法验证与论文插图生成入口。

运行后计算三组代表性测试，交叉验证旋转卡壳与暴力枚举结果，输出 CSV 和四幅图。
"""

from __future__ import annotations

import csv
import json

from config.paths import paired_pdf_path
from problems.problem1.config import (
    CALIPERS_FIGURE,
    COUNTEREXAMPLE_FIGURE,
    HALF_PLANE_CONVENTION_FIGURE,
    INTERSECTION_DETAIL_FIGURE,
    INTERSECTION_FIGURE,
    RESULT_TABLE,
    VALIDATION_FIGURE,
)
from problems.problem1.examples import validation_cases
from problems.problem1.model import localize
from problems.problem1.plotting import (
    plot_counterexample,
    plot_half_plane_convention,
    plot_intersection_polygon_detail,
    plot_rotating_calipers,
    plot_three_station_intersection,
    plot_validation_cases,
)
from radio_locator.geometry import brute_force_diameter


def main() -> None:
    """运行问题 1 示例、保存验证指标并生成全部图像。"""

    cases = validation_cases()
    rows: list[dict[str, object]] = []
    for case in cases:
        result = localize(case.measurements)
        brute_force = brute_force_diameter(result.polygon)
        difference = abs(result.diameter.distance - brute_force.distance)
        if difference > 1e-7:
            raise AssertionError("rotating calipers disagrees with brute force")
        rows.append(
            {
                "case": case.name,
                "measurement_count": len(case.measurements),
                "station_coordinates": json.dumps(
                    [measurement.station for measurement in case.measurements],
                    ensure_ascii=False,
                ),
                "bearings_deg": json.dumps(
                    [
                        round(measurement.bearing_deg, 9)
                        for measurement in case.measurements
                    ]
                ),
                "polygon_vertices": json.dumps(
                    result.polygon.round(9).tolist(),
                    ensure_ascii=False,
                ),
                "vertex_count": len(result.polygon),
                "area_m2": f"{result.area_m2:.6f}",
                "calipers_diameter_m": f"{result.diameter.distance:.9f}",
                "brute_force_diameter_m": f"{brute_force.distance:.9f}",
                "absolute_difference_m": f"{difference:.3e}",
                "diameter_circle_covers": result.circle_covers_polygon,
                "maximum_circle_excess_m": (
                    f"{result.maximum_circle_excess_m:.6f}"
                ),
            }
        )

    RESULT_TABLE.parent.mkdir(parents=True, exist_ok=True)
    with RESULT_TABLE.open("w", encoding="utf-8-sig", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    first_result = localize(cases[0].measurements)
    plot_three_station_intersection(cases[0], first_result, INTERSECTION_FIGURE)
    plot_intersection_polygon_detail(cases[0], first_result, INTERSECTION_DETAIL_FIGURE)
    plot_rotating_calipers(CALIPERS_FIGURE)
    plot_validation_cases(cases, VALIDATION_FIGURE)
    # 第三个验证案例是由三个检测点交会得到的直径圆反例。
    plot_counterexample(cases[2], COUNTEREXAMPLE_FIGURE)
    plot_half_plane_convention(HALF_PLANE_CONVENTION_FIGURE)

    print(f"问题 1 验证表：{RESULT_TABLE}")
    for output in (
        INTERSECTION_FIGURE,
        INTERSECTION_DETAIL_FIGURE,
        CALIPERS_FIGURE,
        VALIDATION_FIGURE,
        COUNTEREXAMPLE_FIGURE,
        HALF_PLANE_CONVENTION_FIGURE,
    ):
        print(f"问题 1 PNG 图像：{output}")
        print(f"问题 1 PDF 图像：{paired_pdf_path(output)}")


if __name__ == "__main__":
    main()
