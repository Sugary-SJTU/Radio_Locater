"""问题 1 的交会定位、凸多边形直径和直径圆覆盖判定。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np
from numpy.typing import NDArray

from problems.problem1.config import (
    ANGLE_ERROR_DEG,
    ARENA_POLYGON_VERTICES,
    DOMAIN_RADIUS_M,
    GEOMETRY_TOLERANCE,
)
from radio_locator.geometry import (
    DiameterResult,
    bearing_deg,
    diameter_circle_coverage,
    intersect_bearing_wedges,
    polygon_area,
    rotating_calipers_diameter,
)

FloatArray = NDArray[np.float64]


@dataclass(frozen=True, slots=True)
class BearingMeasurement:
    """一次检测点坐标及其示向度。"""

    station: tuple[float, float]
    bearing_deg: float


@dataclass(frozen=True, slots=True)
class LocalizationResult:
    """问题 1 的完整计算结果。"""

    polygon: FloatArray
    area_m2: float
    diameter: DiameterResult
    circle_covers_polygon: bool
    circle_center: FloatArray
    circle_radius_m: float
    maximum_circle_excess_m: float


def localize(
    measurements: Sequence[BearingMeasurement],
    error_deg: float = ANGLE_ERROR_DEG,
) -> LocalizationResult:
    """将多次示向度约束求交，并计算区域直径和直径圆覆盖结果。"""

    if len(measurements) < 2:
        raise ValueError("at least two bearing measurements are required")
    stations = [measurement.station for measurement in measurements]
    bearings = [measurement.bearing_deg for measurement in measurements]
    polygon = intersect_bearing_wedges(
        stations,
        bearings,
        error_deg,
        DOMAIN_RADIUS_M,
        ARENA_POLYGON_VERTICES,
        GEOMETRY_TOLERANCE,
    )
    if len(polygon) == 0:
        raise ValueError("bearing wedges have an empty intersection")
    diameter = rotating_calipers_diameter(polygon)
    covers, center, radius, excess = diameter_circle_coverage(
        polygon,
        diameter,
        GEOMETRY_TOLERANCE,
    )
    return LocalizationResult(
        polygon=polygon,
        area_m2=polygon_area(polygon),
        diameter=diameter,
        circle_covers_polygon=covers,
        circle_center=center,
        circle_radius_m=radius,
        maximum_circle_excess_m=excess,
    )


def measurements_from_target(
    stations: Sequence[Sequence[float]],
    target: Sequence[float],
    reading_errors_deg: Sequence[float] | None = None,
) -> list[BearingMeasurement]:
    """由已知测试目标构造含确定性读数误差的验证数据。

    此函数只用于可复现实验；真实问题中 target 未知，输入应来自给定检测数据。
    """

    if reading_errors_deg is None:
        reading_errors_deg = [0.0] * len(stations)
    if len(stations) != len(reading_errors_deg):
        raise ValueError("stations and reading_errors_deg must have equal lengths")
    return [
        BearingMeasurement(
            station=(float(station[0]), float(station[1])),
            bearing_deg=(bearing_deg(station, target) + error) % 360.0,
        )
        for station, error in zip(stations, reading_errors_deg, strict=True)
    ]

