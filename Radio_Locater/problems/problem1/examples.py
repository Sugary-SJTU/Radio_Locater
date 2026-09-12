"""问题 1 的可复现测试点与验证案例。"""

from __future__ import annotations

from dataclasses import dataclass

from problems.problem1.model import BearingMeasurement, measurements_from_target


@dataclass(frozen=True, slots=True)
class LocalizationCase:
    """一个已知真实位置的定位验证案例。"""

    name: str
    target: tuple[float, float]
    measurements: tuple[BearingMeasurement, ...]


def validation_cases() -> tuple[LocalizationCase, ...]:
    """返回良好交会、弱交会和直径圆反例三类代表性测试。

    第三个案例由三次合法示向度约束真实求交得到，不是任意构造的凸多边形。
    其交汇四边形的一对直径端点所成圆不能覆盖另一个顶点。
    """

    specifications = (
        (
            "三站良好交会",
            (350.0, 220.0),
            ((-1000.0, -700.0), (1100.0, -650.0), (-650.0, 1100.0)),
            (0.65, -0.45, 0.30),
        ),
        (
            "同侧弱交会",
            (520.0, 80.0),
            ((-1200.0, -280.0), (-850.0, 20.0), (-550.0, 350.0)),
            (-0.60, 0.20, 0.75),
        ),
        (
            "三站交汇反例",
            (1500.0, 620.0),
            ((0.0, 0.0), (850.0, -850.0), (280.0, 1350.0)),
            (0.40, -0.70, 0.15),
        ),
    )
    cases: list[LocalizationCase] = []
    for name, target, stations, errors in specifications:
        measurements = measurements_from_target(stations, target, errors)
        cases.append(LocalizationCase(name, target, tuple(measurements)))
    return tuple(cases)
