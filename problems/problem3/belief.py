"""问题 3 的 `(Z_j,I_j,R_j)` 联合粒子信念与单频道互信息。

每个存在粒子在初始化时一次性取得位置和接收半径；后续所有检测只重加权，不重新
抽取半径，因此严格保留同一干扰源 `R_j` 在一局内固定的相关性。
"""

from __future__ import annotations

from dataclasses import dataclass
from math import ceil

import numpy as np
from numpy.typing import NDArray

from config.constants import BEARING_ERROR_DEG, NEAR_DISTANCE_M
from problems.problem3.config import Problem3Settings

FloatArray = NDArray[np.float64]


@dataclass(slots=True)
class ChannelBelief:
    """单个频道关于存在性、位置和固定接收半径的离散联合分布。"""

    exists: NDArray[np.bool_]
    points: FloatArray
    radii_m: FloatArray
    weights: FloatArray
    direction_bin_deg: float

    @classmethod
    def initialize(
        cls,
        settings: Problem3Settings,
        channel: int,
    ) -> "ChannelBelief":
        """按存在先验和目标圆域均匀位置先验生成可复现粒子。"""

        count = settings.particle_count_per_channel
        if count < 20:
            raise ValueError("particle_count_per_channel must be at least 20")
        rng = np.random.default_rng(settings.seed * 10_007 + channel * 997)
        existing_count = max(1, min(count - 1, round(count * settings.existence_prior)))
        absent_count = count - existing_count
        radial = settings.arena_radius_m * np.sqrt(rng.random(existing_count))
        angle = rng.uniform(0.0, 2.0 * np.pi, existing_count)
        existing_points = np.column_stack((radial * np.cos(angle), radial * np.sin(angle)))
        points = np.vstack((existing_points, np.zeros((absent_count, 2))))
        radii = np.concatenate(
            (
                rng.uniform(
                    settings.guaranteed_radius_m,
                    1_500.0,
                    existing_count,
                ),
                np.zeros(absent_count),
            )
        )
        exists = np.concatenate(
            (np.ones(existing_count, dtype=bool), np.zeros(absent_count, dtype=bool))
        )
        weights = np.concatenate(
            (
                np.full(existing_count, settings.existence_prior / existing_count),
                np.full(absent_count, (1.0 - settings.existence_prior) / absent_count),
            )
        )
        return cls(exists, points, radii, weights, settings.direction_bin_deg)

    def copy(self) -> "ChannelBelief":
        """复制权重和固定粒子，用于MPC虚拟观测而不污染真实状态。"""

        return ChannelBelief(
            self.exists.copy(),
            self.points.copy(),
            self.radii_m.copy(),
            self.weights.copy(),
            self.direction_bin_deg,
        )

    @property
    def entropy_bits(self) -> float:
        """返回联合粒子离散熵。"""

        positive = self.weights[self.weights > 0.0]
        return float(-np.sum(positive * np.log2(positive)))

    @property
    def existence_probability(self) -> float:
        """返回 `P(Z_j=1)`。"""

        return float(np.sum(self.weights[self.exists]))

    def _distances(self, position: tuple[float, float]) -> FloatArray:
        """计算所有粒子位置到候选检测点的距离。"""

        return np.linalg.norm(self.points - np.asarray(position, dtype=float), axis=1)

    def predicted_labels(self, position: tuple[float, float]) -> NDArray[np.int64]:
        """为每个粒子生成 none、near 或离散direction观测标签。"""

        distances = self._distances(position)
        received = self.exists & (distances <= self.radii_m + 1e-9)
        labels = np.zeros(len(self.weights), dtype=np.int64)  # 0 = none
        labels[received & (distances <= NEAR_DISTANCE_M + 1e-9)] = 1  # 1 = near
        directional = received & (distances > NEAR_DISTANCE_M + 1e-9)
        vectors = self.points - np.asarray(position, dtype=float)
        bearings = np.degrees(np.arctan2(vectors[:, 1], vectors[:, 0])) % 360.0
        bin_count = int(ceil(360.0 / self.direction_bin_deg))
        direction_bins = np.floor(bearings / self.direction_bin_deg).astype(int) % bin_count
        labels[directional] = direction_bins[directional] + 2
        return labels

    def information_gain_bits(self, position: tuple[float, float]) -> float:
        """计算确定性离散观测模型下 `I(xi_j;Y|M)=H(Y)`。"""

        labels = self.predicted_labels(position)
        masses = np.bincount(labels, weights=self.weights)
        positive = masses[masses > 0.0]
        return float(-np.sum(positive * np.log2(positive)))

    def detection_probability(self, position: tuple[float, float]) -> float:
        """返回下一次观测不是none的概率，供p0粗筛和动作日志解释。"""

        labels = self.predicted_labels(position)
        return float(np.sum(self.weights[labels != 0]))

    def update(
        self,
        position: tuple[float, float],
        result: str,
        svd_deg: float | None = None,
    ) -> None:
        """按真实接口的 none/near/direction结果重加权当前频道。"""

        distances = self._distances(position)
        received = self.exists & (distances <= self.radii_m + 1e-9)
        if result == "no_signal":
            compatible = ~received
        elif result == "near":
            compatible = received & (distances <= NEAR_DISTANCE_M + 1e-9)
        elif result == "direction" and svd_deg is not None:
            vectors = self.points - np.asarray(position, dtype=float)
            bearings = np.degrees(np.arctan2(vectors[:, 1], vectors[:, 0])) % 360.0
            difference = np.abs((bearings - svd_deg + 180.0) % 360.0 - 180.0)
            compatible = received & (distances > NEAR_DISTANCE_M) & (
                difference <= BEARING_ERROR_DEG + 0.011
            )
        else:
            raise ValueError("invalid measurement observation")
        # 极小污染概率避免有限粒子偶然没有命中角度带时数值崩溃。
        likelihood = np.where(compatible, 1.0, 1e-8)
        self.weights *= likelihood
        total = float(np.sum(self.weights))
        if total <= 0.0:
            raise RuntimeError("particle belief lost all probability mass")
        self.weights /= total

    def credible_points(self, probability: float = 0.95) -> FloatArray:
        """返回存在粒子中累计权重达到给定概率的最高权重点集。"""

        indices = np.flatnonzero(self.exists)
        if not len(indices):
            return np.empty((0, 2), dtype=float)
        existing_weights = self.weights[indices]
        total = float(np.sum(existing_weights))
        if total <= 0.0:
            return np.empty((0, 2), dtype=float)
        order = indices[np.argsort(existing_weights)[::-1]]
        normalized = self.weights[order] / total
        end = int(np.searchsorted(np.cumsum(normalized), probability, side="left")) + 1
        return self.points[order[:end]].copy()

    def representative_point(self) -> tuple[float, float]:
        """返回条件于存在的加权平均位置。"""

        mask = self.exists
        mass = float(np.sum(self.weights[mask]))
        if mass <= 0.0:
            return 0.0, 0.0
        point = np.sum(self.points[mask] * self.weights[mask, None], axis=0) / mass
        return float(point[0]), float(point[1])

    def estimated_localization_measurements(
        self,
        clear_entropy_bits: float,
        mean_information_gain_bits: float,
        epsilon_bits: float,
    ) -> int:
        """实现 `ceil((H-H_clear)/max(mean_IG,eps))` 的剩余检测次数估计。"""

        remaining = max(self.entropy_bits - clear_entropy_bits, 0.0)
        return int(ceil(remaining / max(mean_information_gain_bits, epsilon_bits)))
