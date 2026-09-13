"""问题3离散联合信念 ``P(Z_j,I_j,R_j|H_t)``。

初始原子来自粗空间网格与离散半径网格；空间细分时子原子继承父原子的半径，所以
一次任务中的 ``R_j`` 始终固定。低概率原子只标记 inactive，安全覆盖证明不依赖剪枝。
"""

from __future__ import annotations

from dataclasses import dataclass
from math import ceil
from typing import Any

import numpy as np
from numpy.typing import NDArray

from config.constants import BEARING_ERROR_DEG, NEAR_DISTANCE_M
from problems.problem3.config import Problem3Settings
from problems.problem3.coverage import arena_grid
from radio_locator.geometry import (
    convex_hull,
    intersect_bearing_wedges,
    minimum_enclosing_circle,
    polygon_area,
    rotating_calipers_diameter,
)

FloatArray = NDArray[np.float64]


@dataclass(frozen=True, slots=True)
class ObservationScenario:
    """MPC虚拟更新所用的一个条件观测分支。"""
    result: str
    probability: float
    bearing_deg: float | None = None


@dataclass(slots=True)
class ChannelBelief:
    """不存在原子和 ``(网格位置,固定接收半径)`` 原子上的联合分布。"""
    exists: NDArray[np.bool_]
    points: FloatArray
    radii_m: FloatArray
    weights: FloatArray
    resolution_m: FloatArray
    active: NDArray[np.bool_]
    direction_bin_deg: float
    prune_threshold: float
    arena_radius_m: float

    @classmethod
    def initialize(cls, settings: Problem3Settings, channel: int) -> "ChannelBelief":
        """可复现地采样“粗空间网格×半径网格”；第0项为 ``Z=0``。"""
        count = settings.particle_count_per_channel
        if count < 20:
            raise ValueError("particle_count_per_channel must be at least 20")
        spatial = arena_grid(settings.arena_radius_m, settings.coarse_grid_size)
        radius_values = np.arange(settings.guaranteed_radius_m, 1500.0 + settings.radius_grid_size / 2, settings.radius_grid_size)
        grid_index, radius_index = np.meshgrid(np.arange(len(spatial)), np.arange(len(radius_values)), indexing="ij")
        pairs = np.column_stack((grid_index.ravel(), radius_index.ravel()))
        target = min(count - 1, len(pairs))
        base_seed = settings.random_seed if settings.random_seed is not None else settings.seed
        rng = np.random.default_rng(base_seed * 10_007 + channel * 997)
        pairs = pairs[rng.choice(len(pairs), target, replace=False)]
        exists = np.concatenate((np.zeros(1, dtype=bool), np.ones(target, dtype=bool)))
        points = np.vstack((np.zeros((1, 2)), spatial[pairs[:, 0]]))
        radii = np.concatenate(([0.0], radius_values[pairs[:, 1]]))
        weights = np.concatenate(([1-settings.existence_prior], np.full(target, settings.existence_prior/target)))
        resolution = np.concatenate(([settings.coarse_grid_size], np.full(target, settings.coarse_grid_size)))
        return cls(exists, points, radii, weights, resolution, np.ones(target+1, dtype=bool), settings.direction_bin_deg, settings.probability_prune_threshold, settings.arena_radius_m)

    def copy(self) -> "ChannelBelief":
        """复制联合状态，供场景树虚拟观测。"""
        return ChannelBelief(self.exists.copy(), self.points.copy(), self.radii_m.copy(), self.weights.copy(), self.resolution_m.copy(), self.active.copy(), self.direction_bin_deg, self.prune_threshold, self.arena_radius_m)

    @property
    def entropy_bits(self) -> float:
        positive = self.weights[self.weights > 0]
        return float(-np.sum(positive*np.log2(positive)))

    @property
    def existence_probability(self) -> float:
        return float(np.sum(self.weights[self.exists]))

    @property
    def effective_particle_count(self) -> float:
        return float(1/max(np.sum(self.weights**2), 1e-300))

    def set_existence_probability(self, probability: float) -> None:
        """只校正两组总质量，不改变条件位置/半径分布。"""
        probability = float(np.clip(probability, 1e-12, 1-1e-12))
        old = self.existence_probability
        if old > 0:
            self.weights[self.exists] *= probability/old
        if old < 1:
            self.weights[~self.exists] *= (1-probability)/(1-old)
        self.weights /= np.sum(self.weights)

    def force_exists(self) -> None:
        """将 ``P(Z=1)`` 设为1，供真实direction/near/clear失败后的确定存在证据使用。"""

        self.weights[~self.exists] = 0.0
        total = float(np.sum(self.weights[self.exists]))
        if total <= 0.0:
            raise RuntimeError("cannot force existence after all existing mass vanished")
        self.weights[self.exists] /= total
        self.active[~self.exists] = False

    def exclude_clear_circle(self, center: tuple[float, float], radius_m: float) -> None:
        """将一次失败clear的圆内位置置零；半径粒子不重抽样。"""

        excluded = self.exists & (self._distances(center) <= radius_m + 1e-9)
        self.weights[excluded] = 0.0
        self.force_exists()

    def restrict_to_bearings(self, observations: list[tuple[tuple[float, float], float]]) -> None:
        """将离散正质量同步裁剪到全部±1°连续测向扇形内。"""

        compatible = self.exists.copy()
        for station, bearing in observations:
            vectors = self.points - np.asarray(station, dtype=float)
            angles = np.degrees(np.arctan2(vectors[:, 1], vectors[:, 0])) % 360.0
            difference = np.abs((angles - bearing + 180.0) % 360.0 - 180.0)
            compatible &= difference <= BEARING_ERROR_DEG + 0.02
        if np.any(compatible):
            self.weights[~compatible] = 0.0
            self.weights[~self.exists] = 0.0
            self.weights /= np.sum(self.weights)
            self.active = compatible
        else:
            # 粗网格可能没有落在1°扇区内：在所有观测射线的近似交会点补一个
            # 合成原子。该原子只服务概率/互信息，确定清除仍由连续support区域判断。
            self.weights[:] = 0.0
            index = int(np.argmax(np.where(self.exists, 1.0, -1.0)))
            fallback_point = self._support_fallback_point(observations)
            self.points[index] = fallback_point
            max_station_distance = max(
                (float(np.linalg.norm(fallback_point - np.asarray(station)))
                 for station, _ in observations),
                default=0.0,
            )
            self.radii_m[index] = float(
                np.clip(max(self.radii_m[index], max_station_distance), 1_000.0, 1_500.0)
            )
            self.weights[index] = 1.0
            self.active[:] = False; self.active[index] = True

    def _support_fallback_point(
        self,
        observations: list[tuple[tuple[float, float], float]],
    ) -> FloatArray:
        """优先取连续示向度交会区域的最小包围圆圆心，退化时用最小二乘交点。"""

        stations = [station for station, _ in observations]
        bearings = [bearing for _, bearing in observations]
        try:
            polygon = intersect_bearing_wedges(
                stations,
                bearings,
                BEARING_ERROR_DEG,
                self.arena_radius_m,
                360,
            )
        except ValueError:
            polygon = np.empty((0, 2), dtype=float)
        if len(polygon):
            circle = minimum_enclosing_circle(polygon)
            return circle.center.copy()

        matrix = np.zeros((2, 2), dtype=float)
        vector = np.zeros(2, dtype=float)
        for station, bearing in observations:
            direction = np.array(
                [np.cos(np.radians(bearing)), np.sin(np.radians(bearing))],
                dtype=float,
            )
            projector = np.eye(2, dtype=float) - np.outer(direction, direction)
            point = np.asarray(station, dtype=float)
            matrix += projector
            vector += projector @ point
        if abs(np.linalg.det(matrix)) > 1e-12:
            return np.linalg.solve(matrix, vector)
        station, bearing = observations[-1]
        direction = np.array(
            [np.cos(np.radians(bearing)), np.sin(np.radians(bearing))],
            dtype=float,
        )
        return np.asarray(station, dtype=float) + 1_000.0 * direction

    def _distances(self, position: tuple[float, float]) -> FloatArray:
        return np.linalg.norm(self.points-np.asarray(position, dtype=float), axis=1)

    def predicted_labels(self, position: tuple[float, float]) -> NDArray[np.int64]:
        """0=none、1=near、2以后为离散示向区间。"""
        distances = self._distances(position)
        received = self.exists & (distances <= self.radii_m+1e-9)
        labels = np.zeros(len(self.weights), dtype=np.int64)
        labels[received & (distances <= NEAR_DISTANCE_M+1e-9)] = 1
        directional = received & (distances > NEAR_DISTANCE_M+1e-9)
        vectors = self.points-np.asarray(position, dtype=float)
        bearings = np.degrees(np.arctan2(vectors[:,1], vectors[:,0])) % 360
        bins = int(ceil(360/self.direction_bin_deg))
        labels[directional] = np.floor(bearings[directional]/self.direction_bin_deg).astype(int) % bins + 2
        return labels

    def information_gain_bits(self, position: tuple[float, float]) -> float:
        """当前历史条件下的互信息；先前none已体现在当前后验权重内。"""
        masses = np.bincount(self.predicted_labels(position), weights=self.weights)
        positive = masses[masses > 0]
        return float(-np.sum(positive*np.log2(positive)))

    def detection_probability(self, position: tuple[float, float]) -> float:
        labels = self.predicted_labels(position)
        return float(np.sum(self.weights[labels != 0]))

    def observation_scenarios(self, position: tuple[float,float], limit: int=5, merge_probability: float=.01) -> list[ObservationScenario]:
        """枚举none、near和主要角区间，并合并低概率角区间。"""
        masses = np.bincount(self.predicted_labels(position), weights=self.weights)
        result: list[ObservationScenario] = []
        if len(masses) and masses[0] > 0: result.append(ObservationScenario("no_signal", float(masses[0])))
        if len(masses)>1 and masses[1] > 0: result.append(ObservationScenario("near", float(masses[1])))
        directions = sorted(((i,float(v)) for i,v in enumerate(masses[2:],2) if v>0), key=lambda x:x[1], reverse=True)
        keep = max(limit-len(result)-1, 0)
        for label,mass in directions[:keep]:
            result.append(ObservationScenario("direction",mass,((label-2)+.5)*self.direction_bin_deg%360))
        tail = directions[keep:]
        tail_mass = sum(v for _,v in tail)
        if tail and tail_mass >= merge_probability and len(result)<limit:
            angles=np.radians([((i-2)+.5)*self.direction_bin_deg for i,_ in tail]); weights=np.asarray([v for _,v in tail])
            angle=np.degrees(np.arctan2(np.sum(weights*np.sin(angles)),np.sum(weights*np.cos(angles))))%360
            result.append(ObservationScenario("direction",tail_mass,float(angle)))
        total=sum(item.probability for item in result)
        return [ObservationScenario(item.result,item.probability/total,item.bearing_deg) for item in result] if total else [ObservationScenario("no_signal",1.0)]

    def update(self, position: tuple[float,float], result: str, svd_deg: float|None=None) -> None:
        """按固定半径接收事件及±1°示向似然进行贝叶斯更新。"""
        distances=self._distances(position); received=self.exists & (distances<=self.radii_m+1e-9)
        if result=="no_signal": compatible=~received
        elif result=="near": compatible=received & (distances<=NEAR_DISTANCE_M+1e-9)
        elif result=="direction" and svd_deg is not None:
            vectors=self.points-np.asarray(position,dtype=float); bearings=np.degrees(np.arctan2(vectors[:,1],vectors[:,0]))%360
            difference=np.abs((bearings-svd_deg+180)%360-180)
            compatible=received & (distances>NEAR_DISTANCE_M) & (difference<=BEARING_ERROR_DEG+.011)
        else: raise ValueError("invalid measurement observation")
        self.weights *= np.where(compatible,1.0,1e-10)
        total=float(np.sum(self.weights))
        if not np.isfinite(total) or total<=0: raise RuntimeError("joint belief lost all finite probability mass")
        self.weights/=total; self.active=self.weights>=self.prune_threshold; self.active[int(np.argmax(self.weights))]=True; self.active[0]=True

    def adaptive_refine(self, step_m: float, maximum_particles: int) -> None:
        """细分高后验位置；所有子粒子继承父粒子的固定半径。"""
        existing=np.flatnonzero(self.exists & self.active)
        if step_m<=0 or not len(existing) or np.min(self.resolution_m[existing])<=step_m: return
        order=existing[np.argsort(self.weights[existing])[::-1]][:max(1,min(len(existing),maximum_particles//9))]
        offsets=step_m*np.asarray([(x,y) for x in (-1,0,1) for y in (-1,0,1)],dtype=float)
        points=[]; radii=[]; weights=[]
        for index in order:
            children=self.points[index]+offsets; children=children[np.linalg.norm(children,axis=1)<=self.arena_radius_m+1e-9]
            if not len(children): continue
            share=self.weights[index]/len(children); self.weights[index]=0
            points.extend(children); radii.extend([float(self.radii_m[index])]*len(children)); weights.extend([float(share)]*len(children))
        if not points: return
        size=len(points); self.exists=np.concatenate((self.exists,np.ones(size,dtype=bool))); self.points=np.vstack((self.points,np.asarray(points)))
        self.radii_m=np.concatenate((self.radii_m,np.asarray(radii))); self.weights=np.concatenate((self.weights,np.asarray(weights)))
        self.resolution_m=np.concatenate((self.resolution_m,np.full(size,step_m))); self.active=np.concatenate((self.active,np.ones(size,dtype=bool)))
        if len(self.weights)>maximum_particles:
            keep=np.zeros(len(self.weights),dtype=bool); keep[0]=True; top=np.argsort(self.weights[1:])[::-1][:maximum_particles-1]+1; keep[top]=True
            self.weights[int(top[0])] += float(np.sum(self.weights[~keep]))
            self.exists,self.points,self.radii_m,self.weights,self.resolution_m,self.active=(array[keep] for array in (self.exists,self.points,self.radii_m,self.weights,self.resolution_m,self.active))
        self.weights/=np.sum(self.weights)

    def credible_indices(self, probability: float=.95) -> NDArray[np.int64]:
        indices=np.flatnonzero(self.exists); mass=float(np.sum(self.weights[indices]))
        if not len(indices) or mass<=0: return np.empty(0,dtype=np.int64)
        order=indices[np.argsort(self.weights[indices])[::-1]]; end=int(np.searchsorted(np.cumsum(self.weights[order]/mass),probability))+1
        return order[:end]

    def credible_points(self, probability: float=.95) -> FloatArray:
        return self.points[self.credible_indices(probability)].copy()

    def representative_point(self) -> tuple[float,float]:
        mask=self.exists; mass=float(np.sum(self.weights[mask]))
        if mass<=0: return (0.,0.)
        point=np.sum(self.points[mask]*self.weights[mask,None],axis=0)/mass; return float(point[0]),float(point[1])

    def map_point(self) -> tuple[float,float]:
        indices=np.flatnonzero(self.exists)
        if not len(indices): return (0.,0.)
        point=self.points[indices[int(np.argmax(self.weights[indices]))]]; return float(point[0]),float(point[1])

    def radius_marginal(self) -> dict[str,float]:
        values={f"{r:.3f}":float(np.sum(self.weights[self.exists & np.isclose(self.radii_m,r)])) for r in np.unique(self.radii_m[self.exists])}
        total=sum(values.values()); return {k:v/total for k,v in values.items()} if total else values

    def position_marginal(self, limit: int=20) -> list[dict[str,float]]:
        masses: dict[tuple[float,float],float]={}
        for p,w in zip(self.points[self.exists],self.weights[self.exists],strict=True): masses[(float(p[0]),float(p[1]))]=masses.get((float(p[0]),float(p[1])),0.)+float(w)
        total=sum(masses.values()); ordered=sorted(masses.items(),key=lambda x:x[1],reverse=True)[:limit]
        return [{"x":p[0],"y":p[1],"probability_given_exists":w/total} for p,w in ordered] if total else []

    def clear_probability_at(self, center: tuple[float,float], radius_m: float=20.) -> float:
        mass=self.existence_probability
        return float(np.sum(self.weights[self.exists & (self._distances(center)<=radius_m+1e-9)])/mass) if mass>0 else 0.

    def geometry_metrics(self, probability: float=.95) -> dict[str,Any]:
        indices=self.credible_indices(probability); points=self.points[indices]
        base={"existence_probability":self.existence_probability,"entropy_bits":self.entropy_bits,"effective_sample_size":self.effective_particle_count,"radius_marginal":self.radius_marginal(),"position_marginal_top":self.position_marginal()}
        if not len(points): return base|{"effective_grid_count":0,"possible_area_m2":0.,"maximum_diameter_m":0.,"minimum_enclosing_radius_m":0.,"minimum_enclosing_center":[0.,0.]}
        hull=convex_hull(points); circle=minimum_enclosing_circle(hull); padding=float(np.max(self.resolution_m[indices])/np.sqrt(2)); unique=len(np.unique(points,axis=0))
        return base|{"effective_grid_count":unique,"possible_area_m2":max(polygon_area(hull),unique*float(np.min(self.resolution_m[indices]))**2),"maximum_diameter_m":rotating_calipers_diameter(hull).distance+2*padding,"minimum_enclosing_radius_m":circle.radius+padding,"minimum_enclosing_center":[float(circle.center[0]),float(circle.center[1])],"grid_resolution_m":float(np.min(self.resolution_m[indices])),"pruned_probability_mass":float(np.sum(self.weights[~self.active]))}

    def estimated_localization_measurements(self, clear_entropy_bits: float, mean_information_gain_bits: float, epsilon_bits: float) -> int:
        return int(ceil(max(self.entropy_bits-clear_entropy_bits,0)/max(mean_information_gain_bits,epsilon_bits)))


def calibrate_existence_count(beliefs: dict[int,ChannelBelief], statuses: dict[int,str], minimum: int=10, maximum: int=16) -> None:
    """公共log-odds校正令存在数期望落入[10,16]，但不指定具体频道。"""
    confirmed=[c for c,s in statuses.items() if s in {"detected","localized","cleared"}]
    for channel in confirmed:
        beliefs[channel].force_exists()
    cleared=sum(s=="cleared" for s in statuses.values())
    # 已direction/near确认存在的频道绝不再参与总数校正，防止其概率被稀释。
    unresolved=[c for c,s in statuses.items() if s=="unknown"]
    if not unresolved: return
    fixed_present=len(confirmed)
    probabilities=np.asarray([beliefs[c].existence_probability for c in unresolved]); expected=fixed_present+float(np.sum(probabilities)); target=float(np.clip(expected,minimum,maximum)-fixed_present); target=float(np.clip(target,1e-9,len(unresolved)-1e-9))
    if abs(float(np.sum(probabilities))-target)<=1e-10: return
    logits=np.log(np.clip(probabilities,1e-12,1-1e-12)/np.clip(1-probabilities,1e-12,1))
    low,high=-40.,40.
    for _ in range(80):
        middle=(low+high)/2; value=float(np.sum(1/(1+np.exp(-(logits+middle)))))
        if value<target: low=middle
        else: high=middle
    adjusted=1/(1+np.exp(-(logits+(low+high)/2)))
    for channel,probability in zip(unresolved,adjusted,strict=True): beliefs[channel].set_existence_probability(float(probability))
