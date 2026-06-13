from dataclasses import dataclass, field
from typing import Dict

import numpy as np

from .math_utils import GRAVITY, euler_from_quat, quat_from_accel, quat_normalize


@dataclass
class ImuSample:
    stamp: float
    gyro: np.ndarray
    accel: np.ndarray


@dataclass
class Estimate:
    stamp: float = 0.0
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    orientation: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))
    gyro_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))
    accel_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))
    zupt_active: bool = False
    zaru_active: bool = False
    extra: Dict[str, float] = field(default_factory=dict)

    def copy(self) -> "Estimate":
        return Estimate(
            stamp=self.stamp,
            position=self.position.copy(),
            velocity=self.velocity.copy(),
            orientation=self.orientation.copy(),
            gyro_bias=self.gyro_bias.copy(),
            accel_bias=self.accel_bias.copy(),
            zupt_active=self.zupt_active,
            zaru_active=self.zaru_active,
            extra=dict(self.extra),
        )

    @property
    def euler(self):
        return euler_from_quat(self.orientation)


class StaticDetector:
    def __init__(
        self,
        gyro_threshold: float = 0.035,
        accel_norm_threshold: float = 0.16,
        window_size: int = 20,
    ) -> None:
        self.gyro_threshold = gyro_threshold
        self.accel_norm_threshold = accel_norm_threshold
        self.window_size = window_size
        self._gyro_norms = []
        self._accel_norm_errors = []

    def update(self, gyro: np.ndarray, accel: np.ndarray) -> bool:
        self._gyro_norms.append(float(np.linalg.norm(gyro)))
        self._accel_norm_errors.append(abs(float(np.linalg.norm(accel)) - GRAVITY))
        self._gyro_norms = self._gyro_norms[-self.window_size :]
        self._accel_norm_errors = self._accel_norm_errors[-self.window_size :]
        if len(self._gyro_norms) < max(3, self.window_size // 3):
            return False
        return (
            np.mean(self._gyro_norms) < self.gyro_threshold
            and np.mean(self._accel_norm_errors) < self.accel_norm_threshold
        )


class BaseEstimator:
    name = "base"

    def __init__(self, config: Dict) -> None:
        self.config = config
        self.gravity = float(config.get("gravity", GRAVITY))
        self.estimate = Estimate()
        self.initialized = False
        self.last_stamp = None
        self.detector = StaticDetector(
            gyro_threshold=float(config.get("static_gyro_threshold", 0.035)),
            accel_norm_threshold=float(config.get("static_accel_threshold", 0.16)),
            window_size=int(config.get("static_window", 20)),
        )

    def initialize(self, sample: ImuSample) -> Estimate:
        self.estimate.stamp = sample.stamp
        self.estimate.orientation = quat_normalize(quat_from_accel(sample.accel))
        self.last_stamp = sample.stamp
        self.initialized = True
        return self.estimate.copy()

    def update(self, sample: ImuSample) -> Estimate:
        if not self.initialized:
            return self.initialize(sample)
        dt = sample.stamp - float(self.last_stamp)
        self.last_stamp = sample.stamp
        if dt <= 0.0 or dt > 1.0:
            self.estimate.stamp = sample.stamp
            return self.estimate.copy()
        return self._update(sample, dt)

    def _update(self, sample: ImuSample, dt: float) -> Estimate:
        raise NotImplementedError
