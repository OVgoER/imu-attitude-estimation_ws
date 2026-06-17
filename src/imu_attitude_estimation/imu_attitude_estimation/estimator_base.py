import math
from dataclasses import dataclass, field
from typing import Dict

import numpy as np

from .math_utils import (
    GRAVITY,
    euler_from_quat,
    quat_conjugate,
    quat_from_accel,
    quat_from_rotvec,
    quat_multiply,
    quat_normalize,
    rotvec_from_quat,
)


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
        self.latest_reference = None
        self.reference_max_age = self._config_float("reference_max_age", 0.05)
        self.reference_pose_gain = self._config_float("reference_pose_gain", 0.0)
        self.reference_velocity_gain = self._config_float("reference_velocity_gain", 0.0)
        self.reference_attitude_gain = self._config_float("reference_attitude_gain", 0.0)
        self.flat_motion_constraint = bool(config.get(f"{self.name}_flat_motion_constraint", config.get("flat_motion_constraint", False)))
        self.flat_motion_height = self._config_float("flat_motion_height", 0.0)
        self.flat_height_gain = self._config_float("flat_height_gain", 0.0)
        self.flat_vertical_velocity_gain = self._config_float("flat_vertical_velocity_gain", 0.0)
        self.stationary_translation_constraint = bool(
            config.get(
                f"{self.name}_stationary_translation_constraint",
                config.get("stationary_translation_constraint", False),
            )
        )
        self.stationary_position_gain = self._config_float("stationary_position_gain", 0.0)
        self.stationary_velocity_gain = self._config_float("stationary_velocity_gain", 0.0)
        self.stationary_translation_static_only = bool(
            config.get(
                f"{self.name}_stationary_translation_static_only",
                config.get("stationary_translation_static_only", False),
            )
        )
        self.stationary_translation_anchor = None
        self.initialized = False
        self.last_stamp = None
        self.detector = StaticDetector(
            gyro_threshold=float(config.get("static_gyro_threshold", 0.035)),
            accel_norm_threshold=float(config.get("static_accel_threshold", 0.16)),
            window_size=int(config.get("static_window", 20)),
        )

    def _config_float(self, key: str, default: float) -> float:
        return float(self.config.get(f"{self.name}_{key}", self.config.get(key, default)))

    def _time_gain(self, rate: float, dt: float, cap: float = 1.0) -> float:
        if rate <= 0.0 or dt <= 0.0:
            return 0.0
        return min(cap, 1.0 - math.exp(-rate * dt))

    def update_reference(self, reference: Estimate) -> None:
        self.latest_reference = reference.copy()

    def _apply_flat_motion_constraint(self, dt: float) -> None:
        if not self.flat_motion_constraint:
            return
        height_gain = self._time_gain(self.flat_height_gain, dt, cap=0.50)
        velocity_gain = self._time_gain(self.flat_vertical_velocity_gain, dt, cap=0.80)
        if height_gain > 0.0:
            self.estimate.position[2] += height_gain * (
                self.flat_motion_height - self.estimate.position[2]
            )
        if velocity_gain > 0.0:
            self.estimate.velocity[2] *= 1.0 - velocity_gain
            self.estimate.extra["flat_motion_constraint"] = 1.0

    def _apply_stationary_translation_constraint(self, dt: float, active: bool = True) -> None:
        if not self.stationary_translation_constraint:
            return
        if self.stationary_translation_static_only and not active:
            return
        if self.stationary_translation_anchor is None:
            self.stationary_translation_anchor = self.estimate.position.copy()
        pos_gain = self._time_gain(self.stationary_position_gain, dt, cap=0.65)
        vel_gain = self._time_gain(self.stationary_velocity_gain, dt, cap=0.90)
        if vel_gain > 0.0:
            self.estimate.velocity *= 1.0 - vel_gain
        if pos_gain > 0.0:
            self.estimate.position += pos_gain * (
                self.stationary_translation_anchor - self.estimate.position
            )
        if pos_gain > 0.0 or vel_gain > 0.0:
            self.estimate.extra["stationary_translation_constraint"] = 1.0

    def _apply_reference_pseudo_measurement(self, dt: float) -> None:
        ref = self.latest_reference
        if ref is None:
            return
        stamp_age = self.estimate.stamp - ref.stamp
        if not np.isfinite(stamp_age) or abs(stamp_age) > self.reference_max_age:
            return
        pos_gain = self._time_gain(self.reference_pose_gain, dt, cap=0.45)
        vel_gain = self._time_gain(self.reference_velocity_gain, dt, cap=0.45)
        att_gain = self._time_gain(self.reference_attitude_gain, dt, cap=0.30)
        if pos_gain > 0.0:
            self.estimate.position = (
                (1.0 - pos_gain) * self.estimate.position + pos_gain * ref.position
            )
        if vel_gain > 0.0:
            self.estimate.velocity = (
                (1.0 - vel_gain) * self.estimate.velocity + vel_gain * ref.velocity
            )
        if att_gain > 0.0:
            err = rotvec_from_quat(
                quat_multiply(ref.orientation, quat_conjugate(self.estimate.orientation))
            )
            self.estimate.orientation = quat_multiply(
                quat_from_rotvec(att_gain * err),
                self.estimate.orientation,
            )
        if pos_gain > 0.0 or vel_gain > 0.0 or att_gain > 0.0:
            self.estimate.extra["reference_update_active"] = 1.0

    def initialize(self, sample: ImuSample) -> Estimate:
        self.estimate.stamp = sample.stamp
        self.estimate.orientation = quat_normalize(quat_from_accel(sample.accel))
        if self.stationary_translation_constraint:
            self.stationary_translation_anchor = self.estimate.position.copy()
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
