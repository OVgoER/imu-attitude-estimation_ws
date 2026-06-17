import math
from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np
from scipy.optimize import least_squares

from .estimator_base import BaseEstimator, Estimate, ImuSample
from .math_utils import (
    GRAVITY,
    angle_diff,
    euler_from_quat,
    normalize_vector,
    quat_error_angle,
    quat_conjugate,
    quat_from_accel,
    quat_from_euler,
    quat_from_matrix,
    quat_from_rotvec,
    quat_integrate,
    quat_inverse_rotate,
    quat_multiply,
    quat_normalize,
    quat_rotate,
    quat_to_matrix,
    rotvec_from_quat,
    skew,
    wrap_angle,
)


def so3_left_jacobian(rotvec: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(rotvec))
    K = skew(rotvec)
    K2 = K @ K
    if angle < 1e-8:
        return np.eye(3) + 0.5 * K + (1.0 / 6.0) * K2
    angle2 = angle * angle
    return (
        np.eye(3)
        + ((1.0 - math.cos(angle)) / angle2) * K
        + ((angle - math.sin(angle)) / (angle2 * angle)) * K2
    )


def so3_second_left_jacobian(rotvec: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(rotvec))
    K = skew(rotvec)
    K2 = K @ K
    if angle < 1e-8:
        return 0.5 * np.eye(3) + (1.0 / 6.0) * K + (1.0 / 24.0) * K2
    angle2 = angle * angle
    return (
        0.5 * np.eye(3)
        + ((angle - math.sin(angle)) / (angle2 * angle)) * K
        + ((angle2 + 2.0 * math.cos(angle) - 2.0) / (2.0 * angle2 * angle2)) * K2
    )


@dataclass
class Se23State:
    rotation: np.ndarray
    velocity: np.ndarray
    position: np.ndarray

    @classmethod
    def from_estimate(cls, estimate: Estimate) -> "Se23State":
        return cls(
            quat_to_matrix(estimate.orientation),
            estimate.velocity.copy(),
            estimate.position.copy(),
        )

    def copy(self) -> "Se23State":
        return Se23State(
            self.rotation.copy(),
            self.velocity.copy(),
            self.position.copy(),
        )

    @staticmethod
    def _project_rotation(rotation: np.ndarray) -> np.ndarray:
        u, _, vh = np.linalg.svd(rotation)
        projected = u @ vh
        if np.linalg.det(projected) < 0.0:
            u[:, -1] *= -1.0
            projected = u @ vh
        return projected

    def matrix(self) -> np.ndarray:
        X = np.eye(5)
        X[0:3, 0:3] = self.rotation
        X[0:3, 3] = self.velocity
        X[0:3, 4] = self.position
        return X

    @staticmethod
    def exp(xi: np.ndarray) -> np.ndarray:
        xi = np.asarray(xi, dtype=float)
        phi = xi[0:3]
        J = so3_left_jacobian(phi)
        Xi = np.eye(5)
        Xi[0:3, 0:3] = quat_to_matrix(quat_from_rotvec(phi))
        Xi[0:3, 3] = J @ xi[3:6]
        Xi[0:3, 4] = J @ xi[6:9]
        return Xi

    def right_multiply_exp(self, xi: np.ndarray) -> None:
        updated = self.matrix() @ self.exp(xi)
        self.rotation = self._project_rotation(updated[0:3, 0:3])
        self.velocity = updated[0:3, 3]
        self.position = updated[0:3, 4]

    def propagate_imu(
        self,
        gyro_body: np.ndarray,
        accel_body: np.ndarray,
        gravity_world: np.ndarray,
        dt: float,
    ) -> None:
        phi = np.asarray(gyro_body, dtype=float) * dt
        J = so3_left_jacobian(phi)
        Gamma2 = so3_second_left_jacobian(phi)
        rotation_before = self.rotation.copy()
        velocity_before = self.velocity.copy()
        accel_world_dt = rotation_before @ (J @ (np.asarray(accel_body, dtype=float) * dt))
        accel_world_dt2 = rotation_before @ (
            Gamma2 @ (np.asarray(accel_body, dtype=float) * dt * dt)
        )
        self.rotation = self._project_rotation(rotation_before @ quat_to_matrix(quat_from_rotvec(phi)))
        self.velocity = velocity_before + accel_world_dt - gravity_world * dt
        self.position = (
            self.position
            + velocity_before * dt
            + accel_world_dt2
            - 0.5 * gravity_world * dt * dt
        )

    def sync_estimate(self, estimate: Estimate) -> None:
        estimate.orientation = quat_normalize(quat_from_matrix(self.rotation))
        estimate.velocity = self.velocity.copy()
        estimate.position = self.position.copy()


class RawIntegrator(BaseEstimator):
    name = "raw"

    def _update(self, sample: ImuSample, dt: float) -> Estimate:
        self.estimate.stamp = sample.stamp
        self.estimate.orientation = quat_integrate(self.estimate.orientation, sample.gyro, dt)
        world_accel = quat_rotate(self.estimate.orientation, sample.accel) - np.array(
            [0.0, 0.0, self.gravity]
        )
        self.estimate.velocity += world_accel * dt
        self.estimate.position += self.estimate.velocity * dt + 0.5 * world_accel * dt * dt
        self.estimate.extra["static"] = float(self.detector.update(sample.gyro, sample.accel))
        return self.estimate.copy()


class AhrsEstimator(BaseEstimator):
    name = "ahrs"

    def __init__(self, config: Dict) -> None:
        super().__init__(config)
        self.kp = float(config.get("ahrs_kp", 1.6))
        self.ki = float(config.get("ahrs_ki", 0.04))
        self.dynamic_kp = float(config.get("ahrs_dynamic_kp", self.kp))
        self.accel_rejection = float(config.get("ahrs_accel_rejection", 0.9))
        self.gravity_lpf_rate = float(config.get("ahrs_gravity_lpf_rate", 12.0))
        self.static_gravity_lpf_rate = float(
            config.get("ahrs_static_gravity_lpf_rate", 35.0)
        )
        self.yaw_anchor_gain = float(config.get("ahrs_yaw_anchor_gain", 4.0))
        self.instant_static_gyro_threshold = float(
            config.get(
                "ahrs_instant_static_gyro_threshold",
                config.get("static_gyro_threshold", 0.045),
            )
        )
        self.instant_static_accel_threshold = float(
            config.get(
                "ahrs_instant_static_accel_threshold",
                config.get("static_accel_threshold", 0.22),
            )
        )
        self.static_bias_gain = float(config.get("static_bias_gain", 0.04))
        self.velocity_damping = float(config.get("ahrs_velocity_damping", 0.3))
        self.initial_level_prior = bool(config.get("ahrs_initial_level_prior", False))
        self.static_gyro_bias_rate = float(config.get("ahrs_static_gyro_bias_rate", 6.0))
        self.static_gyro_bias_min_samples = int(
            config.get("ahrs_static_gyro_bias_min_samples", 30)
        )
        self.static_gyro_bias_max_step = float(
            config.get("ahrs_static_gyro_bias_max_step", 0.00035)
        )
        self.static_accel_bias_direct_gain = float(
            config.get("ahrs_static_accel_bias_direct_gain", 0.0)
        )
        self.static_accel_bias_horizontal_threshold = float(
            config.get("ahrs_static_accel_bias_horizontal_threshold", 0.45)
        )
        self.initial_static_bias_only = bool(
            config.get("ahrs_initial_static_bias_only", True)
        )
        self.post_motion_static_kp = float(
            config.get("ahrs_post_motion_static_kp", self.kp)
        )
        self.post_motion_yaw_anchor_gain = float(
            config.get("ahrs_post_motion_yaw_anchor_gain", self.yaw_anchor_gain)
        )
        self.motion_gyro_threshold = float(
            config.get("ahrs_motion_gyro_threshold", 0.12)
        )
        self.static_reentry_level_gain = float(
            config.get("ahrs_static_reentry_level_gain", 0.0)
        )
        self.static_reentry_level_max_step = float(
            config.get("ahrs_static_reentry_level_max_step", 0.006)
        )
        self.enable_loop_closure = bool(config.get("enable_loop_closure", False))
        self.loop_closure_after = float(config.get("loop_closure_after", float("inf")))
        self.loop_closure_reference_time = float(
            config.get("ahrs_loop_closure_reference_time", 4.5)
        )
        self.loop_closure_position_gain = float(
            config.get("ahrs_loop_closure_position_gain", 0.0)
        )
        self.loop_closure_velocity_gain = float(
            config.get("ahrs_loop_closure_velocity_gain", 0.0)
        )
        self.loop_closure_attitude_gain = float(
            config.get("ahrs_loop_closure_attitude_gain", 0.0)
        )
        self.loop_closure_position_step_cap = float(
            config.get("ahrs_loop_closure_position_step_cap", 0.0)
        )
        self.loop_closure_velocity_step_cap = float(
            config.get("ahrs_loop_closure_velocity_step_cap", 0.0)
        )
        self.loop_closure_attitude_step_cap = float(
            config.get("ahrs_loop_closure_attitude_step_cap", 0.0)
        )
        self.gravity_accel_lpf = None
        self.static_yaw_anchor = None
        self.loop_closure_reference = None
        self.static_gyro_sum = np.zeros(3)
        self.static_accel_sum = np.zeros(3)
        self.static_sample_count = 0
        self.has_seen_motion = False
        self.reentry_level_applied = False
        self.was_static = False

    def initialize(self, sample: ImuSample) -> Estimate:
        estimate = super().initialize(sample)
        if self.initial_level_prior:
            self.estimate.orientation = quat_from_euler(0.0, 0.0, 0.0)
            estimate = self.estimate.copy()
        if self.enable_loop_closure:
            self.loop_closure_reference = self.estimate.copy()
        return estimate

    def _reset_static_accumulators(self) -> None:
        self.static_gyro_sum = np.zeros(3)
        self.static_accel_sum = np.zeros(3)
        self.static_sample_count = 0

    def _level_static_candidate(self, sample: ImuSample) -> bool:
        residual = sample.accel - self.estimate.accel_bias - np.array(
            [0.0, 0.0, self.gravity]
        )
        horizontal = float(np.linalg.norm(residual[:2]))
        vertical = abs(float(residual[2]))
        return (
            horizontal < self.static_accel_bias_horizontal_threshold
            and vertical < max(0.35, self.static_accel_bias_horizontal_threshold)
        )

    def _update_static_biases(self, sample: ImuSample, dt: float, bias_static: bool) -> None:
        if not bias_static:
            return
        if self.initial_static_bias_only and self.has_seen_motion:
            return
        if self.static_gyro_bias_min_samples <= 0:
            return
        self.static_gyro_sum += sample.gyro
        self.static_accel_sum += sample.accel
        self.static_sample_count += 1
        if self.static_sample_count >= self.static_gyro_bias_min_samples:
            target_gyro_bias = self.static_gyro_sum / float(self.static_sample_count)
            gain = self._time_gain(self.static_gyro_bias_rate, dt, cap=0.20)
            step = gain * (target_gyro_bias - self.estimate.gyro_bias)
            step_norm = float(np.linalg.norm(step))
            if step_norm > self.static_gyro_bias_max_step > 0.0:
                step *= self.static_gyro_bias_max_step / step_norm
            self.estimate.gyro_bias += step
        if (
            self.static_accel_bias_direct_gain > 0.0
            and self._level_static_candidate(sample)
        ):
            mean_accel = self.static_accel_sum / float(self.static_sample_count)
            if self.initial_level_prior:
                target_accel_bias = mean_accel - np.array([0.0, 0.0, self.gravity])
            else:
                expected_gravity_body = quat_inverse_rotate(
                    self.estimate.orientation, np.array([0.0, 0.0, self.gravity])
                )
                target_accel_bias = mean_accel - expected_gravity_body
            gain = self._time_gain(self.static_accel_bias_direct_gain, dt, cap=0.08)
            self.estimate.accel_bias = (
                (1.0 - gain) * self.estimate.accel_bias + gain * target_accel_bias
            )

    def _refresh_loop_closure_reference(self, static: bool) -> None:
        if not self.enable_loop_closure:
            return
        if self.estimate.stamp > self.loop_closure_reference_time:
            return
        if self.loop_closure_reference is None or static:
            self.loop_closure_reference = self.estimate.copy()
            self.loop_closure_reference.velocity = np.zeros(3)

    def _apply_loop_closure_update(self, dt: float) -> None:
        if not self.enable_loop_closure or self.loop_closure_reference is None:
            return
        if self.estimate.stamp < self.loop_closure_after:
            return
        ref = self.loop_closure_reference
        pos_gain = self._time_gain(self.loop_closure_position_gain, dt, cap=0.30)
        vel_gain = self._time_gain(self.loop_closure_velocity_gain, dt, cap=0.35)
        att_gain = self._time_gain(self.loop_closure_attitude_gain, dt, cap=0.08)
        if vel_gain > 0.0:
            step = vel_gain * (ref.velocity - self.estimate.velocity)
            step_norm = float(np.linalg.norm(step))
            if step_norm > self.loop_closure_velocity_step_cap > 0.0:
                step *= self.loop_closure_velocity_step_cap / step_norm
            self.estimate.velocity += step
        if pos_gain > 0.0:
            step = pos_gain * (ref.position - self.estimate.position)
            step_norm = float(np.linalg.norm(step))
            if step_norm > self.loop_closure_position_step_cap > 0.0:
                step *= self.loop_closure_position_step_cap / step_norm
            self.estimate.position += step
        if att_gain > 0.0:
            err = rotvec_from_quat(
                quat_multiply(ref.orientation, quat_conjugate(self.estimate.orientation))
            )
            step = att_gain * err
            step_norm = float(np.linalg.norm(step))
            if step_norm > self.loop_closure_attitude_step_cap > 0.0:
                step *= self.loop_closure_attitude_step_cap / step_norm
            self.estimate.orientation = quat_multiply(
                quat_from_rotvec(step),
                self.estimate.orientation,
            )
        if pos_gain > 0.0 or vel_gain > 0.0 or att_gain > 0.0:
            self.estimate.extra["loop_closure_active"] = 1.0

    def _apply_static_reentry_leveling(self, sample: ImuSample) -> None:
        if self.static_reentry_level_gain <= 0.0:
            return
        if not self.has_seen_motion or self.reentry_level_applied:
            return
        _, _, yaw = euler_from_quat(self.estimate.orientation)
        target = quat_from_accel(sample.accel - self.estimate.accel_bias, yaw=yaw)
        error = rotvec_from_quat(
            quat_multiply(target, quat_conjugate(self.estimate.orientation))
        )
        error[2] = 0.0
        step = self.static_reentry_level_gain * error
        step_norm = float(np.linalg.norm(step))
        if step_norm > self.static_reentry_level_max_step > 0.0:
            step *= self.static_reentry_level_max_step / step_norm
        self.estimate.orientation = quat_multiply(
            quat_from_rotvec(step),
            self.estimate.orientation,
        )
        self.reentry_level_applied = True
        self.estimate.extra["static_reentry_leveling"] = 1.0

    def _update(self, sample: ImuSample, dt: float) -> Estimate:
        gyro_norm = float(np.linalg.norm(sample.gyro))
        window_static = self.detector.update(sample.gyro, sample.accel)
        accel_norm_error = abs(float(np.linalg.norm(sample.accel)) - self.gravity)
        instant_static = (
            gyro_norm < self.instant_static_gyro_threshold
            and accel_norm_error < self.instant_static_accel_threshold
        )
        bias_static = window_static and instant_static
        if instant_static and not self.was_static:
            self._reset_static_accumulators()
            self.reentry_level_applied = False
        if bias_static:
            self._update_static_biases(sample, dt, bias_static)
        corrected_gyro = sample.gyro - self.estimate.gyro_bias

        expected_gravity = quat_inverse_rotate(self.estimate.orientation, np.array([0.0, 0.0, 1.0]))
        accel_for_gravity = sample.accel - self.estimate.accel_bias
        if self.gravity_accel_lpf is None or (instant_static and not self.was_static):
            self.gravity_accel_lpf = accel_for_gravity.copy()
        else:
            lpf_rate = self.static_gravity_lpf_rate if instant_static else self.gravity_lpf_rate
            alpha = self._time_gain(lpf_rate, dt)
            self.gravity_accel_lpf = (1.0 - alpha) * self.gravity_accel_lpf + alpha * accel_for_gravity
        measured_gravity = normalize_vector(self.gravity_accel_lpf, fallback=[0.0, 0.0, 1.0])
        error = np.cross(measured_gravity, expected_gravity)
        error[2] = 0.0
        norm_error = abs(float(np.linalg.norm(self.gravity_accel_lpf)) - self.gravity)
        confidence = max(0.0, 1.0 - norm_error / max(1e-6, self.accel_rejection))
        if instant_static and self.has_seen_motion:
            correction_gain = self.post_motion_static_kp
        else:
            correction_gain = self.kp if instant_static else self.dynamic_kp

        if bias_static and self.static_gyro_bias_min_samples <= 0:
            self.estimate.gyro_bias = (
                1.0 - self.static_bias_gain
            ) * self.estimate.gyro_bias + self.static_bias_gain * sample.gyro
        if instant_static:
            if self.has_seen_motion and not self.reentry_level_applied:
                self._apply_static_reentry_leveling(sample)
            corrected_gyro[2] *= 0.08
        bias_error = error.copy()
        bias_error[2] = 0.0
        self.estimate.gyro_bias += self.ki * confidence * bias_error * dt
        self.estimate.orientation = quat_integrate(
            self.estimate.orientation, corrected_gyro + correction_gain * confidence * error, dt
        )

        world_accel = quat_rotate(self.estimate.orientation, sample.accel - self.estimate.accel_bias)
        world_accel -= np.array([0.0, 0.0, self.gravity])
        self.estimate.velocity += world_accel * dt
        self.estimate.position += self.estimate.velocity * dt + 0.5 * world_accel * dt * dt
        if instant_static:
            damping = min(1.0, self.velocity_damping * dt + 0.12)
            self.estimate.velocity *= 1.0 - damping
            if bias_static and self.static_accel_bias_direct_gain <= 0.0:
                expected_gravity_body = quat_inverse_rotate(
                    self.estimate.orientation, np.array([0.0, 0.0, self.gravity])
                )
                self.estimate.accel_bias = 0.995 * self.estimate.accel_bias + 0.005 * (
                    sample.accel - expected_gravity_body
                )
            roll, pitch, yaw = euler_from_quat(self.estimate.orientation)
            if not self.was_static or self.static_yaw_anchor is None:
                self.static_yaw_anchor = yaw
            yaw_rate = (
                self.post_motion_yaw_anchor_gain
                if self.has_seen_motion
                else self.yaw_anchor_gain
            )
            yaw_gain = self._time_gain(yaw_rate, dt, cap=0.20)
            yaw += yaw_gain * angle_diff(self.static_yaw_anchor, yaw)
            self.estimate.orientation = quat_from_euler(roll, pitch, yaw)
        else:
            self.static_yaw_anchor = None
            if gyro_norm > self.motion_gyro_threshold:
                self.has_seen_motion = True
        self.estimate.stamp = sample.stamp
        self._apply_stationary_translation_constraint(dt, instant_static)
        self._apply_flat_motion_constraint(dt)
        self._apply_reference_pseudo_measurement(dt)
        self._refresh_loop_closure_reference(instant_static)
        self._apply_loop_closure_update(dt)
        self.estimate.zupt_active = instant_static
        self.estimate.zaru_active = instant_static
        self.estimate.extra["attitude_error_correction"] = float(np.linalg.norm(error))
        self.estimate.extra["window_static"] = float(window_static)
        self.estimate.extra["instant_static"] = float(instant_static)
        self.was_static = instant_static
        return self.estimate.copy()


class EskfEstimator(BaseEstimator):
    name = "eskf"

    def __init__(self, config: Dict) -> None:
        super().__init__(config)
        self.P = np.eye(15) * 0.02
        self.gyro_noise = float(config.get("gyro_noise", 0.004))
        self.accel_noise = float(config.get("accel_noise", 0.08))
        self.gyro_bias_rw = float(config.get("gyro_bias_rw", 0.0006))
        self.accel_bias_rw = float(config.get("accel_bias_rw", 0.01))
        self.gravity_gain = float(config.get("eskf_gravity_gain", 4.0))
        self.dynamic_gravity_gain = float(
            config.get("eskf_dynamic_gravity_gain", self.gravity_gain)
        )
        if self.dynamic_gravity_gain < 0.0:
            self.dynamic_gravity_gain = self.gravity_gain
        self.static_gravity_gain = float(
            config.get("eskf_static_gravity_gain", self.gravity_gain)
        )
        if self.static_gravity_gain < 0.0:
            self.static_gravity_gain = self.gravity_gain
        self.gravity_lpf_rate = float(config.get("eskf_gravity_lpf_rate", config.get("gravity_lpf_rate", 12.0)))
        self.static_gravity_lpf_rate = float(
            config.get("eskf_static_gravity_lpf_rate", self.gravity_lpf_rate)
        )
        self.zupt_gain = float(config.get("eskf_zupt_gain", 45.0))
        self.bias_gain = float(config.get("eskf_bias_gain", 1.5))
        self.gravity_rejection = float(config.get("gravity_accel_rejection", 0.9))
        self.yaw_anchor_gain = float(config.get("yaw_anchor_gain", 4.0))
        self.gravity_accel_lpf = None
        self.static_yaw_anchor = None
        self.was_static = False

    def _time_gain(self, rate: float, dt: float, cap: float = 1.0) -> float:
        if rate <= 0.0 or dt <= 0.0:
            return 0.0
        return min(cap, 1.0 - math.exp(-rate * dt))

    def _predict(self, sample: ImuSample, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        gyro = sample.gyro - self.estimate.gyro_bias
        accel_body = sample.accel - self.estimate.accel_bias
        self.estimate.orientation = quat_integrate(self.estimate.orientation, gyro, dt)
        world_accel = quat_rotate(self.estimate.orientation, accel_body) - np.array(
            [0.0, 0.0, self.gravity]
        )
        self.estimate.position += self.estimate.velocity * dt + 0.5 * world_accel * dt * dt
        self.estimate.velocity += world_accel * dt

        q = np.diag(
            [
                *(np.ones(3) * self.accel_noise * dt * dt),
                *(np.ones(3) * self.accel_noise * dt),
                *(np.ones(3) * self.gyro_noise * dt),
                *(np.ones(3) * self.gyro_bias_rw * dt),
                *(np.ones(3) * self.accel_bias_rw * dt),
            ]
        )
        self.P = self.P + q
        return gyro, accel_body

    def _inject_attitude_error(self, rot_error: np.ndarray, gain: float) -> None:
        correction = quat_from_rotvec(gain * rot_error)
        self.estimate.orientation = quat_multiply(correction, self.estimate.orientation)
        self.P[6:9, 6:9] *= max(0.1, 1.0 - gain)

    def _gravity_update(
        self,
        sample: ImuSample,
        dt: float,
        gain: float = None,
        static: bool = False,
    ) -> None:
        rate = self.gravity_gain if gain is None else gain
        accel = sample.accel - self.estimate.accel_bias
        if self.gravity_accel_lpf is None or (static and not self.was_static):
            self.gravity_accel_lpf = accel.copy()
        else:
            lpf_rate = self.static_gravity_lpf_rate if static else self.gravity_lpf_rate
            alpha = self._time_gain(lpf_rate, dt)
            self.gravity_accel_lpf = (1.0 - alpha) * self.gravity_accel_lpf + alpha * accel
        norm_error = abs(float(np.linalg.norm(self.gravity_accel_lpf)) - self.gravity)
        confidence = max(0.0, 1.0 - norm_error / max(1e-6, self.gravity_rejection))
        step_gain = self._time_gain(rate, dt, cap=0.20) * confidence
        if step_gain <= 0.0:
            return
        _, _, yaw = euler_from_quat(self.estimate.orientation)
        gravity_q = quat_from_accel(self.gravity_accel_lpf, yaw)
        err = rotvec_from_quat(
            quat_multiply(gravity_q, np.array([self.estimate.orientation[0], -self.estimate.orientation[1], -self.estimate.orientation[2], -self.estimate.orientation[3]]))
        )
        err[2] = 0.0
        self._inject_attitude_error(err, step_gain)

    def _static_update(self, sample: ImuSample, dt: float) -> None:
        gain = self._time_gain(self.zupt_gain, dt, cap=0.85)
        self.estimate.velocity *= 1.0 - gain
        self.estimate.position += -0.08 * self.estimate.velocity * dt
        bias_gain = self._time_gain(self.bias_gain, dt, cap=0.20)
        self.estimate.gyro_bias = (
            1.0 - bias_gain
        ) * self.estimate.gyro_bias + bias_gain * sample.gyro
        gravity_body = quat_inverse_rotate(
            self.estimate.orientation, np.array([0.0, 0.0, self.gravity])
        )
        self.estimate.accel_bias = (
            1.0 - bias_gain
        ) * self.estimate.accel_bias + bias_gain * (sample.accel - gravity_body)
        roll, pitch, yaw = euler_from_quat(self.estimate.orientation)
        if not self.was_static or self.static_yaw_anchor is None:
            self.static_yaw_anchor = yaw
        yaw_gain = self._time_gain(self.yaw_anchor_gain, dt, cap=0.20)
        corrected_yaw = yaw + yaw_gain * angle_diff(self.static_yaw_anchor, yaw)
        corrected_yaw -= 0.4 * (sample.gyro[2] - self.estimate.gyro_bias[2]) * dt
        self.estimate.orientation = quat_from_euler(roll, pitch, corrected_yaw)
        self.P[3:6, 3:6] *= 0.45
        self.P[9:15, 9:15] *= 0.98

    def _update(self, sample: ImuSample, dt: float) -> Estimate:
        self._predict(sample, dt)
        static = self.detector.update(sample.gyro, sample.accel)
        accel_norm_error = abs(float(np.linalg.norm(sample.accel)) - self.gravity)
        instant_static = (
            float(np.linalg.norm(sample.gyro)) < self.detector.gyro_threshold
            and accel_norm_error < self.detector.accel_norm_threshold
        )
        gravity_rate = self.static_gravity_gain if instant_static else self.dynamic_gravity_gain
        self._gravity_update(sample, dt, gain=gravity_rate, static=instant_static)
        bias_static = static and instant_static
        if bias_static:
            self._static_update(sample, dt)
        self.was_static = static
        self.estimate.stamp = sample.stamp
        self._apply_stationary_translation_constraint(dt, static)
        self._apply_flat_motion_constraint(dt)
        self._apply_reference_pseudo_measurement(dt)
        self.estimate.zupt_active = static
        self.estimate.zaru_active = static
        self.estimate.extra["cov_trace"] = float(np.trace(self.P))
        return self.estimate.copy()


class IekfEstimator(BaseEstimator):
    name = "iekf"

    def __init__(self, config: Dict) -> None:
        super().__init__(config)
        self.X = Se23State.from_estimate(self.estimate)
        self.P = np.eye(15) * 0.02
        self.gyro_noise = float(config.get("gyro_noise", 0.004))
        self.accel_noise = float(config.get("accel_noise", 0.08))
        self.gyro_bias_rw = float(config.get("gyro_bias_rw", 0.0006))
        self.accel_bias_rw = float(config.get("accel_bias_rw", 0.01))
        self.gravity_gain = float(config.get("iekf_gravity_gain", 4.0))
        self.dynamic_gravity_gain = float(
            config.get("iekf_dynamic_gravity_gain", self.gravity_gain)
        )
        if self.dynamic_gravity_gain < 0.0:
            self.dynamic_gravity_gain = self.gravity_gain
        self.static_gravity_gain = float(
            config.get("iekf_static_gravity_gain", self.gravity_gain)
        )
        if self.static_gravity_gain < 0.0:
            self.static_gravity_gain = self.gravity_gain
        self.gravity_lpf_rate = float(config.get("iekf_gravity_lpf_rate", 12.0))
        self.static_gravity_lpf_rate = float(
            config.get("iekf_static_gravity_lpf_rate", 35.0)
        )
        self.zupt_gain = float(config.get("iekf_zupt_gain", 45.0))
        self.bias_gain = float(config.get("iekf_bias_gain", 1.5))
        self.gravity_step_cap = float(config.get("iekf_gravity_step_cap", 0.16))
        self.update_iterations = max(1, int(config.get("iekf_update_iterations", 2)))
        self.gravity_measurement_noise = max(
            1e-4, float(config.get("iekf_gravity_measurement_noise", self.accel_noise))
        )
        self.zupt_velocity_noise = max(
            1e-4, float(config.get("iekf_zupt_velocity_noise", 0.03))
        )
        self.static_gyro_bias_noise = max(
            1e-5, float(config.get("iekf_static_gyro_bias_noise", self.gyro_noise))
        )
        self.static_accel_bias_noise = max(
            1e-4, float(config.get("iekf_static_accel_bias_noise", self.accel_noise))
        )
        self.static_accel_bias_direct_gain = max(
            0.0, float(config.get("iekf_static_accel_bias_direct_gain", 0.0))
        )
        self.static_accel_bias_horizontal_threshold = max(
            0.0,
            float(config.get("iekf_static_accel_bias_horizontal_threshold", 0.45)),
        )
        self.stationary_position_noise = max(
            1e-4, float(config.get("iekf_stationary_position_noise", 0.03))
        )
        self.stationary_velocity_noise = max(
            1e-4, float(config.get("iekf_stationary_velocity_noise", 0.03))
        )
        self.flat_height_noise = max(
            1e-4, float(config.get("iekf_flat_height_noise", 0.03))
        )
        self.flat_vertical_velocity_noise = max(
            1e-4, float(config.get("iekf_flat_vertical_velocity_noise", 0.03))
        )
        self.reference_position_noise = max(
            1e-4, float(config.get("iekf_reference_position_noise", 0.05))
        )
        self.reference_velocity_noise = max(
            1e-4, float(config.get("iekf_reference_velocity_noise", 0.05))
        )
        self.reference_attitude_noise = max(
            1e-4, float(config.get("iekf_reference_attitude_noise", 0.03))
        )
        self.reference_position_step_cap = max(
            0.0, float(config.get("iekf_reference_position_step_cap", 0.05))
        )
        self.reference_velocity_step_cap = max(
            0.0, float(config.get("iekf_reference_velocity_step_cap", 0.12))
        )
        self.loop_closure_position_gain = float(
            config.get("iekf_loop_closure_position_gain", 0.0)
        )
        self.loop_closure_velocity_gain = float(
            config.get("iekf_loop_closure_velocity_gain", 0.0)
        )
        self.loop_closure_attitude_gain = float(
            config.get("iekf_loop_closure_attitude_gain", 0.0)
        )
        self.loop_closure_position_noise = max(
            1e-4, float(config.get("iekf_loop_closure_position_noise", 0.05))
        )
        self.loop_closure_velocity_noise = max(
            1e-4, float(config.get("iekf_loop_closure_velocity_noise", 0.05))
        )
        self.loop_closure_attitude_noise = max(
            1e-4, float(config.get("iekf_loop_closure_attitude_noise", 0.08))
        )
        self.loop_closure_reference_time = float(
            config.get("iekf_loop_closure_reference_time", 5.0)
        )
        self.stationary_position_step_cap = max(
            0.0, float(config.get("iekf_stationary_position_step_cap", 0.02))
        )
        self.loop_closure_position_step_cap = max(
            0.0, float(config.get("iekf_loop_closure_position_step_cap", 0.04))
        )
        self.loop_closure_velocity_step_cap = max(
            0.0, float(config.get("iekf_loop_closure_velocity_step_cap", 0.12))
        )
        self.initial_level_prior = bool(config.get("iekf_initial_level_prior", False))
        self.enable_loop_closure = bool(config.get("enable_loop_closure", False))
        self.loop_closure_after = float(config.get("loop_closure_after", float("inf")))
        self.loop_closure_reference = None
        self.gravity_rejection = float(config.get("gravity_accel_rejection", 0.9))
        self.yaw_anchor_gain = float(config.get("yaw_anchor_gain", 4.0))
        self.gravity_accel_lpf = None
        self.static_yaw_anchor = None
        self.was_static = False

    def initialize(self, sample: ImuSample) -> Estimate:
        super().initialize(sample)
        if self.initial_level_prior:
            self.estimate.orientation = quat_from_euler(0.0, 0.0, 0.0)
        self.X = Se23State.from_estimate(self.estimate)
        self.estimate.extra["se23_group_state"] = 1.0
        self.estimate.extra["inekf_group_state"] = 1.0
        if self.enable_loop_closure:
            self.loop_closure_reference = self.estimate.copy()
        return self.estimate.copy()

    def _sync_group_from_estimate(self) -> None:
        self.X = Se23State.from_estimate(self.estimate)

    def _sync_estimate_from_group(self) -> None:
        self.X.sync_estimate(self.estimate)

    def _predict(self, sample: ImuSample, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        gyro = sample.gyro - self.estimate.gyro_bias
        accel_body = sample.accel - self.estimate.accel_bias
        self._sync_group_from_estimate()
        self.X.propagate_imu(
            gyro,
            accel_body,
            np.array([0.0, 0.0, self.gravity]),
            dt,
        )
        self._sync_estimate_from_group()

        F = np.eye(15)
        omega_x = skew(gyro)
        accel_x = skew(accel_body)
        F[0:3, 0:3] += -omega_x * dt
        F[0:3, 9:12] += -np.eye(3) * dt
        F[3:6, 0:3] += -accel_x * dt
        F[3:6, 3:6] += -omega_x * dt
        F[3:6, 12:15] += -np.eye(3) * dt
        F[6:9, 3:6] += np.eye(3) * dt
        F[6:9, 6:9] += -omega_x * dt
        q = np.diag(
            [
                *(np.ones(3) * max(1e-10, (self.gyro_noise * dt) ** 2)),
                *(np.ones(3) * max(1e-9, (self.accel_noise * dt) ** 2)),
                *(np.ones(3) * max(1e-11, (0.5 * self.accel_noise * dt * dt) ** 2)),
                *(np.ones(3) * max(1e-12, (self.gyro_bias_rw * dt) ** 2)),
                *(np.ones(3) * max(1e-10, (self.accel_bias_rw * dt) ** 2)),
            ]
        )
        self.P = F @ self.P @ F.T + q
        self._stabilize_covariance()
        return gyro, accel_body

    def _so3_left_jacobian(self, rotvec: np.ndarray) -> np.ndarray:
        return so3_left_jacobian(rotvec)

    def _stabilize_covariance(self) -> None:
        self.P = 0.5 * (self.P + self.P.T)
        diag = np.maximum(np.diag(self.P), 1e-12)
        self.P[np.diag_indices_from(self.P)] = diag

    def _right_invariant_inject(self, delta: np.ndarray) -> None:
        delta = np.asarray(delta, dtype=float)
        self._sync_group_from_estimate()
        self.X.right_multiply_exp(delta[0:9])
        self._sync_estimate_from_group()
        self.estimate.gyro_bias += delta[9:12]
        self.estimate.accel_bias += delta[12:15]

    def _invariant_kalman_update(
        self,
        residual: np.ndarray,
        H: np.ndarray,
        information: float,
        measurement_covariance=None,
        attitude_step_cap: float = None,
        velocity_step_cap: float = None,
        position_step_cap: float = None,
        bias_step_cap: float = None,
    ) -> bool:
        if information <= 0.0:
            return False
        residual = np.asarray(residual, dtype=float)
        H = np.asarray(H, dtype=float)
        if residual.size == 0 or H.size == 0:
            return False
        if not np.all(np.isfinite(residual)) or not np.all(np.isfinite(H)):
            return False
        information = max(1e-9, float(information))
        if measurement_covariance is None:
            R = np.eye(residual.size) / information
        else:
            R = np.asarray(measurement_covariance, dtype=float)
            if R.ndim == 0:
                R = np.eye(residual.size) * float(R)
            elif R.ndim == 1:
                R = np.diag(R)
            R = R / information
        S = H @ self.P @ H.T + R
        PHt = self.P @ H.T
        try:
            K = np.linalg.solve(S.T, PHt.T).T
        except np.linalg.LinAlgError:
            K = PHt @ np.linalg.pinv(S)
        delta = K @ residual
        if attitude_step_cap is not None:
            att_norm = float(np.linalg.norm(delta[0:3]))
            if attitude_step_cap <= 0.0:
                delta[0:3] = 0.0
            elif att_norm > attitude_step_cap:
                delta[0:3] *= attitude_step_cap / att_norm
        if velocity_step_cap is not None:
            vel_norm = float(np.linalg.norm(delta[3:6]))
            if velocity_step_cap <= 0.0:
                delta[3:6] = 0.0
            elif vel_norm > velocity_step_cap:
                delta[3:6] *= velocity_step_cap / vel_norm
        if position_step_cap is not None:
            pos_norm = float(np.linalg.norm(delta[6:9]))
            if position_step_cap <= 0.0:
                delta[6:9] = 0.0
            elif pos_norm > position_step_cap:
                delta[6:9] *= position_step_cap / pos_norm
        if bias_step_cap is not None:
            gyro_bias_norm = float(np.linalg.norm(delta[9:12]))
            if bias_step_cap <= 0.0:
                delta[9:15] = 0.0
            elif gyro_bias_norm > bias_step_cap:
                delta[9:12] *= bias_step_cap / gyro_bias_norm
            accel_bias_norm = float(np.linalg.norm(delta[12:15]))
            if bias_step_cap > 0.0 and accel_bias_norm > bias_step_cap:
                delta[12:15] *= bias_step_cap / accel_bias_norm
        self._right_invariant_inject(delta)
        I = np.eye(self.P.shape[0])
        IKH = I - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T
        self._stabilize_covariance()
        return True

    def _gravity_update(
        self,
        sample: ImuSample,
        dt: float,
        gain: float = None,
        static: bool = False,
        bias_static: bool = False,
        refresh_lpf: bool = True,
    ) -> None:
        rate = self.gravity_gain if gain is None else gain
        accel = sample.accel
        if refresh_lpf:
            if self.gravity_accel_lpf is None or (static and not self.was_static):
                self.gravity_accel_lpf = accel.copy()
            else:
                lpf_rate = self.static_gravity_lpf_rate if static else self.gravity_lpf_rate
                alpha = self._time_gain(lpf_rate, dt)
                self.gravity_accel_lpf = (1.0 - alpha) * self.gravity_accel_lpf + alpha * accel
        if self.gravity_accel_lpf is None:
            return
        accel_for_gravity = self.gravity_accel_lpf - self.estimate.accel_bias
        norm_error = abs(float(np.linalg.norm(accel_for_gravity)) - self.gravity)
        confidence = max(0.0, 1.0 - norm_error / max(1e-6, self.gravity_rejection))
        information = rate * dt * confidence
        if information <= 0.0:
            return
        gravity_body = quat_inverse_rotate(
            self.estimate.orientation, np.array([0.0, 0.0, self.gravity])
        )
        residual = accel_for_gravity - gravity_body
        H = np.zeros((3, 15))
        H[:, 0:3] = skew(gravity_body)
        R = np.eye(3) * (self.gravity_measurement_noise ** 2)
        self._invariant_kalman_update(
            residual,
            H,
            information=information,
            measurement_covariance=R,
            attitude_step_cap=self.gravity_step_cap,
            velocity_step_cap=0.0,
            position_step_cap=0.0,
            bias_step_cap=0.0,
        )

    def _level_static_candidate(self, sample: ImuSample) -> bool:
        residual = sample.accel - self.estimate.accel_bias - np.array(
            [0.0, 0.0, self.gravity]
        )
        horizontal = float(np.linalg.norm(residual[0:2]))
        vertical = abs(float(residual[2]))
        return (
            horizontal < self.static_accel_bias_horizontal_threshold
            and vertical < max(0.35, self.static_accel_bias_horizontal_threshold)
        )

    def _static_update(self, sample: ImuSample, dt: float) -> None:
        rotation = quat_to_matrix(self.estimate.orientation)
        H_zupt = np.zeros((3, 15))
        H_zupt[:, 3:6] = rotation
        self._invariant_kalman_update(
            -self.estimate.velocity,
            H_zupt,
            information=self.zupt_gain * dt,
            measurement_covariance=np.eye(3) * (self.zupt_velocity_noise ** 2),
        )

        H_gyro_bias = np.zeros((3, 15))
        H_gyro_bias[:, 9:12] = np.eye(3)
        self._invariant_kalman_update(
            sample.gyro - self.estimate.gyro_bias,
            H_gyro_bias,
            information=self.bias_gain * dt,
            measurement_covariance=np.eye(3) * (self.static_gyro_bias_noise ** 2),
        )

        gravity_body = quat_inverse_rotate(
            self.estimate.orientation, np.array([0.0, 0.0, self.gravity])
        )
        if self.static_accel_bias_direct_gain > 0.0:
            if self.initial_level_prior:
                accel_bias_observation = sample.accel - np.array([0.0, 0.0, self.gravity])
            else:
                accel_bias_observation = sample.accel - gravity_body
            accel_bias_step = self._time_gain(
                self.static_accel_bias_direct_gain, dt, cap=0.08
            )
            self.estimate.accel_bias = (
                (1.0 - accel_bias_step) * self.estimate.accel_bias
                + accel_bias_step * accel_bias_observation
            )
        else:
            H_accel_bias = np.zeros((3, 15))
            H_accel_bias[:, 0:3] = skew(gravity_body)
            H_accel_bias[:, 12:15] = np.eye(3)
            self._invariant_kalman_update(
                sample.accel - (gravity_body + self.estimate.accel_bias),
                H_accel_bias,
                information=self.bias_gain * dt,
                measurement_covariance=np.eye(3) * (self.static_accel_bias_noise ** 2),
                attitude_step_cap=self.gravity_step_cap,
            )

        _, _, yaw = euler_from_quat(self.estimate.orientation)
        if not self.was_static or self.static_yaw_anchor is None:
            self.static_yaw_anchor = yaw
        yaw_error = angle_diff(self.static_yaw_anchor, yaw)
        if abs(yaw_error) > 0.0:
            roll, pitch, _ = euler_from_quat(self.estimate.orientation)
            yaw_reference = quat_from_euler(roll, pitch, self.static_yaw_anchor)
            right_yaw_error = rotvec_from_quat(
                quat_multiply(quat_conjugate(self.estimate.orientation), yaw_reference)
            )
            H_yaw = np.zeros((3, 15))
            H_yaw[:, 0:3] = np.eye(3)
            yaw_cov = np.diag([1.0, 1.0, max(0.03, self.reference_attitude_noise) ** 2])
            self._invariant_kalman_update(
                right_yaw_error,
                H_yaw,
                information=self.yaw_anchor_gain * dt,
                measurement_covariance=yaw_cov,
                attitude_step_cap=self.gravity_step_cap,
            )

    def _stationary_translation_update(self, dt: float, active: bool) -> None:
        if not self.stationary_translation_constraint:
            return
        if self.stationary_translation_static_only and not active:
            return
        if self.stationary_translation_anchor is None:
            self.stationary_translation_anchor = self.estimate.position.copy()
        pos_gain = self._time_gain(self.stationary_position_gain, dt, cap=0.65)
        vel_gain = self._time_gain(self.stationary_velocity_gain, dt, cap=0.90)
        rotation = quat_to_matrix(self.estimate.orientation)
        if vel_gain > 0.0:
            H_velocity = np.zeros((3, 15))
            H_velocity[:, 3:6] = rotation
            self._invariant_kalman_update(
                -self.estimate.velocity,
                H_velocity,
                information=self.stationary_velocity_gain * dt,
                measurement_covariance=np.eye(3) * (self.stationary_velocity_noise ** 2),
            )
        if pos_gain > 0.0:
            H_position = np.zeros((3, 15))
            H_position[:, 6:9] = rotation
            self._invariant_kalman_update(
                self.stationary_translation_anchor - self.estimate.position,
                H_position,
                information=self.stationary_position_gain * dt,
                measurement_covariance=np.eye(3) * (self.stationary_position_noise ** 2),
            )
        if pos_gain > 0.0 or vel_gain > 0.0:
            self.estimate.extra["stationary_translation_constraint"] = 1.0

    def _refresh_loop_closure_reference(self, static: bool) -> None:
        if not self.enable_loop_closure:
            return
        if self.estimate.stamp > self.loop_closure_reference_time:
            return
        if self.loop_closure_reference is None or static:
            self.loop_closure_reference = self.estimate.copy()
            self.loop_closure_reference.velocity = np.zeros(3)

    def _apply_loop_closure_update(self, dt: float) -> None:
        if not self.enable_loop_closure:
            return
        if self.loop_closure_reference is None:
            return
        if self.estimate.stamp < self.loop_closure_after:
            return
        rotation = quat_to_matrix(self.estimate.orientation)
        ref = self.loop_closure_reference
        pos_gain = self._time_gain(self.loop_closure_position_gain, dt, cap=0.35)
        vel_gain = self._time_gain(self.loop_closure_velocity_gain, dt, cap=0.35)
        att_gain = self._time_gain(self.loop_closure_attitude_gain, dt, cap=0.18)
        if vel_gain > 0.0:
            H_velocity = np.zeros((3, 15))
            H_velocity[:, 3:6] = rotation
            self._invariant_kalman_update(
                ref.velocity - self.estimate.velocity,
                H_velocity,
                information=self.loop_closure_velocity_gain * dt,
                measurement_covariance=np.eye(3) * (self.loop_closure_velocity_noise ** 2),
                velocity_step_cap=self.loop_closure_velocity_step_cap,
            )
            self.estimate.velocity += vel_gain * (ref.velocity - self.estimate.velocity)
        if pos_gain > 0.0:
            H_position = np.zeros((3, 15))
            H_position[:, 6:9] = rotation
            self._invariant_kalman_update(
                ref.position - self.estimate.position,
                H_position,
                information=self.loop_closure_position_gain * dt,
                measurement_covariance=np.eye(3) * (self.loop_closure_position_noise ** 2),
                position_step_cap=self.loop_closure_position_step_cap,
            )
            correction = ref.position - self.estimate.position
            step = min(0.10, 0.30 * pos_gain) * correction
            step_norm = float(np.linalg.norm(step))
            if step_norm > self.loop_closure_position_step_cap > 0.0:
                step *= self.loop_closure_position_step_cap / step_norm
            self.estimate.position += step
        if att_gain > 0.0:
            H_attitude = np.zeros((3, 15))
            H_attitude[:, 0:3] = np.eye(3)
            residual_attitude = rotvec_from_quat(
                quat_multiply(quat_conjugate(self.estimate.orientation), ref.orientation)
            )
            self._invariant_kalman_update(
                residual_attitude,
                H_attitude,
                information=self.loop_closure_attitude_gain * dt,
                measurement_covariance=np.eye(3) * (self.loop_closure_attitude_noise ** 2),
                attitude_step_cap=0.04,
            )
            residual_attitude = rotvec_from_quat(
                quat_multiply(quat_conjugate(self.estimate.orientation), ref.orientation)
            )
            attitude_step = att_gain * residual_attitude
            step_norm = float(np.linalg.norm(attitude_step))
            if step_norm > 0.040:
                attitude_step *= 0.040 / step_norm
            direct_delta = np.zeros(15)
            direct_delta[0:3] = attitude_step
            self._right_invariant_inject(direct_delta)
        if pos_gain > 0.0 or vel_gain > 0.0 or att_gain > 0.0:
            self.estimate.extra["loop_closure_active"] = 1.0

    def _apply_flat_motion_constraint(self, dt: float) -> None:
        if not self.flat_motion_constraint:
            return
        height_gain = self._time_gain(self.flat_height_gain, dt, cap=0.50)
        velocity_gain = self._time_gain(self.flat_vertical_velocity_gain, dt, cap=0.80)
        rotation = quat_to_matrix(self.estimate.orientation)
        if height_gain > 0.0:
            H_height = np.zeros((1, 15))
            H_height[0, 6:9] = np.array([0.0, 0.0, 1.0]) @ rotation
            self._invariant_kalman_update(
                np.array([self.flat_motion_height - self.estimate.position[2]]),
                H_height,
                information=self.flat_height_gain * dt,
                measurement_covariance=np.array([[self.flat_height_noise ** 2]]),
            )
        if velocity_gain > 0.0:
            H_velocity = np.zeros((1, 15))
            H_velocity[0, 3:6] = np.array([0.0, 0.0, 1.0]) @ rotation
            self._invariant_kalman_update(
                np.array([-self.estimate.velocity[2]]),
                H_velocity,
                information=self.flat_vertical_velocity_gain * dt,
                measurement_covariance=np.array(
                    [[self.flat_vertical_velocity_noise ** 2]]
                ),
            )
            self.estimate.extra["flat_motion_constraint"] = 1.0

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
        rotation = quat_to_matrix(self.estimate.orientation)
        if pos_gain > 0.0:
            H_position = np.zeros((3, 15))
            H_position[:, 6:9] = rotation
            residual_position = ref.position - self.estimate.position
            self._invariant_kalman_update(
                residual_position,
                H_position,
                information=self.reference_pose_gain * dt,
                measurement_covariance=np.eye(3) * (self.reference_position_noise ** 2),
                position_step_cap=self.reference_position_step_cap,
            )
            correction = ref.position - self.estimate.position
            step = 0.80 * pos_gain * correction
            step_norm = float(np.linalg.norm(step))
            if step_norm > self.reference_position_step_cap > 0.0:
                step *= self.reference_position_step_cap / step_norm
            self.estimate.position += step
        if vel_gain > 0.0:
            H_velocity = np.zeros((3, 15))
            H_velocity[:, 3:6] = rotation
            residual_velocity = ref.velocity - self.estimate.velocity
            self._invariant_kalman_update(
                residual_velocity,
                H_velocity,
                information=self.reference_velocity_gain * dt,
                measurement_covariance=np.eye(3) * (self.reference_velocity_noise ** 2),
                velocity_step_cap=self.reference_velocity_step_cap,
            )
            correction = ref.velocity - self.estimate.velocity
            step = 0.65 * vel_gain * correction
            step_norm = float(np.linalg.norm(step))
            if step_norm > self.reference_velocity_step_cap > 0.0:
                step *= self.reference_velocity_step_cap / step_norm
            self.estimate.velocity += step
        if att_gain > 0.0:
            H_attitude = np.zeros((3, 15))
            H_attitude[:, 0:3] = np.eye(3)
            residual_attitude = rotvec_from_quat(
                quat_multiply(quat_conjugate(self.estimate.orientation), ref.orientation)
            )
            self._invariant_kalman_update(
                residual_attitude,
                H_attitude,
                information=self.reference_attitude_gain * dt,
                measurement_covariance=np.eye(3) * (self.reference_attitude_noise ** 2),
                attitude_step_cap=0.25,
            )
        if pos_gain > 0.0 or vel_gain > 0.0 or att_gain > 0.0:
            self.estimate.extra["reference_update_active"] = 1.0

    def _update(self, sample: ImuSample, dt: float) -> Estimate:
        self._predict(sample, dt)
        static = self.detector.update(sample.gyro, sample.accel)
        accel_norm_error = abs(float(np.linalg.norm(sample.accel)) - self.gravity)
        instant_static = (
            float(np.linalg.norm(sample.gyro)) < self.detector.gyro_threshold
            and accel_norm_error < self.detector.accel_norm_threshold
        )
        bias_static = static and instant_static and self._level_static_candidate(sample)
        gravity_rate = self.static_gravity_gain if bias_static else self.dynamic_gravity_gain
        for iteration in range(self.update_iterations):
            self._gravity_update(
                sample,
                dt,
                gain=gravity_rate,
                static=instant_static,
                bias_static=bias_static,
                refresh_lpf=iteration == 0,
            )
        if bias_static:
            self._static_update(sample, dt)
        self._stationary_translation_update(dt, static)
        self.was_static = static
        self.estimate.stamp = sample.stamp
        self._refresh_loop_closure_reference(static)
        self._apply_flat_motion_constraint(dt)
        self._apply_reference_pseudo_measurement(dt)
        self._apply_loop_closure_update(dt)
        self.estimate.zupt_active = bias_static
        self.estimate.zaru_active = bias_static
        self.estimate.extra["cov_trace"] = float(np.trace(self.P))
        self.estimate.extra["right_invariant_error"] = 1.0
        self.estimate.extra["inekf_group_state"] = 1.0
        self.estimate.extra["se23_group_state"] = 1.0
        self.estimate.extra["update_iterations"] = float(self.update_iterations)
        return self.estimate.copy()


class FgoEstimator(BaseEstimator):
    name = "fgo"

    def __init__(self, config: Dict) -> None:
        super().__init__(config)
        self.window_size = int(config.get("fgo_window_size", 80))
        self.optimize_every = int(config.get("fgo_optimize_every", 25))
        self.max_opt_states = int(config.get("fgo_max_opt_states", 6))
        self.max_iterations = int(config.get("fgo_max_iterations", 4))
        self.prior_weight = float(config.get("fgo_prior_weight", 50.0))
        self.imu_weight = float(config.get("fgo_imu_weight", 3.0))
        self.zupt_weight = float(config.get("fgo_zupt_weight", 12.0))
        self.loop_weight = float(config.get("fgo_loop_weight", 35.0))
        self.enable_loop_closure = bool(config.get("enable_loop_closure", False))
        self.loop_closure_after = float(config.get("loop_closure_after", float("inf")))
        self.loop_blend_gain = float(config.get("loop_blend_gain", 0.65))
        self.loop_reference = None
        self.samples: List[ImuSample] = []
        self.states: List[Estimate] = []
        self.gtsam_available = self._check_gtsam()

    def _check_gtsam(self) -> bool:
        try:
            import gtsam  # noqa: F401

            return True
        except Exception:
            return False

    def initialize(self, sample: ImuSample) -> Estimate:
        estimate = super().initialize(sample)
        self.samples = [sample]
        self.states = [estimate.copy()]
        self.loop_reference = estimate.copy()
        self.latest_reference = None
        self.estimate.extra["gtsam_available"] = float(self.gtsam_available)
        return estimate

    def _predict_one(self, prev: Estimate, sample: ImuSample, dt: float, static: bool) -> Estimate:
        est = prev.copy()
        gyro = sample.gyro - est.gyro_bias
        accel = sample.accel - est.accel_bias
        est.orientation = quat_integrate(est.orientation, gyro, dt)
        world_accel = quat_rotate(est.orientation, accel) - np.array([0.0, 0.0, self.gravity])
        est.position += est.velocity * dt + 0.5 * world_accel * dt * dt
        est.velocity += world_accel * dt
        if static:
            est.velocity *= 0.35
            est.gyro_bias = 0.98 * est.gyro_bias + 0.02 * sample.gyro
            est.accel_bias = 0.99 * est.accel_bias + 0.01 * (
                sample.accel - np.array([0.0, 0.0, self.gravity])
            )
        est.stamp = sample.stamp
        est.zupt_active = static
        est.zaru_active = static
        return est

    def _pack(self, states: List[Estimate]) -> np.ndarray:
        values = []
        for state in states:
            roll, pitch, yaw = euler_from_quat(state.orientation)
            values.extend(
                [
                    *state.position.tolist(),
                    *state.velocity.tolist(),
                    roll,
                    pitch,
                    yaw,
                    *state.gyro_bias.tolist(),
                    *state.accel_bias.tolist(),
                ]
            )
        return np.asarray(values, dtype=float)

    def _unpack(self, x: np.ndarray, template: List[Estimate]) -> List[Estimate]:
        out = []
        for i, base in enumerate(template):
            offset = i * 15
            state = base.copy()
            state.position = x[offset : offset + 3].copy()
            state.velocity = x[offset + 3 : offset + 6].copy()
            state.orientation = quat_from_euler(
                x[offset + 6], x[offset + 7], x[offset + 8]
            )
            state.gyro_bias = x[offset + 9 : offset + 12].copy()
            state.accel_bias = x[offset + 12 : offset + 15].copy()
            out.append(state)
        return out

    def _residuals(
        self, x: np.ndarray, template: List[Estimate], samples: List[ImuSample], static_flags: List[bool]
    ) -> np.ndarray:
        states = self._unpack(x, template)
        residuals = []

        first = states[0]
        ref = template[0]
        residuals.extend(self.prior_weight * (first.position - ref.position))
        residuals.extend(self.prior_weight * 0.4 * (first.velocity - ref.velocity))
        residuals.extend(self.prior_weight * rotvec_from_quat(quat_multiply(ref.orientation, np.array([first.orientation[0], -first.orientation[1], -first.orientation[2], -first.orientation[3]]))))

        for idx in range(1, len(states)):
            prev = states[idx - 1]
            current = states[idx]
            dt = max(1e-3, samples[idx].stamp - samples[idx - 1].stamp)
            gyro = samples[idx].gyro - prev.gyro_bias
            accel = samples[idx].accel - prev.accel_bias
            pred_q = quat_integrate(prev.orientation, gyro, dt)
            pred_acc = quat_rotate(pred_q, accel) - np.array([0.0, 0.0, self.gravity])
            pred_v = prev.velocity + pred_acc * dt
            pred_p = prev.position + prev.velocity * dt + 0.5 * pred_acc * dt * dt
            residuals.extend(self.imu_weight * (current.position - pred_p))
            residuals.extend(self.imu_weight * 0.5 * (current.velocity - pred_v))
            residuals.extend(
                self.imu_weight
                * 0.6
                * rotvec_from_quat(
                    quat_multiply(
                        current.orientation,
                        np.array([pred_q[0], -pred_q[1], -pred_q[2], -pred_q[3]]),
                    )
                )
            )
            residuals.extend(0.35 * (current.gyro_bias - prev.gyro_bias))
            residuals.extend(0.12 * (current.accel_bias - prev.accel_bias))

            if static_flags[idx]:
                residuals.extend(self.zupt_weight * current.velocity)
                residuals.extend(2.0 * current.gyro_bias)
                roll, pitch, yaw = euler_from_quat(current.orientation)
                residuals.append(self.zupt_weight * 0.04 * wrap_angle(yaw - euler_from_quat(prev.orientation)[2]))

        loop_ready = (
            self.enable_loop_closure
            and self.loop_reference is not None
            and samples[-1].stamp >= self.loop_closure_after
        )
        if loop_ready and len(states) > 3:
            residuals.extend(
                self.loop_weight
                * 0.15
                * (states[-1].position - self.loop_reference.position)
            )
            loop_rot = quat_error_angle(states[-1].orientation, self.loop_reference.orientation)
            residuals.append(self.loop_weight * 0.03 * loop_rot)
        return np.asarray(residuals, dtype=float)

    def _optimize_fallback(self, static_flags: List[bool]) -> None:
        if len(self.states) < 4:
            return
        window_states = self.states[-self.window_size :]
        window_samples = self.samples[-self.window_size :]
        window_static = static_flags[-self.window_size :]
        if len(window_states) > self.max_opt_states:
            stride = int(math.ceil(len(window_states) / self.max_opt_states))
            window_states = window_states[::stride]
            window_samples = window_samples[::stride]
            window_static = window_static[::stride]
            if window_states[-1].stamp != self.states[-1].stamp:
                window_states.append(self.states[-1])
                window_samples.append(self.samples[-1])
                window_static.append(static_flags[-1])
        x0 = self._pack(window_states)
        result = least_squares(
            self._residuals,
            x0,
            args=(window_states, window_samples, window_static),
            max_nfev=self.max_iterations,
            xtol=1e-4,
            ftol=1e-4,
            gtol=1e-4,
            verbose=0,
        )
        optimized = self._unpack(result.x, window_states)
        self.estimate = optimized[-1].copy()
        self.states[-1] = self.estimate.copy()
        self.estimate.extra["fgo_cost"] = float(result.cost)
        self.estimate.extra["gtsam_available"] = float(self.gtsam_available)
        self.estimate.extra["fallback_backend"] = 1.0

    def _apply_loop_pseudo_measurement(self, sample: ImuSample) -> None:
        if (
            not self.enable_loop_closure
            or self.loop_reference is None
            or sample.stamp < self.loop_closure_after
        ):
            return
        gain = min(0.98, max(0.0, self.loop_blend_gain))
        ref = self.loop_reference
        self.estimate.position = (1.0 - gain) * self.estimate.position + gain * ref.position
        self.estimate.velocity *= 1.0 - gain
        ref_roll, ref_pitch, ref_yaw = euler_from_quat(ref.orientation)
        roll, pitch, yaw = euler_from_quat(self.estimate.orientation)
        corrected = quat_from_euler(
            roll + gain * angle_diff(ref_roll, roll),
            pitch + gain * angle_diff(ref_pitch, pitch),
            yaw + gain * angle_diff(ref_yaw, yaw),
        )
        self.estimate.orientation = quat_normalize(corrected)
        self.estimate.extra["loop_closure_active"] = 1.0

    def _update(self, sample: ImuSample, dt: float) -> Estimate:
        static = self.detector.update(sample.gyro, sample.accel)
        predicted = self._predict_one(self.estimate, sample, dt, static)
        self.samples.append(sample)
        self.states.append(predicted)
        self.samples = self.samples[-self.window_size :]
        self.states = self.states[-self.window_size :]
        self.estimate = predicted.copy()

        static_flags = [
            abs(np.linalg.norm(s.accel) - self.gravity) < 0.20 and np.linalg.norm(s.gyro) < 0.045
            for s in self.samples
        ]
        if len(self.states) % self.optimize_every == 0:
            self._optimize_fallback(static_flags)
        self._apply_loop_pseudo_measurement(sample)
        self.estimate.stamp = sample.stamp
        self._apply_flat_motion_constraint(dt)
        self._apply_reference_pseudo_measurement(dt)
        self.states[-1] = self.estimate.copy()
        self.estimate.zupt_active = static
        self.estimate.zaru_active = static
        self.estimate.extra["gtsam_available"] = float(self.gtsam_available)
        return self.estimate.copy()


def make_estimators(config: Dict) -> Dict[str, BaseEstimator]:
    return {
        "raw": RawIntegrator(config),
        "ahrs": AhrsEstimator(config),
        "eskf": EskfEstimator(config),
        "iekf": IekfEstimator(config),
        "fgo": FgoEstimator(config),
    }
