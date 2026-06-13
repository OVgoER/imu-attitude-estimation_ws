import math
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
    quat_from_accel,
    quat_from_euler,
    quat_from_rotvec,
    quat_integrate,
    quat_multiply,
    quat_normalize,
    quat_rotate,
    rotvec_from_quat,
    wrap_angle,
)


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
        self.static_bias_gain = float(config.get("static_bias_gain", 0.04))
        self.velocity_damping = float(config.get("ahrs_velocity_damping", 0.3))

    def _update(self, sample: ImuSample, dt: float) -> Estimate:
        static = self.detector.update(sample.gyro, sample.accel)
        corrected_gyro = sample.gyro - self.estimate.gyro_bias

        expected_gravity = quat_rotate(self.estimate.orientation, np.array([0.0, 0.0, 1.0]))
        measured_gravity = normalize_vector(sample.accel, fallback=[0.0, 0.0, 1.0])
        error = np.cross(expected_gravity, measured_gravity)

        if static:
            self.estimate.gyro_bias = (
                1.0 - self.static_bias_gain
            ) * self.estimate.gyro_bias + self.static_bias_gain * sample.gyro
            corrected_gyro[2] *= 0.08
        self.estimate.gyro_bias += self.ki * error * dt
        self.estimate.orientation = quat_integrate(
            self.estimate.orientation, corrected_gyro + self.kp * error, dt
        )

        world_accel = quat_rotate(self.estimate.orientation, sample.accel - self.estimate.accel_bias)
        world_accel -= np.array([0.0, 0.0, self.gravity])
        self.estimate.velocity += world_accel * dt
        self.estimate.position += self.estimate.velocity * dt + 0.5 * world_accel * dt * dt
        if static:
            damping = min(1.0, self.velocity_damping * dt + 0.12)
            self.estimate.velocity *= 1.0 - damping
            self.estimate.accel_bias = 0.995 * self.estimate.accel_bias + 0.005 * (
                sample.accel - np.array([0.0, 0.0, self.gravity])
            )
        self.estimate.stamp = sample.stamp
        self.estimate.zupt_active = static
        self.estimate.zaru_active = static
        self.estimate.extra["attitude_error_correction"] = float(np.linalg.norm(error))
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
        self.gravity_gain = float(config.get("eskf_gravity_gain", 0.35))
        self.zupt_gain = float(config.get("eskf_zupt_gain", 0.38))
        self.bias_gain = float(config.get("eskf_bias_gain", 0.025))

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

    def _gravity_update(self, sample: ImuSample, gain: float = None) -> None:
        gain = self.gravity_gain if gain is None else gain
        _, _, yaw = euler_from_quat(self.estimate.orientation)
        gravity_q = quat_from_accel(sample.accel - self.estimate.accel_bias, yaw)
        err = rotvec_from_quat(
            quat_multiply(gravity_q, np.array([self.estimate.orientation[0], -self.estimate.orientation[1], -self.estimate.orientation[2], -self.estimate.orientation[3]]))
        )
        err[2] = 0.0
        self._inject_attitude_error(err, gain)

    def _static_update(self, sample: ImuSample, dt: float) -> None:
        gain = min(0.85, self.zupt_gain + 0.03)
        self.estimate.velocity *= 1.0 - gain
        self.estimate.position += -0.08 * self.estimate.velocity * dt
        self.estimate.gyro_bias = (
            1.0 - self.bias_gain
        ) * self.estimate.gyro_bias + self.bias_gain * sample.gyro
        gravity_body = np.array([0.0, 0.0, self.gravity])
        self.estimate.accel_bias = (
            1.0 - self.bias_gain
        ) * self.estimate.accel_bias + self.bias_gain * (sample.accel - gravity_body)
        roll, pitch, yaw = euler_from_quat(self.estimate.orientation)
        self.estimate.orientation = quat_from_euler(roll, pitch, yaw - 0.9 * sample.gyro[2] * dt)
        self.P[3:6, 3:6] *= 0.45
        self.P[9:15, 9:15] *= 0.98

    def _update(self, sample: ImuSample, dt: float) -> Estimate:
        self._predict(sample, dt)
        static = self.detector.update(sample.gyro, sample.accel)
        self._gravity_update(sample)
        if static:
            self._static_update(sample, dt)
        self.estimate.stamp = sample.stamp
        self.estimate.zupt_active = static
        self.estimate.zaru_active = static
        self.estimate.extra["cov_trace"] = float(np.trace(self.P))
        return self.estimate.copy()


class IekfEstimator(EskfEstimator):
    name = "iekf"

    def __init__(self, config: Dict) -> None:
        super().__init__(config)
        self.gravity_gain = float(config.get("iekf_gravity_gain", 0.48))
        self.zupt_gain = float(config.get("iekf_zupt_gain", 0.46))
        self.bias_gain = float(config.get("iekf_bias_gain", 0.032))

    def _predict(self, sample: ImuSample, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        gyro = sample.gyro - self.estimate.gyro_bias
        accel_body = sample.accel - self.estimate.accel_bias
        midpoint_q = quat_integrate(self.estimate.orientation, gyro, 0.5 * dt)
        world_accel = quat_rotate(midpoint_q, accel_body) - np.array([0.0, 0.0, self.gravity])
        self.estimate.position += self.estimate.velocity * dt + 0.5 * world_accel * dt * dt
        self.estimate.velocity += world_accel * dt
        self.estimate.orientation = quat_integrate(self.estimate.orientation, gyro, dt)
        q_scale = 0.82
        self.P = self.P + np.eye(15) * q_scale * dt * 0.002
        return gyro, accel_body

    def _gravity_update(self, sample: ImuSample, gain: float = None) -> None:
        gain = self.gravity_gain if gain is None else gain
        world_gravity_dir = np.array([0.0, 0.0, 1.0])
        measured_world = normalize_vector(
            quat_rotate(self.estimate.orientation, sample.accel - self.estimate.accel_bias),
            fallback=world_gravity_dir,
        )
        err = np.cross(measured_world, world_gravity_dir)
        err[2] = 0.0
        self._inject_attitude_error(err, gain)


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
        self.states[-1] = self.estimate.copy()
        self.estimate.stamp = sample.stamp
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
