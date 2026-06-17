import math
from dataclasses import dataclass
from typing import Callable, Tuple

import numpy as np

from .math_utils import (
    GRAVITY,
    angle_diff,
    quat_from_euler,
    quat_inverse_rotate,
    quat_multiply,
    quat_normalize,
)


@dataclass
class TrajectoryState:
    t: float
    phase: str
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    orientation: np.ndarray
    angular_velocity: np.ndarray


def _static_state(t: float, phase: str, position=None, yaw: float = 0.0) -> TrajectoryState:
    pos = np.zeros(3) if position is None else np.asarray(position, dtype=float)
    return TrajectoryState(
        t=t,
        phase=phase,
        position=pos,
        velocity=np.zeros(3),
        acceleration=np.zeros(3),
        orientation=quat_from_euler(0.0, 0.0, yaw),
        angular_velocity=np.zeros(3),
    )


def _rotation_axis_name(axis: str) -> str:
    key = axis.lower()
    if key in ("x", "roll"):
        return "roll"
    if key in ("y", "pitch"):
        return "pitch"
    return "yaw"


def _body_rates_from_euler_rates(
    roll: float,
    pitch: float,
    roll_dot: float,
    pitch_dot: float,
    yaw_dot: float,
) -> np.ndarray:
    """Convert ZYX Euler angle rates to body-frame angular velocity."""
    return np.array(
        [
            roll_dot - yaw_dot * math.sin(pitch),
            pitch_dot * math.cos(roll) + yaw_dot * math.sin(roll) * math.cos(pitch),
            -pitch_dot * math.sin(roll) + yaw_dot * math.cos(roll) * math.cos(pitch),
        ],
        dtype=float,
    )


def _heading_rate(vx: float, vy: float, ax: float, ay: float, fallback: float = 0.0) -> float:
    speed_sq = vx * vx + vy * vy
    if speed_sq < 1e-9:
        return fallback
    return (vx * ay - vy * ax) / speed_sq


def fast_rotation_state(t: float, axis: str = "yaw") -> TrajectoryState:
    if t < 5.0:
        return _static_state(t, "static_init")

    local = max(0.0, t - 5.0)
    axis_name = _rotation_axis_name(axis)
    spin_duration = 20.0
    rate_fast = math.radians(360.0)
    roll = pitch = yaw = 0.0
    roll_dot = pitch_dot = yaw_dot = 0.0
    omega = np.zeros(3)
    angle = rate_fast * min(local, spin_duration)
    if local >= spin_duration:
        phase = "static_final"
    else:
        phase = f"fast_{axis_name}"
    if axis_name == "roll":
        roll = angle
        if phase.startswith("fast_"):
            roll_dot = rate_fast
    elif axis_name == "pitch":
        pitch = angle
        if phase.startswith("fast_"):
            pitch_dot = rate_fast
    else:
        yaw = angle
        if phase.startswith("fast_"):
            yaw_dot = rate_fast
    omega = _body_rates_from_euler_rates(roll, pitch, roll_dot, pitch_dot, yaw_dot)
    return TrajectoryState(
        t=t,
        phase=phase,
        position=np.zeros(3),
        velocity=np.zeros(3),
        acceleration=np.zeros(3),
        orientation=quat_from_euler(roll, pitch, yaw),
        angular_velocity=omega,
    )


def static_dynamic_state(t: float, trajectory: str = "circle") -> TrajectoryState:
    del trajectory
    static_duration = 5.0
    circle_duration = 5.0
    cycle_duration = 15.0
    radius = 1.5
    cycle_index = int(t // cycle_duration)
    local_cycle = t - cycle_index * cycle_duration
    if local_cycle < static_duration:
        phase = "static_init" if cycle_index == 0 else "static_hold"
        return _static_state(t, phase)
    local = local_cycle - static_duration
    if local >= circle_duration:
        return _static_state(t, "static_final")

    u = max(0.0, min(1.0, local / circle_duration))
    smooth = 3.0 * u * u - 2.0 * u * u * u
    smooth_dot = (6.0 * u - 6.0 * u * u) / circle_duration
    smooth_ddot = (6.0 - 12.0 * u) / (circle_duration * circle_duration)
    theta = 2.0 * math.pi * smooth
    theta_dot = 2.0 * math.pi * smooth_dot
    theta_ddot = 2.0 * math.pi * smooth_ddot

    x = radius * (1.0 - math.cos(theta))
    y = radius * math.sin(theta)
    vx = radius * math.sin(theta) * theta_dot
    vy = radius * math.cos(theta) * theta_dot
    ax = radius * (math.cos(theta) * theta_dot * theta_dot + math.sin(theta) * theta_ddot)
    ay = radius * (-math.sin(theta) * theta_dot * theta_dot + math.cos(theta) * theta_ddot)
    roll = 0.04 * math.sin(theta)
    pitch = 0.04 * math.sin(theta)
    yaw = theta
    roll_dot = 0.04 * math.cos(theta) * theta_dot
    pitch_dot = 0.04 * math.cos(theta) * theta_dot
    return TrajectoryState(
        t=t,
        phase="circle_motion",
        position=np.array([x, y, 0.0]),
        velocity=np.array([vx, vy, 0.0]),
        acceleration=np.array([ax, ay, 0.0]),
        orientation=quat_from_euler(roll, pitch, yaw),
        angular_velocity=_body_rates_from_euler_rates(roll, pitch, roll_dot, pitch_dot, theta_dot),
    )


def static_zero_drift_state(t: float) -> TrajectoryState:
    yaw = 0.0
    return _static_state(t, "static_zero_drift", yaw=yaw)


def _smooth_angular_motion(local_t: float, omega: float, ramp_duration: float = 3.0) -> Tuple[float, float, float]:
    if local_t <= 0.0:
        return 0.0, 0.0, 0.0
    if local_t >= ramp_duration:
        theta_ramp = 0.5 * omega * ramp_duration
        return theta_ramp + omega * (local_t - ramp_duration), omega, 0.0

    u = local_t / ramp_duration
    u2 = u * u
    u3 = u2 * u
    u4 = u3 * u
    u5 = u4 * u
    u6 = u5 * u
    speed_scale = 10.0 * u3 - 15.0 * u4 + 6.0 * u5
    accel_scale = (30.0 * u2 - 60.0 * u3 + 30.0 * u4) / ramp_duration
    angle_scale_integral = 2.5 * u4 - 3.0 * u5 + u6
    theta = omega * ramp_duration * angle_scale_integral
    theta_dot = omega * speed_scale
    theta_ddot = omega * accel_scale
    return theta, theta_dot, theta_ddot


def _smooth_unit_interval(local_t: float, duration: float) -> Tuple[float, float, float]:
    if duration <= 0.0:
        return 1.0, 0.0, 0.0
    u = max(0.0, min(1.0, local_t / duration))
    u2 = u * u
    u3 = u2 * u
    u4 = u3 * u
    u5 = u4 * u
    smooth = 10.0 * u3 - 15.0 * u4 + 6.0 * u5
    smooth_dot = (30.0 * u2 - 60.0 * u3 + 30.0 * u4) / duration
    smooth_ddot = (60.0 * u - 180.0 * u2 + 120.0 * u3) / (duration * duration)
    return smooth, smooth_dot, smooth_ddot


def _blend_yaw_from_static(nominal_yaw: float, nominal_yaw_dot: float, blend: float, blend_dot: float) -> Tuple[float, float]:
    blend = max(0.0, min(1.0, blend))
    if blend >= 1.0 - 1e-9:
        return nominal_yaw, nominal_yaw_dot
    yaw_delta = angle_diff(nominal_yaw, 0.0)
    return blend * yaw_delta, blend_dot * yaw_delta + blend * nominal_yaw_dot


def trajectory_tracking_state(t: float, trajectory: str = "circle") -> TrajectoryState:
    if t < 5.0:
        return _static_state(t, "static_init")
    tt = t - 5.0
    omega = 0.28
    if trajectory == "figure8":
        a = 2.0
        tau, tau_dot, tau_ddot = _smooth_angular_motion(tt, 1.0)
        theta = omega * tau
        theta_dot = omega * tau_dot
        theta_ddot = omega * tau_ddot
        x = a * math.sin(theta)
        y = a * math.sin(theta) * math.cos(theta)
        dx_dtheta = a * math.cos(theta)
        dy_dtheta = a * math.cos(2.0 * theta)
        d2x_dtheta2 = -a * math.sin(theta)
        d2y_dtheta2 = -2.0 * a * math.sin(2.0 * theta)
        vx = dx_dtheta * theta_dot
        vy = dy_dtheta * theta_dot
        ax = d2x_dtheta2 * theta_dot * theta_dot + dx_dtheta * theta_ddot
        ay = d2y_dtheta2 * theta_dot * theta_dot + dy_dtheta * theta_ddot
        phase = "figure8"
    elif trajectory == "spiral":
        tau, tau_dot, tau_ddot = _smooth_angular_motion(tt, 1.0)
        theta = omega * tau
        theta_dot = omega * tau_dot
        theta_ddot = omega * tau_ddot
        r = 0.08 * tau
        r_dot = 0.08 * tau_dot
        r_ddot = 0.08 * tau_ddot
        x = r * math.cos(theta)
        y = r * math.sin(theta)
        z = 0.04 * tau
        vx = r_dot * math.cos(theta) - r * math.sin(theta) * theta_dot
        vy = r_dot * math.sin(theta) + r * math.cos(theta) * theta_dot
        vz = 0.04 * tau_dot
        ax = (
            r_ddot * math.cos(theta)
            - 2.0 * r_dot * math.sin(theta) * theta_dot
            - r * math.cos(theta) * theta_dot * theta_dot
            - r * math.sin(theta) * theta_ddot
        )
        ay = (
            r_ddot * math.sin(theta)
            + 2.0 * r_dot * math.cos(theta) * theta_dot
            - r * math.sin(theta) * theta_dot * theta_dot
            + r * math.cos(theta) * theta_ddot
        )
        az = 0.04 * tau_ddot
        roll = 0.08 * math.sin(theta)
        pitch = 0.08 * math.sin(theta)
        nominal_yaw = theta + math.pi / 2.0
        yaw, yaw_dot = _blend_yaw_from_static(nominal_yaw, theta_dot, tau_dot, tau_ddot)
        roll_dot = 0.08 * math.cos(theta) * theta_dot
        pitch_dot = 0.08 * math.cos(theta) * theta_dot
        return TrajectoryState(
            t=t,
            phase="spiral",
            position=np.array([x, y, z]),
            velocity=np.array([vx, vy, vz]),
            acceleration=np.array([ax, ay, az]),
            orientation=quat_from_euler(roll, pitch, yaw),
            angular_velocity=_body_rates_from_euler_rates(roll, pitch, roll_dot, pitch_dot, yaw_dot),
        )
    else:
        r = 2.0
        theta, theta_dot, theta_ddot = _smooth_angular_motion(tt, omega)
        x = r * (1.0 - math.cos(theta))
        y = r * math.sin(theta)
        vx = r * math.sin(theta) * theta_dot
        vy = r * math.cos(theta) * theta_dot
        ax = r * (math.cos(theta) * theta_dot * theta_dot + math.sin(theta) * theta_ddot)
        ay = r * (-math.sin(theta) * theta_dot * theta_dot + math.cos(theta) * theta_ddot)
        roll = 0.06 * math.sin(theta)
        pitch = 0.04 * math.sin(theta)
        yaw = theta
        roll_dot = 0.06 * math.cos(theta) * theta_dot
        pitch_dot = 0.04 * math.cos(theta) * theta_dot
        return TrajectoryState(
            t=t,
            phase="circle",
            position=np.array([x, y, 0.0]),
            velocity=np.array([vx, vy, 0.0]),
            acceleration=np.array([ax, ay, 0.0]),
            orientation=quat_from_euler(roll, pitch, yaw),
            angular_velocity=_body_rates_from_euler_rates(roll, pitch, roll_dot, pitch_dot, theta_dot),
        )
    nominal_yaw = math.atan2(vy, vx) if abs(vx) + abs(vy) > 1e-9 else 0.0
    nominal_yaw_dot = _heading_rate(vx, vy, ax, ay, fallback=omega)
    if trajectory == "figure8":
        _, tau_dot, tau_ddot = _smooth_angular_motion(tt, 1.0)
        yaw, yaw_dot = _blend_yaw_from_static(nominal_yaw, nominal_yaw_dot, tau_dot, tau_ddot)
    else:
        yaw, yaw_dot = nominal_yaw, nominal_yaw_dot
    if trajectory == "figure8":
        roll = 0.06 * math.sin(theta)
        pitch = 0.04 * math.sin(theta)
        roll_dot = 0.06 * math.cos(theta) * theta_dot
        pitch_dot = 0.04 * math.cos(theta) * theta_dot
    else:
        roll = 0.06 * math.sin(omega * tt)
        pitch = 0.04 * math.cos(omega * tt)
        roll_dot = 0.06 * omega * math.cos(omega * tt)
        pitch_dot = -0.04 * omega * math.sin(omega * tt)
    return TrajectoryState(
        t=t,
        phase=phase,
        position=np.array([x, y, 0.0]),
        velocity=np.array([vx, vy, 0.0]),
        acceleration=np.array([ax, ay, 0.0]),
        orientation=quat_from_euler(roll, pitch, yaw),
        angular_velocity=_body_rates_from_euler_rates(roll, pitch, roll_dot, pitch_dot, yaw_dot),
    )


def loop_a_state(t: float) -> TrajectoryState:
    if t < 5.0:
        return _static_state(t, "static_init")
    total = 40.0
    local = min(max(t - 5.0, 0.0), total)
    if t > 45.0:
        return _static_state(t, "static_final")

    progress, progress_dot, progress_ddot = _smooth_unit_interval(local, total)
    theta = 2.0 * math.pi * progress
    theta_dot = 2.0 * math.pi * progress_dot
    theta_ddot = 2.0 * math.pi * progress_ddot

    # A closed 3D curve with zero velocity/acceleration at A. The harmonics make
    # the loop exercise vertical motion and changing curvature without using
    # hard discontinuities at the static/motion boundaries.
    x = 1.40 * math.sin(theta) + 0.55 * math.sin(2.0 * theta) - 0.25 * math.sin(3.0 * theta)
    y = 1.10 * (1.0 - math.cos(theta)) + 0.45 * math.sin(2.0 * theta) + 0.20 * math.sin(3.0 * theta)
    z = 0.45 * math.sin(theta) + 0.35 * math.sin(2.0 * theta) + 0.18 * (1.0 - math.cos(3.0 * theta))

    dx = 1.40 * math.cos(theta) + 1.10 * math.cos(2.0 * theta) - 0.75 * math.cos(3.0 * theta)
    dy = 1.10 * math.sin(theta) + 0.90 * math.cos(2.0 * theta) + 0.60 * math.cos(3.0 * theta)
    dz = 0.45 * math.cos(theta) + 0.70 * math.cos(2.0 * theta) + 0.54 * math.sin(3.0 * theta)

    ddx = -1.40 * math.sin(theta) - 2.20 * math.sin(2.0 * theta) + 2.25 * math.sin(3.0 * theta)
    ddy = 1.10 * math.cos(theta) - 1.80 * math.sin(2.0 * theta) - 1.80 * math.sin(3.0 * theta)
    ddz = -0.45 * math.sin(theta) - 1.40 * math.sin(2.0 * theta) + 1.62 * math.cos(3.0 * theta)

    velocity = np.array([dx, dy, dz]) * theta_dot
    acceleration = np.array([ddx, ddy, ddz]) * theta_dot * theta_dot + np.array([dx, dy, dz]) * theta_ddot

    roll = 0.20 * math.sin(theta) + 0.07 * math.sin(3.0 * theta)
    pitch = 0.16 * math.sin(2.0 * theta) - 0.05 * math.sin(theta)
    yaw = theta + 0.25 * math.sin(2.0 * theta)
    roll_dot = (0.20 * math.cos(theta) + 0.21 * math.cos(3.0 * theta)) * theta_dot
    pitch_dot = (0.32 * math.cos(2.0 * theta) - 0.05 * math.cos(theta)) * theta_dot
    yaw_dot = (1.0 + 0.50 * math.cos(2.0 * theta)) * theta_dot
    return TrajectoryState(
        t=t,
        phase="loop_to_a_3d",
        position=np.array([x, y, z]),
        velocity=velocity,
        acceleration=acceleration,
        orientation=quat_from_euler(roll, pitch, yaw),
        angular_velocity=_body_rates_from_euler_rates(roll, pitch, roll_dot, pitch_dot, yaw_dot),
    )


def scenario_state(t: float, scenario: str, trajectory: str = "circle") -> TrajectoryState:
    if scenario == "fast_rotation":
        return fast_rotation_state(t, trajectory)
    if scenario == "static_dynamic":
        return static_dynamic_state(t, trajectory)
    if scenario in ("static_zero_drift", "zero_drift", "static"):
        return static_zero_drift_state(t)
    if scenario == "trajectory":
        return trajectory_tracking_state(t, trajectory)
    if scenario == "loop_a":
        return loop_a_state(t)
    return trajectory_tracking_state(t, trajectory)


def imu_measurement_from_state(
    state: TrajectoryState,
    gyro_bias: np.ndarray,
    accel_bias: np.ndarray,
    rng: np.random.Generator,
    gyro_noise: float,
    accel_noise: float,
) -> Tuple[np.ndarray, np.ndarray]:
    specific_force_world = state.acceleration + np.array([0.0, 0.0, GRAVITY])
    accel_body = quat_inverse_rotate(state.orientation, specific_force_world)
    gyro = state.angular_velocity.copy()
    gyro += gyro_bias + rng.normal(0.0, gyro_noise, 3)
    accel = accel_body + accel_bias + rng.normal(0.0, accel_noise, 3)
    return gyro, accel
