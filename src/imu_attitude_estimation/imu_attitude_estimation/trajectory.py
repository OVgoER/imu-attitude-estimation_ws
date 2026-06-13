import math
from dataclasses import dataclass
from typing import Callable, Tuple

import numpy as np

from .math_utils import GRAVITY, quat_from_euler, quat_inverse_rotate, quat_multiply, quat_normalize


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


def fast_rotation_state(t: float) -> TrajectoryState:
    if t < 5.0:
        return _static_state(t, "static_init")
    local = t - 5.0
    rate_slow = math.radians(90.0)
    rate_mid = math.radians(180.0)
    rate_fast = math.radians(360.0)
    roll = pitch = yaw = 0.0
    omega = np.zeros(3)
    phase = "static_end"
    if local < 4.0:
        yaw = rate_mid * local
        omega = np.array([0.0, 0.0, rate_mid])
        phase = "fast_yaw"
    elif local < 8.0:
        tt = local - 4.0
        yaw = rate_mid * 4.0
        pitch = rate_slow * math.sin(math.pi * tt / 4.0)
        omega = np.array([0.0, rate_slow * math.pi / 4.0 * math.cos(math.pi * tt / 4.0), 0.0])
        phase = "fast_pitch"
    elif local < 12.0:
        tt = local - 8.0
        yaw = rate_mid * 4.0
        roll = rate_fast * tt
        omega = np.array([rate_fast, 0.0, 0.0])
        phase = "fast_roll"
    elif local < 18.0:
        tt = local - 12.0
        roll = 0.7 * math.sin(2.4 * tt)
        pitch = 0.55 * math.sin(1.7 * tt + 0.3)
        yaw = rate_mid * 4.0 + 1.4 * math.sin(1.2 * tt)
        omega = np.array(
            [
                0.7 * 2.4 * math.cos(2.4 * tt),
                0.55 * 1.7 * math.cos(1.7 * tt + 0.3),
                1.4 * 1.2 * math.cos(1.2 * tt),
            ]
        )
        phase = "combined_rotation"
    else:
        phase = "static_final"
    return TrajectoryState(
        t=t,
        phase=phase,
        position=np.zeros(3),
        velocity=np.zeros(3),
        acceleration=np.zeros(3),
        orientation=quat_from_euler(roll, pitch, yaw),
        angular_velocity=omega,
    )


def static_dynamic_state(t: float) -> TrajectoryState:
    if t < 5.0:
        return _static_state(t, "static_init")
    cycle = 14.0
    idx = int((t - 5.0) // cycle)
    local = (t - 5.0) % cycle
    base_x = 0.15 * idx
    if local < 7.0:
        return _static_state(t, "static_hold", position=[base_x, 0.0, 0.0], yaw=0.15 * idx)
    tt = local - 7.0
    amp = 0.35
    x = base_x + amp * (1.0 - math.cos(2.0 * math.pi * tt / 7.0))
    y = 0.15 * math.sin(2.0 * math.pi * tt / 7.0)
    vx = amp * (2.0 * math.pi / 7.0) * math.sin(2.0 * math.pi * tt / 7.0)
    vy = 0.15 * (2.0 * math.pi / 7.0) * math.cos(2.0 * math.pi * tt / 7.0)
    ax = amp * (2.0 * math.pi / 7.0) ** 2 * math.cos(2.0 * math.pi * tt / 7.0)
    ay = -0.15 * (2.0 * math.pi / 7.0) ** 2 * math.sin(2.0 * math.pi * tt / 7.0)
    yaw = 0.15 * idx + 0.45 * math.sin(2.0 * math.pi * tt / 7.0)
    yaw_rate = 0.45 * (2.0 * math.pi / 7.0) * math.cos(2.0 * math.pi * tt / 7.0)
    return TrajectoryState(
        t=t,
        phase="moving",
        position=np.array([x, y, 0.0]),
        velocity=np.array([vx, vy, 0.0]),
        acceleration=np.array([ax, ay, 0.0]),
        orientation=quat_from_euler(0.04 * math.sin(tt), 0.05 * math.cos(tt), yaw),
        angular_velocity=np.array([0.04 * math.cos(tt), -0.05 * math.sin(tt), yaw_rate]),
    )


def static_zero_drift_state(t: float) -> TrajectoryState:
    yaw = 0.0
    return _static_state(t, "static_zero_drift", yaw=yaw)


def trajectory_tracking_state(t: float, trajectory: str = "circle") -> TrajectoryState:
    if t < 5.0:
        return _static_state(t, "static_init")
    tt = t - 5.0
    omega = 0.28
    if trajectory == "figure8":
        a = 2.0
        x = a * math.sin(omega * tt)
        y = a * math.sin(omega * tt) * math.cos(omega * tt)
        vx = a * omega * math.cos(omega * tt)
        vy = a * omega * math.cos(2.0 * omega * tt)
        ax = -a * omega * omega * math.sin(omega * tt)
        ay = -2.0 * a * omega * omega * math.sin(2.0 * omega * tt)
        phase = "figure8"
    elif trajectory == "spiral":
        r = 0.08 * tt
        x = r * math.cos(omega * tt)
        y = r * math.sin(omega * tt)
        z = 0.04 * tt
        vx = 0.08 * math.cos(omega * tt) - r * omega * math.sin(omega * tt)
        vy = 0.08 * math.sin(omega * tt) + r * omega * math.cos(omega * tt)
        vz = 0.04
        ax = -2.0 * 0.08 * omega * math.sin(omega * tt) - r * omega * omega * math.cos(omega * tt)
        ay = 2.0 * 0.08 * omega * math.cos(omega * tt) - r * omega * omega * math.sin(omega * tt)
        return TrajectoryState(
            t=t,
            phase="spiral",
            position=np.array([x, y, z]),
            velocity=np.array([vx, vy, vz]),
            acceleration=np.array([ax, ay, 0.0]),
            orientation=quat_from_euler(0.08 * math.sin(omega * tt), 0.08 * math.cos(omega * tt), omega * tt + math.pi / 2.0),
            angular_velocity=np.array([0.08 * omega * math.cos(omega * tt), -0.08 * omega * math.sin(omega * tt), omega]),
        )
    else:
        r = 2.0
        x = r * math.cos(omega * tt)
        y = r * math.sin(omega * tt)
        vx = -r * omega * math.sin(omega * tt)
        vy = r * omega * math.cos(omega * tt)
        ax = -r * omega * omega * math.cos(omega * tt)
        ay = -r * omega * omega * math.sin(omega * tt)
        phase = "circle"
    yaw = math.atan2(vy, vx) if abs(vx) + abs(vy) > 1e-9 else 0.0
    return TrajectoryState(
        t=t,
        phase=phase,
        position=np.array([x, y, 0.0]),
        velocity=np.array([vx, vy, 0.0]),
        acceleration=np.array([ax, ay, 0.0]),
        orientation=quat_from_euler(0.06 * math.sin(omega * tt), 0.04 * math.cos(omega * tt), yaw),
        angular_velocity=np.array([0.06 * omega * math.cos(omega * tt), -0.04 * omega * math.sin(omega * tt), omega]),
    )


def loop_a_state(t: float) -> TrajectoryState:
    if t < 5.0:
        return _static_state(t, "static_init")
    total = 40.0
    local = min(max(t - 5.0, 0.0), total)
    s = local / total
    theta = 2.0 * math.pi * s
    radius = 1.5
    x = radius * (1.0 - math.cos(theta))
    y = radius * math.sin(theta)
    vx = radius * math.sin(theta) * 2.0 * math.pi / total
    vy = radius * math.cos(theta) * 2.0 * math.pi / total
    ax = radius * math.cos(theta) * (2.0 * math.pi / total) ** 2
    ay = -radius * math.sin(theta) * (2.0 * math.pi / total) ** 2
    if t > 45.0:
        return _static_state(t, "static_final")
    yaw = theta
    return TrajectoryState(
        t=t,
        phase="loop_to_a",
        position=np.array([x, y, 0.0]),
        velocity=np.array([vx, vy, 0.0]),
        acceleration=np.array([ax, ay, 0.0]),
        orientation=quat_from_euler(0.04 * math.sin(theta), 0.04 * math.cos(theta), yaw),
        angular_velocity=np.array(
            [
                0.04 * (2.0 * math.pi / total) * math.cos(theta),
                -0.04 * (2.0 * math.pi / total) * math.sin(theta),
                2.0 * math.pi / total,
            ]
        ),
    )


def scenario_state(t: float, scenario: str, trajectory: str = "circle") -> TrajectoryState:
    if scenario == "fast_rotation":
        return fast_rotation_state(t)
    if scenario == "static_dynamic":
        return static_dynamic_state(t)
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
