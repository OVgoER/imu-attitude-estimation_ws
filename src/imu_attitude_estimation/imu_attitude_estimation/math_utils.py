import math
from typing import Iterable, Tuple

import numpy as np


GRAVITY = 9.80665
EPS = 1e-12


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_vector(vector: Iterable[float], fallback=None) -> np.ndarray:
    arr = np.asarray(vector, dtype=float)
    norm = np.linalg.norm(arr)
    if norm < EPS:
        if fallback is None:
            return arr * 0.0
        return np.asarray(fallback, dtype=float)
    return arr / norm


def skew(vector: Iterable[float]) -> np.ndarray:
    x, y, z = np.asarray(vector, dtype=float)
    return np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ],
        dtype=float,
    )


def quat_normalize(q: Iterable[float]) -> np.ndarray:
    quat = np.asarray(q, dtype=float)
    norm = np.linalg.norm(quat)
    if norm < EPS:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    if quat[0] < 0.0:
        quat = -quat
    return quat / norm


def quat_conjugate(q: Iterable[float]) -> np.ndarray:
    w, x, y, z = np.asarray(q, dtype=float)
    return np.array([w, -x, -y, -z], dtype=float)


def quat_multiply(q1: Iterable[float], q2: Iterable[float]) -> np.ndarray:
    w1, x1, y1, z1 = np.asarray(q1, dtype=float)
    w2, x2, y2, z2 = np.asarray(q2, dtype=float)
    return quat_normalize(
        np.array(
            [
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            ],
            dtype=float,
        )
    )


def quat_from_rotvec(rotvec: Iterable[float]) -> np.ndarray:
    rv = np.asarray(rotvec, dtype=float)
    angle = np.linalg.norm(rv)
    if angle < 1e-10:
        half = 0.5 * rv
        return quat_normalize(np.array([1.0, half[0], half[1], half[2]], dtype=float))
    axis = rv / angle
    half_angle = 0.5 * angle
    return quat_normalize(
        np.array([math.cos(half_angle), *(math.sin(half_angle) * axis)], dtype=float)
    )


def rotvec_from_quat(q: Iterable[float]) -> np.ndarray:
    quat = quat_normalize(q)
    w = clamp(float(quat[0]), -1.0, 1.0)
    vector = quat[1:4]
    s = np.linalg.norm(vector)
    if s < 1e-10:
        return 2.0 * vector
    angle = 2.0 * math.atan2(s, w)
    if angle > math.pi:
        angle -= 2.0 * math.pi
    return angle * vector / s


def quat_to_matrix(q: Iterable[float]) -> np.ndarray:
    w, x, y, z = quat_normalize(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def quat_rotate(q: Iterable[float], vector: Iterable[float]) -> np.ndarray:
    return quat_to_matrix(q) @ np.asarray(vector, dtype=float)


def quat_inverse_rotate(q: Iterable[float], vector: Iterable[float]) -> np.ndarray:
    return quat_to_matrix(q).T @ np.asarray(vector, dtype=float)


def quat_integrate(q: Iterable[float], omega_body: Iterable[float], dt: float) -> np.ndarray:
    if dt <= 0.0:
        return quat_normalize(q)
    delta = quat_from_rotvec(np.asarray(omega_body, dtype=float) * dt)
    return quat_multiply(q, delta)


def quat_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr = math.cos(0.5 * roll)
    sr = math.sin(0.5 * roll)
    cp = math.cos(0.5 * pitch)
    sp = math.sin(0.5 * pitch)
    cy = math.cos(0.5 * yaw)
    sy = math.sin(0.5 * yaw)
    return quat_normalize(
        np.array(
            [
                cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
            ],
            dtype=float,
        )
    )


def euler_from_quat(q: Iterable[float]) -> Tuple[float, float, float]:
    w, x, y, z = quat_normalize(q)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(clamp(sinp, -1.0, 1.0))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def quat_from_accel(accel_body: Iterable[float], yaw: float = 0.0) -> np.ndarray:
    acc = normalize_vector(accel_body, fallback=[0.0, 0.0, GRAVITY])
    roll = math.atan2(acc[1], acc[2])
    pitch = math.atan2(-acc[0], math.sqrt(acc[1] * acc[1] + acc[2] * acc[2]))
    return quat_from_euler(roll, pitch, yaw)


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def angle_diff(a: float, b: float) -> float:
    return wrap_angle(a - b)


def quat_error_angle(q_est: Iterable[float], q_ref: Iterable[float]) -> float:
    dq = quat_multiply(quat_conjugate(q_ref), q_est)
    return float(np.linalg.norm(rotvec_from_quat(dq)))


def ros_quat_to_wxyz(msg_quat) -> np.ndarray:
    return quat_normalize([msg_quat.w, msg_quat.x, msg_quat.y, msg_quat.z])


def wxyz_to_ros_quat(q: Iterable[float], msg_quat) -> None:
    quat = quat_normalize(q)
    msg_quat.w = float(quat[0])
    msg_quat.x = float(quat[1])
    msg_quat.y = float(quat[2])
    msg_quat.z = float(quat[3])
