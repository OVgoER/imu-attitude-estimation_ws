import math

import numpy as np

from imu_attitude_estimation.math_utils import (
    euler_from_quat,
    quat_error_angle,
    quat_from_euler,
    quat_from_matrix,
    quat_from_rotvec,
    quat_integrate,
    quat_to_matrix,
    rotvec_from_quat,
)


def test_quat_euler_roundtrip():
    q = quat_from_euler(0.2, -0.1, 0.4)
    roll, pitch, yaw = euler_from_quat(q)
    assert abs(roll - 0.2) < 1e-9
    assert abs(pitch + 0.1) < 1e-9
    assert abs(yaw - 0.4) < 1e-9


def test_rotvec_roundtrip():
    rv = np.array([0.1, -0.2, 0.3])
    assert np.linalg.norm(rotvec_from_quat(quat_from_rotvec(rv)) - rv) < 1e-9


def test_quat_integrate_yaw():
    q = quat_integrate(quat_from_euler(0.0, 0.0, 0.0), [0.0, 0.0, math.pi], 0.5)
    assert abs(quat_error_angle(q, quat_from_euler(0.0, 0.0, math.pi / 2.0))) < 1e-9


def test_quat_matrix_roundtrip():
    q = quat_from_euler(0.3, -0.4, 0.8)
    assert quat_error_angle(quat_from_matrix(quat_to_matrix(q)), q) < 1e-9
