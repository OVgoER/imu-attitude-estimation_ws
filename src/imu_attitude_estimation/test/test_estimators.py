import numpy as np

from imu_attitude_estimation.estimator_base import ImuSample
from imu_attitude_estimation.estimators import make_estimators
from imu_attitude_estimation.math_utils import GRAVITY, euler_from_quat


def test_static_bias_reduces_yaw_drift():
    estimators = make_estimators({"enable_loop_closure": True, "fgo_optimize_every": 100000})
    gyro = np.array([0.002, -0.003, 0.02])
    accel = np.array([0.0, 0.0, GRAVITY])
    for i in range(500):
        sample = ImuSample(i * 0.01, gyro, accel)
        for estimator in estimators.values():
            estimator.update(sample)
    raw_yaw = abs(euler_from_quat(estimators["raw"].estimate.orientation)[2])
    ahrs_yaw = abs(euler_from_quat(estimators["ahrs"].estimate.orientation)[2])
    eskf_yaw = abs(euler_from_quat(estimators["eskf"].estimate.orientation)[2])
    assert ahrs_yaw < raw_yaw
    assert eskf_yaw < raw_yaw


def test_zupt_velocity_converges_static():
    estimators = make_estimators({"enable_loop_closure": True, "fgo_optimize_every": 100000})
    accel = np.array([0.0, 0.0, GRAVITY])
    for i in range(300):
        sample = ImuSample(i * 0.01, np.zeros(3), accel)
        for estimator in estimators.values():
            estimator.update(sample)
    assert np.linalg.norm(estimators["eskf"].estimate.velocity) < 0.05
    assert np.linalg.norm(estimators["iekf"].estimate.velocity) < 0.05
