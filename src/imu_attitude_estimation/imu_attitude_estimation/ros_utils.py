from typing import Iterable

import numpy as np
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from .estimator_base import Estimate, ImuSample
from .math_utils import quat_normalize, wxyz_to_ros_quat


def time_to_float(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def float_to_time_msg(node, stamp: float):
    msg = node.get_clock().now().to_msg()
    if stamp >= 0.0:
        msg.sec = int(stamp)
        msg.nanosec = int((stamp - int(stamp)) * 1e9)
    return msg


def imu_msg_to_sample(msg: Imu) -> ImuSample:
    return ImuSample(
        stamp=time_to_float(msg.header.stamp),
        gyro=np.array(
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            dtype=float,
        ),
        accel=np.array(
            [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ],
            dtype=float,
        ),
    )


def estimate_to_odom(
    estimate: Estimate,
    frame_id: str,
    child_frame_id: str,
    stamp_msg,
) -> Odometry:
    msg = Odometry()
    msg.header.stamp = stamp_msg
    msg.header.frame_id = frame_id
    msg.child_frame_id = child_frame_id
    msg.pose.pose.position.x = float(estimate.position[0])
    msg.pose.pose.position.y = float(estimate.position[1])
    msg.pose.pose.position.z = float(estimate.position[2])
    wxyz_to_ros_quat(estimate.orientation, msg.pose.pose.orientation)
    msg.twist.twist.linear.x = float(estimate.velocity[0])
    msg.twist.twist.linear.y = float(estimate.velocity[1])
    msg.twist.twist.linear.z = float(estimate.velocity[2])
    msg.twist.twist.angular.x = float(estimate.gyro_bias[0])
    msg.twist.twist.angular.y = float(estimate.gyro_bias[1])
    msg.twist.twist.angular.z = float(estimate.gyro_bias[2])
    covariance = [0.0] * 36
    covariance[0] = 0.05
    covariance[7] = 0.05
    covariance[14] = 0.05
    covariance[21] = 0.02
    covariance[28] = 0.02
    covariance[35] = 0.02
    msg.pose.covariance = covariance
    msg.twist.covariance = covariance
    return msg


def odom_to_estimate(msg: Odometry) -> Estimate:
    est = Estimate()
    est.stamp = time_to_float(msg.header.stamp)
    est.position = np.array(
        [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
        dtype=float,
    )
    est.velocity = np.array(
        [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z],
        dtype=float,
    )
    q = msg.pose.pose.orientation
    est.orientation = quat_normalize([q.w, q.x, q.y, q.z])
    return est


def vector3_to_list(vector_msg) -> list:
    return [float(vector_msg.x), float(vector_msg.y), float(vector_msg.z)]


def fill_imu_msg(
    msg: Imu,
    stamp_msg,
    frame_id: str,
    orientation_wxyz: Iterable[float],
    gyro: Iterable[float],
    accel: Iterable[float],
    orientation_covariance_known: bool = False,
) -> Imu:
    msg.header.stamp = stamp_msg
    msg.header.frame_id = frame_id
    wxyz_to_ros_quat(orientation_wxyz, msg.orientation)
    if orientation_covariance_known:
        msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.02]
    else:
        msg.orientation_covariance[0] = -1.0
    gyro = np.asarray(gyro, dtype=float)
    accel = np.asarray(accel, dtype=float)
    msg.angular_velocity.x = float(gyro[0])
    msg.angular_velocity.y = float(gyro[1])
    msg.angular_velocity.z = float(gyro[2])
    msg.linear_acceleration.x = float(accel[0])
    msg.linear_acceleration.y = float(accel[1])
    msg.linear_acceleration.z = float(accel[2])
    msg.angular_velocity_covariance = [0.0001, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0001]
    msg.linear_acceleration_covariance = [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.02]
    return msg


def estimate_to_transform(
    estimate: Estimate, frame_id: str, child_frame_id: str, stamp_msg
) -> TransformStamped:
    msg = TransformStamped()
    msg.header.stamp = stamp_msg
    msg.header.frame_id = frame_id
    msg.child_frame_id = child_frame_id
    msg.transform.translation.x = float(estimate.position[0])
    msg.transform.translation.y = float(estimate.position[1])
    msg.transform.translation.z = float(estimate.position[2])
    wxyz_to_ros_quat(estimate.orientation, msg.transform.rotation)
    return msg
