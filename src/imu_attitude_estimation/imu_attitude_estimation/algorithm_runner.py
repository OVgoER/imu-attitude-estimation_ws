import os
from typing import Dict

import rclpy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

from .estimators import make_estimators
from .ros_utils import estimate_to_odom, imu_msg_to_sample, odom_to_estimate


DEFAULT_CONFIG = {
    "gravity": 9.80665,
    "static_gyro_threshold": 0.045,
    "static_accel_threshold": 0.22,
    "static_window": 16,
    "ahrs_kp": 1.6,
    "ahrs_ki": 0.04,
    "static_bias_gain": 0.04,
    "gyro_noise": 0.004,
    "accel_noise": 0.08,
    "gyro_bias_rw": 0.0006,
    "accel_bias_rw": 0.01,
    "enable_loop_closure": False,
    "loop_closure_after": 45.0,
    "flat_motion_height": 0.0,
}


class AlgorithmRunner(Node):
    def __init__(self) -> None:
        super().__init__("imu_algorithm_runner")
        self.declare_parameter("imu_topic", "/imu/raw")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("base_frame_prefix", "imu_estimator")
        self.declare_parameter("enable_loop_closure", False)
        self.declare_parameter("loop_closure_after", 45.0)
        self.declare_parameter("publish_python_fgo", False)
        self.declare_parameter("use_reference_pose", False)
        self.declare_parameter("reference_topic", "/ground_truth/odom")
        self.declare_parameter("reference_pose_gain", 0.0)
        self.declare_parameter("reference_velocity_gain", 0.0)
        self.declare_parameter("reference_attitude_gain", 0.0)
        self.declare_parameter("ahrs_kp", 1.6)
        self.declare_parameter("ahrs_ki", 0.04)
        self.declare_parameter("ahrs_dynamic_kp", -1.0)
        self.declare_parameter("ahrs_accel_rejection", 0.9)
        self.declare_parameter("ahrs_gravity_lpf_rate", 12.0)
        self.declare_parameter("ahrs_static_gravity_lpf_rate", 35.0)
        self.declare_parameter("ahrs_yaw_anchor_gain", 4.0)
        self.declare_parameter("ahrs_velocity_damping", 0.3)
        self.declare_parameter("ahrs_initial_level_prior", False)
        self.declare_parameter("ahrs_static_gyro_bias_rate", 6.0)
        self.declare_parameter("ahrs_static_gyro_bias_min_samples", 30)
        self.declare_parameter("ahrs_static_gyro_bias_max_step", 0.00035)
        self.declare_parameter("ahrs_static_accel_bias_direct_gain", 0.0)
        self.declare_parameter("ahrs_static_accel_bias_horizontal_threshold", 0.45)
        self.declare_parameter("ahrs_initial_static_bias_only", True)
        self.declare_parameter("ahrs_post_motion_static_kp", 0.5)
        self.declare_parameter("ahrs_post_motion_yaw_anchor_gain", 0.5)
        self.declare_parameter("ahrs_motion_gyro_threshold", 0.12)
        self.declare_parameter("ahrs_static_reentry_level_gain", 0.0)
        self.declare_parameter("ahrs_static_reentry_level_max_step", 0.006)
        self.declare_parameter("ahrs_loop_closure_position_gain", 0.0)
        self.declare_parameter("ahrs_loop_closure_velocity_gain", 0.0)
        self.declare_parameter("ahrs_loop_closure_attitude_gain", 0.0)
        self.declare_parameter("ahrs_loop_closure_position_step_cap", 0.0)
        self.declare_parameter("ahrs_loop_closure_velocity_step_cap", 0.0)
        self.declare_parameter("ahrs_loop_closure_attitude_step_cap", 0.0)
        self.declare_parameter("ahrs_loop_closure_reference_time", 4.5)
        self.declare_parameter("static_bias_gain", 0.04)
        self.declare_parameter("ahrs_stationary_translation_constraint", False)
        self.declare_parameter("ahrs_stationary_position_gain", 0.0)
        self.declare_parameter("ahrs_stationary_velocity_gain", 0.0)
        self.declare_parameter("ahrs_stationary_translation_static_only", False)
        self.declare_parameter("eskf_reference_pose_gain", -1.0)
        self.declare_parameter("eskf_reference_velocity_gain", -1.0)
        self.declare_parameter("eskf_reference_attitude_gain", -1.0)
        self.declare_parameter("eskf_gravity_gain", 4.0)
        self.declare_parameter("eskf_dynamic_gravity_gain", -1.0)
        self.declare_parameter("eskf_static_gravity_gain", -1.0)
        self.declare_parameter("eskf_gravity_lpf_rate", 12.0)
        self.declare_parameter("eskf_static_gravity_lpf_rate", 35.0)
        self.declare_parameter("eskf_zupt_gain", 45.0)
        self.declare_parameter("eskf_bias_gain", 1.5)
        self.declare_parameter("eskf_stationary_translation_constraint", False)
        self.declare_parameter("eskf_stationary_position_gain", 0.0)
        self.declare_parameter("eskf_stationary_velocity_gain", 0.0)
        self.declare_parameter("eskf_stationary_translation_static_only", False)
        self.declare_parameter("iekf_reference_pose_gain", -1.0)
        self.declare_parameter("iekf_reference_velocity_gain", -1.0)
        self.declare_parameter("iekf_reference_attitude_gain", -1.0)
        self.declare_parameter("iekf_gravity_gain", 4.0)
        self.declare_parameter("iekf_dynamic_gravity_gain", -1.0)
        self.declare_parameter("iekf_static_gravity_gain", -1.0)
        self.declare_parameter("iekf_gravity_lpf_rate", 12.0)
        self.declare_parameter("iekf_static_gravity_lpf_rate", 35.0)
        self.declare_parameter("iekf_zupt_gain", 45.0)
        self.declare_parameter("iekf_bias_gain", 1.5)
        self.declare_parameter("iekf_gravity_step_cap", 0.16)
        self.declare_parameter("iekf_update_iterations", 2)
        self.declare_parameter("iekf_gravity_measurement_noise", 0.08)
        self.declare_parameter("iekf_zupt_velocity_noise", 0.03)
        self.declare_parameter("iekf_static_gyro_bias_noise", 0.004)
        self.declare_parameter("iekf_static_accel_bias_noise", 0.08)
        self.declare_parameter("iekf_static_accel_bias_direct_gain", 0.0)
        self.declare_parameter("iekf_static_accel_bias_horizontal_threshold", 0.45)
        self.declare_parameter("iekf_stationary_position_noise", 0.03)
        self.declare_parameter("iekf_stationary_velocity_noise", 0.03)
        self.declare_parameter("iekf_reference_position_noise", 0.05)
        self.declare_parameter("iekf_reference_velocity_noise", 0.05)
        self.declare_parameter("iekf_reference_attitude_noise", 0.03)
        self.declare_parameter("iekf_reference_position_step_cap", 0.05)
        self.declare_parameter("iekf_reference_velocity_step_cap", 0.12)
        self.declare_parameter("iekf_loop_closure_position_gain", 0.0)
        self.declare_parameter("iekf_loop_closure_velocity_gain", 0.0)
        self.declare_parameter("iekf_loop_closure_attitude_gain", 0.0)
        self.declare_parameter("iekf_loop_closure_position_noise", 0.05)
        self.declare_parameter("iekf_loop_closure_velocity_noise", 0.05)
        self.declare_parameter("iekf_loop_closure_attitude_noise", 0.08)
        self.declare_parameter("iekf_loop_closure_reference_time", 5.0)
        self.declare_parameter("iekf_stationary_position_step_cap", 0.02)
        self.declare_parameter("iekf_loop_closure_position_step_cap", 0.04)
        self.declare_parameter("iekf_loop_closure_velocity_step_cap", 0.12)
        self.declare_parameter("iekf_initial_level_prior", False)
        self.declare_parameter("iekf_stationary_translation_constraint", False)
        self.declare_parameter("iekf_stationary_position_gain", 0.0)
        self.declare_parameter("iekf_stationary_velocity_gain", 0.0)
        self.declare_parameter("iekf_stationary_translation_static_only", False)
        self.declare_parameter("reference_max_age", 0.05)
        self.declare_parameter("flat_motion_constraint", False)
        self.declare_parameter("flat_height_gain", 0.0)
        self.declare_parameter("flat_vertical_velocity_gain", 0.0)

        config = dict(DEFAULT_CONFIG)
        config["enable_loop_closure"] = bool(self.get_parameter("enable_loop_closure").value)
        config["loop_closure_after"] = float(self.get_parameter("loop_closure_after").value)
        config["reference_pose_gain"] = float(self.get_parameter("reference_pose_gain").value)
        config["reference_velocity_gain"] = float(self.get_parameter("reference_velocity_gain").value)
        config["reference_attitude_gain"] = float(self.get_parameter("reference_attitude_gain").value)
        config["ahrs_kp"] = float(self.get_parameter("ahrs_kp").value)
        config["ahrs_ki"] = float(self.get_parameter("ahrs_ki").value)
        ahrs_dynamic_kp = float(self.get_parameter("ahrs_dynamic_kp").value)
        if ahrs_dynamic_kp >= 0.0:
            config["ahrs_dynamic_kp"] = ahrs_dynamic_kp
        config["ahrs_accel_rejection"] = float(
            self.get_parameter("ahrs_accel_rejection").value
        )
        config["ahrs_gravity_lpf_rate"] = float(
            self.get_parameter("ahrs_gravity_lpf_rate").value
        )
        config["ahrs_static_gravity_lpf_rate"] = float(
            self.get_parameter("ahrs_static_gravity_lpf_rate").value
        )
        config["ahrs_yaw_anchor_gain"] = float(
            self.get_parameter("ahrs_yaw_anchor_gain").value
        )
        config["ahrs_velocity_damping"] = float(
            self.get_parameter("ahrs_velocity_damping").value
        )
        config["ahrs_initial_level_prior"] = bool(
            self.get_parameter("ahrs_initial_level_prior").value
        )
        config["ahrs_static_gyro_bias_rate"] = float(
            self.get_parameter("ahrs_static_gyro_bias_rate").value
        )
        config["ahrs_static_gyro_bias_min_samples"] = int(
            self.get_parameter("ahrs_static_gyro_bias_min_samples").value
        )
        config["ahrs_static_gyro_bias_max_step"] = float(
            self.get_parameter("ahrs_static_gyro_bias_max_step").value
        )
        config["ahrs_static_accel_bias_direct_gain"] = float(
            self.get_parameter("ahrs_static_accel_bias_direct_gain").value
        )
        config["ahrs_static_accel_bias_horizontal_threshold"] = float(
            self.get_parameter("ahrs_static_accel_bias_horizontal_threshold").value
        )
        config["ahrs_initial_static_bias_only"] = bool(
            self.get_parameter("ahrs_initial_static_bias_only").value
        )
        config["ahrs_post_motion_static_kp"] = float(
            self.get_parameter("ahrs_post_motion_static_kp").value
        )
        config["ahrs_post_motion_yaw_anchor_gain"] = float(
            self.get_parameter("ahrs_post_motion_yaw_anchor_gain").value
        )
        config["ahrs_motion_gyro_threshold"] = float(
            self.get_parameter("ahrs_motion_gyro_threshold").value
        )
        config["ahrs_static_reentry_level_gain"] = float(
            self.get_parameter("ahrs_static_reentry_level_gain").value
        )
        config["ahrs_static_reentry_level_max_step"] = float(
            self.get_parameter("ahrs_static_reentry_level_max_step").value
        )
        config["ahrs_loop_closure_position_gain"] = float(
            self.get_parameter("ahrs_loop_closure_position_gain").value
        )
        config["ahrs_loop_closure_velocity_gain"] = float(
            self.get_parameter("ahrs_loop_closure_velocity_gain").value
        )
        config["ahrs_loop_closure_attitude_gain"] = float(
            self.get_parameter("ahrs_loop_closure_attitude_gain").value
        )
        config["ahrs_loop_closure_position_step_cap"] = float(
            self.get_parameter("ahrs_loop_closure_position_step_cap").value
        )
        config["ahrs_loop_closure_velocity_step_cap"] = float(
            self.get_parameter("ahrs_loop_closure_velocity_step_cap").value
        )
        config["ahrs_loop_closure_attitude_step_cap"] = float(
            self.get_parameter("ahrs_loop_closure_attitude_step_cap").value
        )
        config["ahrs_loop_closure_reference_time"] = float(
            self.get_parameter("ahrs_loop_closure_reference_time").value
        )
        config["static_bias_gain"] = float(self.get_parameter("static_bias_gain").value)
        config["ahrs_stationary_translation_constraint"] = bool(
            self.get_parameter("ahrs_stationary_translation_constraint").value
        )
        config["ahrs_stationary_position_gain"] = float(
            self.get_parameter("ahrs_stationary_position_gain").value
        )
        config["ahrs_stationary_velocity_gain"] = float(
            self.get_parameter("ahrs_stationary_velocity_gain").value
        )
        config["ahrs_stationary_translation_static_only"] = bool(
            self.get_parameter("ahrs_stationary_translation_static_only").value
        )
        for name in ["eskf", "iekf"]:
            for key in ["reference_pose_gain", "reference_velocity_gain", "reference_attitude_gain"]:
                value = float(self.get_parameter(f"{name}_{key}").value)
                if value >= 0.0:
                    config[f"{name}_{key}"] = value
        config["eskf_gravity_gain"] = float(self.get_parameter("eskf_gravity_gain").value)
        config["eskf_dynamic_gravity_gain"] = float(
            self.get_parameter("eskf_dynamic_gravity_gain").value
        )
        config["eskf_static_gravity_gain"] = float(
            self.get_parameter("eskf_static_gravity_gain").value
        )
        config["eskf_gravity_lpf_rate"] = float(
            self.get_parameter("eskf_gravity_lpf_rate").value
        )
        config["eskf_static_gravity_lpf_rate"] = float(
            self.get_parameter("eskf_static_gravity_lpf_rate").value
        )
        config["eskf_zupt_gain"] = float(self.get_parameter("eskf_zupt_gain").value)
        config["eskf_bias_gain"] = float(self.get_parameter("eskf_bias_gain").value)
        config["eskf_stationary_translation_constraint"] = bool(
            self.get_parameter("eskf_stationary_translation_constraint").value
        )
        config["eskf_stationary_position_gain"] = float(
            self.get_parameter("eskf_stationary_position_gain").value
        )
        config["eskf_stationary_velocity_gain"] = float(
            self.get_parameter("eskf_stationary_velocity_gain").value
        )
        config["eskf_stationary_translation_static_only"] = bool(
            self.get_parameter("eskf_stationary_translation_static_only").value
        )
        config["iekf_gravity_gain"] = float(self.get_parameter("iekf_gravity_gain").value)
        config["iekf_dynamic_gravity_gain"] = float(
            self.get_parameter("iekf_dynamic_gravity_gain").value
        )
        config["iekf_static_gravity_gain"] = float(
            self.get_parameter("iekf_static_gravity_gain").value
        )
        config["iekf_gravity_lpf_rate"] = float(
            self.get_parameter("iekf_gravity_lpf_rate").value
        )
        config["iekf_static_gravity_lpf_rate"] = float(
            self.get_parameter("iekf_static_gravity_lpf_rate").value
        )
        config["iekf_zupt_gain"] = float(self.get_parameter("iekf_zupt_gain").value)
        config["iekf_bias_gain"] = float(self.get_parameter("iekf_bias_gain").value)
        config["iekf_gravity_step_cap"] = float(
            self.get_parameter("iekf_gravity_step_cap").value
        )
        config["iekf_update_iterations"] = int(
            self.get_parameter("iekf_update_iterations").value
        )
        for key in [
            "gravity_measurement_noise",
            "zupt_velocity_noise",
            "static_gyro_bias_noise",
            "static_accel_bias_noise",
            "static_accel_bias_direct_gain",
            "static_accel_bias_horizontal_threshold",
            "stationary_position_noise",
            "stationary_velocity_noise",
            "reference_position_noise",
            "reference_velocity_noise",
            "reference_attitude_noise",
            "reference_position_step_cap",
            "reference_velocity_step_cap",
            "loop_closure_position_gain",
            "loop_closure_velocity_gain",
            "loop_closure_attitude_gain",
            "loop_closure_position_noise",
            "loop_closure_velocity_noise",
            "loop_closure_attitude_noise",
            "loop_closure_reference_time",
            "stationary_position_step_cap",
            "loop_closure_position_step_cap",
            "loop_closure_velocity_step_cap",
        ]:
            config[f"iekf_{key}"] = float(self.get_parameter(f"iekf_{key}").value)
        config["iekf_initial_level_prior"] = bool(
            self.get_parameter("iekf_initial_level_prior").value
        )
        config["iekf_stationary_translation_constraint"] = bool(
            self.get_parameter("iekf_stationary_translation_constraint").value
        )
        config["iekf_stationary_position_gain"] = float(
            self.get_parameter("iekf_stationary_position_gain").value
        )
        config["iekf_stationary_velocity_gain"] = float(
            self.get_parameter("iekf_stationary_velocity_gain").value
        )
        config["iekf_stationary_translation_static_only"] = bool(
            self.get_parameter("iekf_stationary_translation_static_only").value
        )
        config["reference_max_age"] = float(self.get_parameter("reference_max_age").value)
        config["eskf_flat_motion_constraint"] = bool(self.get_parameter("flat_motion_constraint").value)
        config["iekf_flat_motion_constraint"] = bool(self.get_parameter("flat_motion_constraint").value)
        config["eskf_flat_height_gain"] = float(self.get_parameter("flat_height_gain").value)
        config["iekf_flat_height_gain"] = float(self.get_parameter("flat_height_gain").value)
        config["eskf_flat_vertical_velocity_gain"] = float(
            self.get_parameter("flat_vertical_velocity_gain").value
        )
        config["iekf_flat_vertical_velocity_gain"] = float(
            self.get_parameter("flat_vertical_velocity_gain").value
        )
        config["ahrs_flat_motion_constraint"] = bool(self.get_parameter("flat_motion_constraint").value)
        config["ahrs_flat_height_gain"] = float(self.get_parameter("flat_height_gain").value)
        config["ahrs_flat_vertical_velocity_gain"] = float(
            self.get_parameter("flat_vertical_velocity_gain").value
        )
        self.estimators = make_estimators(config)
        self.publish_python_fgo = bool(self.get_parameter("publish_python_fgo").value)
        self.use_reference_pose = bool(self.get_parameter("use_reference_pose").value)
        self.odom_publishers: Dict[str, rclpy.publisher.Publisher] = {}
        for name in self.estimators:
            if name == "fgo" and not self.publish_python_fgo:
                continue
            topic = f"/attitude/{name}" if name != "raw" else "/attitude/raw_integrated"
            self.odom_publishers[name] = self.create_publisher(Odometry, topic, 20)
        self.bias_pub = self.create_publisher(Vector3Stamped, "/imu/bias_estimate", 20)
        self.zupt_pub = self.create_publisher(Bool, "/imu/zupt_active", 20)
        self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            self.on_imu,
            100,
        )
        if self.use_reference_pose:
            self.create_subscription(
                Odometry,
                str(self.get_parameter("reference_topic").value),
                self.on_reference,
                100,
            )
        if self.publish_python_fgo and not self.estimators["fgo"].gtsam_available:
            self.get_logger().warn(
                "Python gtsam is not installed; FGO uses the built-in SciPy sliding-window "
                "SE(3)+IMU preintegration fallback. Install python3-gtsam for the GTSAM backend."
            )
        if not self.publish_python_fgo:
            self.get_logger().info("Python FGO publisher disabled; /attitude/fgo is expected from the C++ GTSAM node.")
        self.get_logger().info("IMU algorithm runner is ready.")

    def on_reference(self, msg: Odometry) -> None:
        reference = odom_to_estimate(msg)
        for name, estimator in self.estimators.items():
            if name == "fgo" and not self.publish_python_fgo:
                continue
            estimator.update_reference(reference)

    def on_imu(self, msg: Imu) -> None:
        sample = imu_msg_to_sample(msg)
        frame = str(self.get_parameter("world_frame").value)
        prefix = str(self.get_parameter("base_frame_prefix").value)
        zupt_any = False
        for name, estimator in self.estimators.items():
            estimate = estimator.update(sample)
            zupt_any = zupt_any or estimate.zupt_active
            if name == "fgo" and not self.publish_python_fgo:
                continue
            child = f"{prefix}_{name}"
            self.odom_publishers[name].publish(
                estimate_to_odom(estimate, frame, child, msg.header.stamp)
            )
        bias_msg = Vector3Stamped()
        bias_msg.header = msg.header
        fgo_est = self.estimators["fgo"].estimate
        bias_msg.vector.x = float(fgo_est.gyro_bias[0])
        bias_msg.vector.y = float(fgo_est.gyro_bias[1])
        bias_msg.vector.z = float(fgo_est.gyro_bias[2])
        self.bias_pub.publish(bias_msg)
        zupt_msg = Bool()
        zupt_msg.data = bool(zupt_any)
        self.zupt_pub.publish(zupt_msg)


def main(args=None) -> None:
    os.environ.setdefault("ROS_LOG_DIR", "/tmp/ros_logs")
    rclpy.init(args=args)
    node = AlgorithmRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
