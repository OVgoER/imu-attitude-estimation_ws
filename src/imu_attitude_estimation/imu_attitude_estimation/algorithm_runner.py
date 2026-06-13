import os
from typing import Dict

import rclpy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

from .estimators import make_estimators
from .ros_utils import estimate_to_odom, imu_msg_to_sample


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

        config = dict(DEFAULT_CONFIG)
        config["enable_loop_closure"] = bool(self.get_parameter("enable_loop_closure").value)
        config["loop_closure_after"] = float(self.get_parameter("loop_closure_after").value)
        self.estimators = make_estimators(config)
        self.publish_python_fgo = bool(self.get_parameter("publish_python_fgo").value)
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
        if self.publish_python_fgo and not self.estimators["fgo"].gtsam_available:
            self.get_logger().warn(
                "Python gtsam is not installed; FGO uses the built-in SciPy sliding-window "
                "SE(3)+IMU preintegration fallback. Install python3-gtsam for the GTSAM backend."
            )
        if not self.publish_python_fgo:
            self.get_logger().info("Python FGO publisher disabled; /attitude/fgo is expected from the C++ GTSAM node.")
        self.get_logger().info("IMU algorithm runner is ready.")

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
    finally:
        node.destroy_node()
        rclpy.shutdown()
