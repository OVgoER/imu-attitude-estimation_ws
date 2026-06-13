import os
from pathlib import Path

import numpy as np
import rclpy
import gz.msgs10.boolean_pb2 as gz_boolean
import gz.msgs10.pose_pb2 as gz_pose
import gz.transport13 as gz_transport
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from .estimator_base import Estimate
from .ros_utils import estimate_to_odom, fill_imu_msg, float_to_time_msg
from .trajectory import imu_measurement_from_state, scenario_state


class BenchmarkDriver(Node):
    def __init__(self) -> None:
        super().__init__("imu_benchmark_driver")
        self.declare_parameter("scenario", "fast_rotation")
        self.declare_parameter("trajectory", "circle")
        self.declare_parameter("duration", 35.0)
        self.declare_parameter("rate_hz", 200.0)
        self.declare_parameter("seed", 7)
        self.declare_parameter("gyro_noise", 0.004)
        self.declare_parameter("accel_noise", 0.08)
        self.declare_parameter("gyro_bias", [0.006, -0.004, 0.010])
        self.declare_parameter("accel_bias", [0.04, -0.02, 0.06])
        self.declare_parameter("publish_clock", True)
        self.declare_parameter("sync_gazebo_pose", True)
        self.declare_parameter("gazebo_sync_backend", "gz_transport")
        self.declare_parameter("gazebo_sync_rate_hz", 30.0)
        self.declare_parameter("gazebo_model_name", "imu_platform")
        self.declare_parameter("gazebo_pose_service", "/world/imu_benchmark/set_pose")

        self.scenario = str(self.get_parameter("scenario").value)
        self.trajectory = str(self.get_parameter("trajectory").value)
        self.duration = float(self.get_parameter("duration").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.dt = 1.0 / max(1.0, self.rate_hz)
        self.t = 0.0
        self.rng = np.random.default_rng(int(self.get_parameter("seed").value))
        self.gyro_bias = np.asarray(self.get_parameter("gyro_bias").value, dtype=float)
        self.accel_bias = np.asarray(self.get_parameter("accel_bias").value, dtype=float)
        self.gyro_noise = float(self.get_parameter("gyro_noise").value)
        self.accel_noise = float(self.get_parameter("accel_noise").value)
        self.publish_clock = bool(self.get_parameter("publish_clock").value)
        self.sync_gazebo_pose = bool(self.get_parameter("sync_gazebo_pose").value)
        self.gazebo_sync_backend = str(self.get_parameter("gazebo_sync_backend").value)
        self.gazebo_sync_period = 1.0 / max(1.0, float(self.get_parameter("gazebo_sync_rate_hz").value))
        self.gazebo_model_name = str(self.get_parameter("gazebo_model_name").value)
        self.gazebo_pose_service = str(self.get_parameter("gazebo_pose_service").value)

        self.imu_pub = self.create_publisher(Imu, "/imu/raw", 50)
        self.gt_pub = self.create_publisher(Odometry, "/ground_truth/odom", 50)
        self.phase_pub = self.create_publisher(String, "/experiment/phase", 10)
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.pose_client = self.create_client(
            SetEntityPose, self.gazebo_pose_service
        )
        self.pose_request_in_flight = None
        self.gz_node = gz_transport.Node() if self.sync_gazebo_pose else None
        self.last_gazebo_sync_log = 0.0
        self.last_gazebo_sync_time = -1e9
        self.timer = self.create_timer(self.dt, self.on_timer)
        self.get_logger().info(
            f"Benchmark driver running scenario={self.scenario}, trajectory={self.trajectory}, "
            f"duration={self.duration:.1f}s, rate={self.rate_hz:.1f}Hz"
        )

    def on_timer(self) -> None:
        if self.t > self.duration:
            self.get_logger().info("Benchmark complete; shutting down driver.")
            rclpy.shutdown()
            return
        state = scenario_state(self.t, self.scenario, self.trajectory)
        stamp = float_to_time_msg(self, self.t)
        if self.publish_clock:
            clock = Clock()
            clock.clock = stamp
            self.clock_pub.publish(clock)
        gyro, accel = imu_measurement_from_state(
            state,
            self.gyro_bias,
            self.accel_bias,
            self.rng,
            self.gyro_noise,
            self.accel_noise,
        )
        imu = fill_imu_msg(Imu(), stamp, "imu_link", state.orientation, gyro, accel)
        self.imu_pub.publish(imu)

        gt = Estimate(
            stamp=self.t,
            position=state.position.copy(),
            velocity=state.velocity.copy(),
            orientation=state.orientation.copy(),
        )
        self.gt_pub.publish(estimate_to_odom(gt, "world", "imu_gt", stamp))
        self.sync_pose_to_gazebo(state)
        phase = String()
        phase.data = state.phase
        self.phase_pub.publish(phase)
        self.t += self.dt

    def sync_pose_to_gazebo(self, state) -> None:
        if not self.sync_gazebo_pose:
            return
        if self.t - self.last_gazebo_sync_time < self.gazebo_sync_period:
            return
        self.last_gazebo_sync_time = self.t
        if self.gazebo_sync_backend == "ros_service":
            self.sync_pose_to_gazebo_ros_service(state)
        else:
            self.sync_pose_to_gazebo_transport(state)

    def sync_pose_to_gazebo_transport(self, state) -> None:
        request = gz_pose.Pose()
        request.name = self.gazebo_model_name
        request.position.x = float(state.position[0])
        request.position.y = float(state.position[1])
        request.position.z = float(state.position[2] + 0.15)
        request.orientation.w = float(state.orientation[0])
        request.orientation.x = float(state.orientation[1])
        request.orientation.y = float(state.orientation[2])
        request.orientation.z = float(state.orientation[3])
        try:
            result = self.gz_node.request(
                self.gazebo_pose_service,
                request,
                gz_pose.Pose,
                gz_boolean.Boolean,
                20,
            )
            if (
                self.t - self.last_gazebo_sync_log > 2.0
                and (not result or not getattr(result[1], "data", False))
            ):
                self.get_logger().warn(
                    f"Gazebo pose sync service {self.gazebo_pose_service} did not accept "
                    f"model {self.gazebo_model_name} yet."
                )
                self.last_gazebo_sync_log = self.t
        except Exception as exc:
            if self.t - self.last_gazebo_sync_log > 2.0:
                self.get_logger().warn(f"Gazebo pose sync failed: {exc}")
                self.last_gazebo_sync_log = self.t

    def sync_pose_to_gazebo_ros_service(self, state) -> None:
        if not self.pose_client.service_is_ready():
            return
        if self.pose_request_in_flight is not None and not self.pose_request_in_flight.done():
            return
        request = SetEntityPose.Request()
        request.entity.name = self.gazebo_model_name
        request.entity.type = request.entity.MODEL
        request.pose.position.x = float(state.position[0])
        request.pose.position.y = float(state.position[1])
        request.pose.position.z = float(state.position[2] + 0.15)
        request.pose.orientation.w = float(state.orientation[0])
        request.pose.orientation.x = float(state.orientation[1])
        request.pose.orientation.y = float(state.orientation[2])
        request.pose.orientation.z = float(state.orientation[3])
        self.pose_request_in_flight = self.pose_client.call_async(request)


def main(args=None) -> None:
    os.environ.setdefault("ROS_LOG_DIR", "/tmp/ros_logs")
    Path("/tmp/ros_logs").mkdir(parents=True, exist_ok=True)
    rclpy.init(args=args)
    node = BenchmarkDriver()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
