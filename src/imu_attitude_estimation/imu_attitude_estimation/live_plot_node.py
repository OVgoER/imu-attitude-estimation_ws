import os
from collections import defaultdict, deque

import matplotlib

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .math_utils import angle_diff, euler_from_quat, quat_error_angle
from .ros_utils import odom_to_estimate


class LivePlotNode(Node):
    def __init__(self) -> None:
        super().__init__("imu_live_plot_node")
        self.declare_parameter("max_points", 500)
        self.max_points = int(self.get_parameter("max_points").value)
        self.gt = None
        self.data = defaultdict(lambda: deque(maxlen=self.max_points))
        self.algorithms = ["raw_integrated", "ahrs", "eskf", "iekf", "fgo"]
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_gt, 50)
        for name in self.algorithms:
            topic = f"/attitude/{name}" if name != "raw_integrated" else "/attitude/raw_integrated"
            self.create_subscription(
                Odometry,
                topic,
                lambda msg, key=name: self.on_estimate(key, msg),
                50,
            )

        plt.ion()
        self.metrics = ["att", "roll", "pitch", "yaw", "pos", "speed"]
        self.fig, self.axes = plt.subplots(6, 1, figsize=(12, 13), sharex=True)
        self.fig.canvas.manager.set_window_title("IMU estimation live errors")
        self.lines = {}
        for name in self.algorithms:
            for index, metric in enumerate(self.metrics):
                self.lines[(name, metric)], = self.axes[index].plot([], [], label=name)
        self.axes[0].set_ylabel("att err [rad]")
        self.axes[1].set_ylabel("roll err [rad]")
        self.axes[2].set_ylabel("pitch err [rad]")
        self.axes[3].set_ylabel("yaw err [rad]")
        self.axes[4].set_ylabel("pos err [m]")
        self.axes[5].set_ylabel("speed [m/s]")
        self.axes[5].set_xlabel("time [s]")
        for ax in self.axes:
            ax.grid(True, alpha=0.3)
            ax.legend(loc="upper right", ncol=3, fontsize=8)
        self.fig.tight_layout()
        self.timer = self.create_timer(0.2, self.redraw)
        self.get_logger().info("Live plot window is open.")

    def on_gt(self, msg: Odometry) -> None:
        self.gt = odom_to_estimate(msg)

    def on_estimate(self, name: str, msg: Odometry) -> None:
        if self.gt is None:
            return
        est = odom_to_estimate(msg)
        t = est.stamp
        pos_err = float(np.linalg.norm(est.position - self.gt.position))
        att_err = float(quat_error_angle(est.orientation, self.gt.orientation))
        roll, pitch, yaw = euler_from_quat(est.orientation)
        gt_roll, gt_pitch, gt_yaw = euler_from_quat(self.gt.orientation)
        speed = float(np.linalg.norm(est.velocity))
        self.data[(name, "t")].append(t)
        self.data[(name, "pos")].append(pos_err)
        self.data[(name, "att")].append(att_err)
        self.data[(name, "roll")].append(angle_diff(roll, gt_roll))
        self.data[(name, "pitch")].append(angle_diff(pitch, gt_pitch))
        self.data[(name, "yaw")].append(angle_diff(yaw, gt_yaw))
        self.data[(name, "speed")].append(speed)

    def redraw(self) -> None:
        for name in self.algorithms:
            t = list(self.data[(name, "t")])
            for metric in self.metrics:
                self.lines[(name, metric)].set_data(t, list(self.data[(name, metric)]))
        for ax in self.axes:
            ax.relim()
            ax.autoscale_view()
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


def main(args=None) -> None:
    os.environ.setdefault("ROS_LOG_DIR", "/tmp/ros_logs")
    rclpy.init(args=args)
    node = LivePlotNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            plt.pause(0.01)
    finally:
        node.destroy_node()
        rclpy.shutdown()
