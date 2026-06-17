import csv
import os
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


class LivePlotNode(Node):
    def __init__(self) -> None:
        super().__init__("imu_live_plot_node")
        self.declare_parameter("max_points", 0)
        self.declare_parameter("poll_period", 0.5)
        self.declare_parameter("output_dir", "results")
        self.declare_parameter("run_id", "")
        self.declare_parameter("csv_path", "")
        self.max_points = int(self.get_parameter("max_points").value)
        self.poll_period = float(self.get_parameter("poll_period").value)
        self.output_dir = Path(str(self.get_parameter("output_dir").value))
        self.run_id = str(self.get_parameter("run_id").value)
        self.csv_path_param = str(self.get_parameter("csv_path").value)
        self.csv_path = self.resolve_csv_path()
        self.wait_logged = False
        self.last_row_count = -1
        self.last_csv_path = None
        self.plot_mode = None
        self.trajectory = ""
        self.data = defaultdict(list)
        self.algorithms = ["raw_integrated", "ahrs", "eskf", "iekf", "fgo"]
        self.trajectory_algorithms = ["ahrs", "eskf", "iekf", "fgo"]
        self.column_suffix = {
            "att": "att_err",
            "roll": "roll_err",
            "pitch": "pitch_err",
            "yaw": "yaw_err",
            "pos": "pos_err",
            "speed": "speed",
        }

        plt.ion()
        self.fig = plt.figure(figsize=(12, 13))
        self.fig.canvas.manager.set_window_title("IMU estimation live results")
        self.axes = []
        self.lines = {}
        self.timer = self.create_timer(self.poll_period, self.poll_csv)
        self.get_logger().info("Live plot window is open and follows the results CSV.")

    def resolve_csv_path(self):
        if self.csv_path_param:
            return Path(self.csv_path_param)
        if self.run_id:
            return self.output_dir / f"{self.run_id}.csv"
        candidates = sorted(
            self.output_dir.glob("*.csv"),
            key=lambda path: path.stat().st_mtime,
            reverse=True,
        )
        return candidates[0] if candidates else None

    def poll_csv(self) -> None:
        csv_path = self.resolve_csv_path()
        if csv_path is None or not csv_path.exists():
            if not self.wait_logged:
                target = csv_path if csv_path is not None else self.output_dir / "*.csv"
                self.get_logger().info(f"Waiting for metrics CSV: {target}")
                self.wait_logged = True
            return
        if csv_path != self.last_csv_path:
            self.get_logger().info(f"Reading live plot data from: {csv_path}")
            self.last_csv_path = csv_path
            self.last_row_count = -1
            self.wait_logged = False

        try:
            rows = self.read_complete_rows(csv_path)
        except Exception as exc:
            self.get_logger().warn(f"Could not read metrics CSV yet: {exc}")
            return
        if len(rows) == self.last_row_count:
            return
        self.last_row_count = len(rows)
        if self.max_points > 0:
            rows = rows[-self.max_points :]
        mode = self.mode_for_rows(rows)
        self.trajectory = self.trajectory_for_rows(rows)
        if mode != self.plot_mode:
            self.configure_plot(mode)
        self.replace_data(rows)
        self.redraw()

    def read_complete_rows(self, csv_path: Path):
        required = ["time"]
        for name in self.algorithms:
            for suffix in self.column_suffix.values():
                required.append(f"{name}_{suffix}")
        with csv_path.open(newline="") as file:
            rows = []
            for row in csv.DictReader(file):
                if all(row.get(column) not in (None, "") for column in required):
                    rows.append(row)
            return rows

    def replace_data(self, rows) -> None:
        self.data = defaultdict(list)
        for row in rows:
            t = self.float_value(row, "time")
            if not np.isfinite(t):
                continue
            for column in ["gt_x", "gt_y", "gt_z"]:
                self.data[("gt", column)].append(self.float_value(row, column))
            for name in self.algorithms:
                self.data[(name, "t")].append(t)
                for axis in ["x", "y", "z"]:
                    self.data[(name, axis)].append(self.float_value(row, f"{name}_{axis}"))
                for metric, suffix in self.column_suffix.items():
                    self.data[(name, metric)].append(
                        self.float_value(row, f"{name}_{suffix}")
                    )

    @staticmethod
    def mode_for_rows(rows) -> str:
        for row in rows:
            if row.get("scenario") == "trajectory":
                return "trajectory"
        return "error"

    @staticmethod
    def trajectory_for_rows(rows) -> str:
        for row in rows:
            trajectory = row.get("trajectory", "")
            if trajectory:
                return trajectory
        return ""

    def configure_plot(self, mode: str) -> None:
        self.fig.clear()
        self.lines = {}
        self.plot_mode = mode
        if mode == "trajectory":
            ax_att = self.fig.add_subplot(2, 2, 1)
            ax_pos = self.fig.add_subplot(2, 2, 2)
            ax_3d = self.fig.add_subplot(2, 1, 2, projection="3d")
            self.axes = [ax_att, ax_pos, ax_3d]
            for name in self.algorithms:
                self.lines[(name, "att")], = ax_att.plot([], [], label=name)
                self.lines[(name, "pos")], = ax_pos.plot([], [], label=name)
            self.lines[("gt", "xyz")], = ax_3d.plot([], [], [], "k--", linewidth=2.0, label="ground_truth")
            for name in self.trajectory_algorithms:
                self.lines[(name, "xyz")], = ax_3d.plot([], [], [], label=name)
            ax_att.set_ylabel("att err [rad]")
            ax_pos.set_ylabel("pos err [m]")
            ax_pos.set_xlabel("time [s]")
            ax_3d.set_xlabel("x [m]")
            ax_3d.set_ylabel("y [m]")
            ax_3d.set_zlabel("z [m]")
            ax_3d.set_title("3D trajectory")
        else:
            self.metrics = ["att", "roll", "pitch", "yaw", "pos", "speed"]
            self.axes = list(self.fig.subplots(6, 1, sharex=True))
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
            ax.legend(loc="best", fontsize=8)
        self.fig.tight_layout()

    @staticmethod
    def float_value(row, column: str) -> float:
        try:
            return float(row[column])
        except Exception:
            return float("nan")

    def redraw(self) -> None:
        if self.plot_mode == "trajectory":
            for name in self.algorithms:
                t = list(self.data[(name, "t")])
                self.lines[(name, "att")].set_data(t, list(self.data[(name, "att")]))
                self.lines[(name, "pos")].set_data(t, list(self.data[(name, "pos")]))
            gt_x = list(self.data[("gt", "gt_x")])
            gt_y = list(self.data[("gt", "gt_y")])
            gt_z = list(self.data[("gt", "gt_z")])
            self.lines[("gt", "xyz")].set_data(gt_x, gt_y)
            self.lines[("gt", "xyz")].set_3d_properties(gt_z)
            for name in self.trajectory_algorithms:
                xs = list(self.data[(name, "x")])
                ys = list(self.data[(name, "y")])
                zs = list(self.data[(name, "z")])
                self.lines[(name, "xyz")].set_data(xs, ys)
                self.lines[(name, "xyz")].set_3d_properties(zs)
        else:
            for name in self.algorithms:
                t = list(self.data[(name, "t")])
                for metric in self.metrics:
                    self.lines[(name, metric)].set_data(t, list(self.data[(name, metric)]))
        for ax in self.axes:
            ax.relim()
            ax.autoscale_view()
            if hasattr(ax, "get_zlim3d"):
                self.set_trajectory_axes_3d(ax)
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def set_trajectory_axes_3d(self, ax) -> None:
        fixed_radius = self.trajectory_axis_radius()
        if fixed_radius is not None:
            ax.set_xlim3d([-fixed_radius, fixed_radius])
            ax.set_ylim3d([-fixed_radius, fixed_radius])
            ax.set_zlim3d([-fixed_radius, fixed_radius])
            ax.set_box_aspect((1.0, 1.0, 1.0))
            return
        self.set_axes_equal_3d(ax)

    def trajectory_axis_radius(self):
        if self.trajectory in {"circle", "figure8"}:
            return 2.0
        if self.trajectory == "spiral":
            return 4.0
        return None

    @staticmethod
    def set_axes_equal_3d(ax) -> None:
        limits = np.asarray([ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d()], dtype=float)
        centers = np.mean(limits, axis=1)
        radius = 0.5 * np.max(limits[:, 1] - limits[:, 0])
        if not np.isfinite(radius) or radius <= 0.0:
            radius = 1.0
        ax.set_xlim3d([centers[0] - radius, centers[0] + radius])
        ax.set_ylim3d([centers[1] - radius, centers[1] + radius])
        ax.set_zlim3d([centers[2] - radius, centers[2] + radius])


def main(args=None) -> None:
    os.environ.setdefault("ROS_LOG_DIR", "/tmp/ros_logs")
    rclpy.init(args=args)
    node = LivePlotNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            plt.pause(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
