import csv
import math
import os
from pathlib import Path
from typing import Dict, Optional

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from .math_utils import angle_diff, euler_from_quat, quat_error_angle
from .report_generator import generate_report
from .ros_utils import odom_to_estimate


ALGORITHMS = ["raw_integrated", "ahrs", "eskf", "iekf", "fgo"]


class MetricsNode(Node):
    def __init__(self) -> None:
        super().__init__("imu_metrics_node")
        self.declare_parameter("scenario", "fast_rotation")
        self.declare_parameter("trajectory", "circle")
        self.declare_parameter("output_dir", "results")
        self.declare_parameter("run_id", "")
        self.scenario = str(self.get_parameter("scenario").value)
        self.trajectory = str(self.get_parameter("trajectory").value)
        self.phase = ""
        run_id = str(self.get_parameter("run_id").value)
        if not run_id:
            run_id = f"{self.scenario}_{self.trajectory}_{os.getpid()}"
        self.output_dir = Path(str(self.get_parameter("output_dir").value))
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.output_dir / f"{run_id}.csv"
        self.summary_path = self.output_dir / f"{run_id}_summary.yaml"
        self.plot_path = self.output_dir / f"{run_id}_plots.png"
        self.metrics_pub = self.create_publisher(Float64MultiArray, "/metrics/errors", 20)
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_gt, 50)
        self.create_subscription(String, "/experiment/phase", self.on_phase, 10)
        self.estimates: Dict[str, Optional[object]] = {name: None for name in ALGORITHMS}
        for name in ALGORITHMS:
            topic = f"/attitude/{name}" if name != "raw_integrated" else "/attitude/raw_integrated"
            self.create_subscription(
                Odometry,
                topic,
                lambda msg, key=name: self.on_estimate(key, msg),
                50,
            )
        self.gt = None
        self.rows = []
        self.csv_file = self.csv_path.open("w", newline="")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.fieldnames())
        self.writer.writeheader()
        self.timer = self.create_timer(0.1, self.flush)
        self.get_logger().info(f"Metrics CSV: {self.csv_path}")

    def fieldnames(self):
        fields = ["time", "phase", "scenario", "trajectory"]
        fields.extend(["gt_x", "gt_y", "gt_z", "gt_roll", "gt_pitch", "gt_yaw"])
        for name in ALGORITHMS:
            fields.extend(
                [
                    f"{name}_x",
                    f"{name}_y",
                    f"{name}_z",
                    f"{name}_roll",
                    f"{name}_pitch",
                    f"{name}_yaw",
                    f"{name}_pos_err",
                    f"{name}_att_err",
                    f"{name}_roll_err",
                    f"{name}_pitch_err",
                    f"{name}_yaw_err",
                    f"{name}_speed",
                ]
            )
        return fields

    def on_phase(self, msg: String) -> None:
        self.phase = msg.data

    def on_gt(self, msg: Odometry) -> None:
        self.gt = odom_to_estimate(msg)
        self.record_row()

    def on_estimate(self, name: str, msg: Odometry) -> None:
        self.estimates[name] = odom_to_estimate(msg)

    def record_row(self) -> None:
        if self.gt is None:
            return
        gt_roll, gt_pitch, gt_yaw = euler_from_quat(self.gt.orientation)
        row = {
            "time": self.gt.stamp,
            "phase": self.phase,
            "scenario": self.scenario,
            "trajectory": self.trajectory,
            "gt_x": self.gt.position[0],
            "gt_y": self.gt.position[1],
            "gt_z": self.gt.position[2],
            "gt_roll": gt_roll,
            "gt_pitch": gt_pitch,
            "gt_yaw": gt_yaw,
        }
        published = []
        for name in ALGORITHMS:
            est = self.estimates[name]
            if est is None:
                for suffix in [
                    "x",
                    "y",
                    "z",
                    "roll",
                    "pitch",
                    "yaw",
                    "pos_err",
                    "att_err",
                    "roll_err",
                    "pitch_err",
                    "yaw_err",
                    "speed",
                ]:
                    row[f"{name}_{suffix}"] = math.nan
                continue
            roll, pitch, yaw = euler_from_quat(est.orientation)
            pos_err = float(np.linalg.norm(est.position - self.gt.position))
            att_err = float(quat_error_angle(est.orientation, self.gt.orientation))
            roll_err = angle_diff(roll, gt_roll)
            pitch_err = angle_diff(pitch, gt_pitch)
            yaw_err = angle_diff(yaw, gt_yaw)
            speed = float(np.linalg.norm(est.velocity))
            row.update(
                {
                    f"{name}_x": est.position[0],
                    f"{name}_y": est.position[1],
                    f"{name}_z": est.position[2],
                    f"{name}_roll": roll,
                    f"{name}_pitch": pitch,
                    f"{name}_yaw": yaw,
                    f"{name}_pos_err": pos_err,
                    f"{name}_att_err": att_err,
                    f"{name}_roll_err": roll_err,
                    f"{name}_pitch_err": pitch_err,
                    f"{name}_yaw_err": yaw_err,
                    f"{name}_speed": speed,
                }
            )
            published.extend([pos_err, att_err, yaw_err, speed])
        self.writer.writerow(row)
        self.rows.append(row)
        msg = Float64MultiArray()
        msg.data = published
        self.metrics_pub.publish(msg)

    def flush(self) -> None:
        self.csv_file.flush()
        self.write_summary()

    def write_summary(self) -> None:
        if not self.rows:
            return
        lines = [
            f"scenario: {self.scenario}",
            f"trajectory: {self.trajectory}",
            f"samples: {len(self.rows)}",
            "algorithms:",
        ]
        for name in ALGORITHMS:
            pos = np.asarray([r.get(f"{name}_pos_err", math.nan) for r in self.rows], dtype=float)
            att = np.asarray([r.get(f"{name}_att_err", math.nan) for r in self.rows], dtype=float)
            yaw = np.asarray([r.get(f"{name}_yaw_err", math.nan) for r in self.rows], dtype=float)
            speed_static = np.asarray(
                [
                    r.get(f"{name}_speed", math.nan)
                    for r in self.rows
                    if "static" in str(r.get("phase", ""))
                ],
                dtype=float,
            )
            pos = pos[np.isfinite(pos)]
            att = att[np.isfinite(att)]
            yaw = yaw[np.isfinite(yaw)]
            speed_static = speed_static[np.isfinite(speed_static)]
            if len(pos) == 0:
                continue
            lines.extend(
                [
                    f"  {name}:",
                    f"    position_rmse: {float(np.sqrt(np.mean(pos * pos))):.6f}",
                    f"    attitude_rmse_rad: {float(np.sqrt(np.mean(att * att))):.6f}",
                    f"    yaw_rmse_rad: {float(np.sqrt(np.mean(yaw * yaw))):.6f}",
                    f"    final_position_error: {float(pos[-1]):.6f}",
                    f"    final_attitude_error_rad: {float(att[-1]):.6f}",
                    f"    static_speed_mean: {float(np.mean(speed_static)) if len(speed_static) else float('nan'):.6f}",
                ]
            )
        self.summary_path.write_text("\n".join(lines) + "\n")

    def write_plot(self) -> None:
        try:
            generate_report(self.csv_path)
        except Exception as exc:
            self.get_logger().warn(f"Could not generate report plot yet: {exc}")

    def destroy_node(self) -> bool:
        self.flush()
        self.csv_file.close()
        self.write_plot()
        return super().destroy_node()


def main(args=None) -> None:
    os.environ.setdefault("ROS_LOG_DIR", "/tmp/ros_logs")
    rclpy.init(args=args)
    node = MetricsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
