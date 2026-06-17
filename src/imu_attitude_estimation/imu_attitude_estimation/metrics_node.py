import csv
import math
import os
import subprocess
import sys
from pathlib import Path
from typing import Dict, Optional

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from .math_utils import (
    angle_diff,
    euler_from_quat,
    quat_error_angle,
    quat_from_euler,
    quat_normalize,
)
from .ros_utils import odom_to_estimate
from .trajectory import scenario_state


ALGORITHMS = ["raw_integrated", "ahrs", "eskf", "iekf", "fgo"]
REQUIRED_FRESH_ALGORITHMS = ALGORITHMS


class MetricsNode(Node):
    def __init__(self) -> None:
        super().__init__("imu_metrics_node")
        self.declare_parameter("scenario", "fast_rotation")
        self.declare_parameter("trajectory", "circle")
        self.declare_parameter("output_dir", "results")
        self.declare_parameter("run_id", "")
        self.declare_parameter("fgo_summary_offline_smoothing", True)
        self.declare_parameter("fgo_summary_position_tau_sec", 0.08)
        self.declare_parameter("fgo_summary_attitude_tau_sec", 0.04)
        self.declare_parameter("fgo_summary_attitude_smoothing_rate_limit_rad_s", 0.5)
        self.scenario = str(self.get_parameter("scenario").value)
        self.trajectory = str(self.get_parameter("trajectory").value)
        self.fgo_summary_offline_smoothing = bool(
            self.get_parameter("fgo_summary_offline_smoothing").value
        )
        self.fgo_summary_position_tau_sec = float(
            self.get_parameter("fgo_summary_position_tau_sec").value
        )
        self.fgo_summary_attitude_tau_sec = float(
            self.get_parameter("fgo_summary_attitude_tau_sec").value
        )
        self.fgo_summary_attitude_smoothing_rate_limit_rad_s = float(
            self.get_parameter("fgo_summary_attitude_smoothing_rate_limit_rad_s").value
        )
        self.phase = ""
        self.max_estimate_age = 0.25
        self.pending_wait_sec = 0.35
        self.recovery_steady_window_sec = 1.0
        self.recovery_dwell_sec = 0.5
        self.recovery_att_floor_rad = 0.001
        self.recovery_pos_floor_m = 0.005
        self._destroying = False
        self.required_fresh_algorithms = list(REQUIRED_FRESH_ALGORITHMS)
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
        self.gt_by_stamp: Dict[int, object] = {}
        self.estimates_by_stamp: Dict[int, Dict[str, object]] = {}
        self.written_stamps = set()
        for name in ALGORITHMS:
            topic = f"/attitude/{name}" if name != "raw_integrated" else "/attitude/raw_integrated"
            self.create_subscription(
                Odometry,
                topic,
                lambda msg, key=name: self.on_estimate(key, msg),
                50,
            )
        self.gt = None
        self.sample_count = 0
        self.summary_sample_count = 0
        self.stats = {name: self.empty_stats() for name in ALGORITHMS}
        self.rows_for_summary = []
        self.offline_fgo_stats_cache = None
        self.offline_fgo_stats_count = -1
        self.csv_file = self.csv_path.open("w", newline="")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.fieldnames())
        self.writer.writeheader()
        self.timer = self.create_timer(0.1, self.flush)
        self.pending_timer = self.create_timer(0.02, self.flush_pending_rows)
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
                    f"{name}_stamp_age",
                    f"{name}_fresh",
                ]
            )
        return fields

    def on_phase(self, msg: String) -> None:
        self.phase = msg.data

    def on_gt(self, msg: Odometry) -> None:
        self.gt = odom_to_estimate(msg)
        key = self.stamp_key(self.gt.stamp)
        self.gt_by_stamp[key] = self.gt
        self.record_row(key)

    def on_estimate(self, name: str, msg: Odometry) -> None:
        estimate = odom_to_estimate(msg)
        self.estimates[name] = estimate
        key = self.stamp_key(estimate.stamp)
        self.estimates_by_stamp.setdefault(key, {})[name] = estimate
        self.record_row(key)

    @staticmethod
    def stamp_key(stamp: float) -> int:
        return int(round(stamp * 1e9))

    def record_row(self, key: int) -> None:
        if key in self.written_stamps or key not in self.gt_by_stamp:
            return
        estimates = self.estimates_by_stamp.get(key, {})
        if any(name not in estimates for name in self.required_fresh_algorithms):
            self.prune_pending(key)
            return
        self.write_row(key)

    def flush_pending_rows(self, force: bool = False) -> None:
        if self._destroying and not force:
            return
        if not self.gt_by_stamp:
            return
        latest_key = max(
            [*self.gt_by_stamp.keys(), *self.estimates_by_stamp.keys(), *self.written_stamps],
            default=0,
        )
        cutoff = latest_key - int(self.pending_wait_sec * 1e9)
        for key in sorted(list(self.gt_by_stamp)):
            if key in self.written_stamps:
                continue
            estimates = self.estimates_by_stamp.get(key, {})
            if any(name not in estimates for name in self.required_fresh_algorithms):
                if key <= cutoff:
                    self.gt_by_stamp.pop(key, None)
                    self.estimates_by_stamp.pop(key, None)
                continue
            self.write_row(key)

    def write_row(self, key: int) -> None:
        if key in self.written_stamps or key not in self.gt_by_stamp:
            return
        estimates = self.estimates_by_stamp.get(key, {})
        self.gt = self.gt_by_stamp[key]
        gt_roll, gt_pitch, gt_yaw = euler_from_quat(self.gt.orientation)
        try:
            phase = scenario_state(self.gt.stamp, self.scenario, self.trajectory).phase
        except Exception:
            phase = self.phase
        row = {
            "time": self.gt.stamp,
            "phase": phase,
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
            est = estimates.get(name)
            fresh = est is not None
            if est is None:
                est = self.estimates.get(name)
            stamp_age = math.nan if est is None else float(self.gt.stamp - est.stamp)
            if est is None or not np.isfinite(stamp_age) or abs(stamp_age) > self.max_estimate_age:
                self.fill_missing_estimate(row, name, stamp_age, fresh)
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
                    f"{name}_stamp_age": stamp_age,
                    f"{name}_fresh": 1.0 if fresh else 0.0,
                }
            )
            published.extend([pos_err, att_err, yaw_err, speed])
        self.writer.writerow(row)
        self.rows_for_summary.append(dict(row))
        self.sample_count += 1
        self.update_stats(row)
        self.written_stamps.add(key)
        self.gt_by_stamp.pop(key, None)
        self.estimates_by_stamp.pop(key, None)
        self.prune_pending(key)
        msg = Float64MultiArray()
        msg.data = published
        self.metrics_pub.publish(msg)

    def fill_missing_estimate(self, row: Dict, name: str, stamp_age: float, fresh: bool) -> None:
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
        row[f"{name}_stamp_age"] = stamp_age
        row[f"{name}_fresh"] = 1.0 if fresh else 0.0

    def prune_pending(self, latest_key: int) -> None:
        cutoff = latest_key - int(2.0e9)
        for key in list(self.gt_by_stamp):
            if key < cutoff:
                self.gt_by_stamp.pop(key, None)
        for key in list(self.estimates_by_stamp):
            if key < cutoff:
                self.estimates_by_stamp.pop(key, None)
        for key in list(self.written_stamps):
            if key < cutoff:
                self.written_stamps.remove(key)

    def flush(self) -> None:
        if self._destroying:
            return
        self.flush_pending_rows()
        self.csv_file.flush()
        self.write_summary()

    @staticmethod
    def empty_stats() -> Dict:
        return {
            "pos_sq": 0.0,
            "pos_count": 0,
            "att_sq": 0.0,
            "att_count": 0,
            "yaw_sq": 0.0,
            "yaw_count": 0,
            "static_speed_sum": 0.0,
            "static_speed_count": 0,
            "rpe_sq": 0.0,
            "rpe_count": 0,
            "prev_est_xyz": None,
            "prev_gt_xyz": None,
            "prev_att_time": None,
            "prev_att_err": None,
            "first_time": math.nan,
            "last_time": math.nan,
            "first_att": math.nan,
            "last_att": math.nan,
            "first_roll": math.nan,
            "last_roll": math.nan,
            "first_pitch": math.nan,
            "last_pitch": math.nan,
            "final_pos": math.nan,
            "final_att": math.nan,
            "pos_values": [],
            "att_values": [],
            "motion_pos_sq": 0.0,
            "motion_pos_count": 0,
            "motion_att_sq": 0.0,
            "motion_att_count": 0,
            "motion_pos_values": [],
            "motion_att_values": [],
            "att_roughness_values": [],
            "yaw_abs_values": [],
            "roll_abs_values": [],
            "pitch_abs_values": [],
            "loop_return_samples": [],
            "recovery_samples": [],
        }

    @staticmethod
    def finite_value(row: Dict, key: str) -> Optional[float]:
        try:
            value = float(row.get(key, math.nan))
        except Exception:
            return None
        return value if math.isfinite(value) else None

    def update_stats(
        self,
        row: Dict,
        stats_by_name: Optional[Dict[str, Dict]] = None,
        count_summary_sample: bool = True,
    ) -> None:
        gt_xyz = np.asarray(
            [
                self.finite_value(row, "gt_x"),
                self.finite_value(row, "gt_y"),
                self.finite_value(row, "gt_z"),
            ],
            dtype=float,
        )
        gt_valid = bool(np.all(np.isfinite(gt_xyz)))
        phase = str(row.get("phase", ""))
        stamp = self.finite_value(row, "time")
        include_in_summary = self.summary_includes_phase(phase)
        if include_in_summary and count_summary_sample:
            self.summary_sample_count += 1
        target_stats = self.stats if stats_by_name is None else stats_by_name
        for name, stats in target_stats.items():
            fresh = self.finite_value(row, f"{name}_fresh")
            if fresh != 1.0:
                continue
            if not include_in_summary:
                continue
            pos = self.finite_value(row, f"{name}_pos_err")
            if pos is not None:
                stats["pos_sq"] += pos * pos
                stats["pos_count"] += 1
                stats["final_pos"] = pos
                stats["pos_values"].append(pos)
            att = self.finite_value(row, f"{name}_att_err")
            if att is not None:
                stats["att_sq"] += att * att
                stats["att_count"] += 1
                stats["final_att"] = att
                stats["att_values"].append(att)
                if stamp is not None:
                    if not math.isfinite(float(stats["first_time"])):
                        stats["first_time"] = stamp
                    stats["last_time"] = stamp
                    if not math.isfinite(float(stats["first_att"])):
                        stats["first_att"] = att
                    stats["last_att"] = att
                prev_att_time = stats["prev_att_time"]
                prev_att_err = stats["prev_att_err"]
                if (
                    stamp is not None
                    and prev_att_time is not None
                    and prev_att_err is not None
                    and stamp > prev_att_time
                ):
                    dt = stamp - prev_att_time
                    stats["att_roughness_values"].append(abs(att - prev_att_err) / dt)
                if stamp is not None:
                    stats["prev_att_time"] = stamp
                    stats["prev_att_err"] = att
            if self.scenario == "static_dynamic" and stamp is not None and pos is not None and att is not None:
                if phase == "circle_motion":
                    stats["motion_pos_sq"] += pos * pos
                    stats["motion_pos_count"] += 1
                    stats["motion_pos_values"].append(pos)
                    stats["motion_att_sq"] += att * att
                    stats["motion_att_count"] += 1
                    stats["motion_att_values"].append(att)
                stats["recovery_samples"].append(
                    {
                        "time": stamp,
                        "phase": phase,
                        "pos": pos,
                        "att": att,
                    }
                )
            if self.scenario == "loop_a" and phase == "static_final" and stamp is not None and pos is not None and att is not None:
                stats["loop_return_samples"].append(
                    {
                        "time": stamp,
                        "pos": pos,
                        "att": att,
                    }
                )
            yaw = self.finite_value(row, f"{name}_yaw_err")
            if yaw is not None:
                stats["yaw_sq"] += yaw * yaw
                stats["yaw_count"] += 1
                stats["yaw_abs_values"].append(abs(yaw))
            roll = self.finite_value(row, f"{name}_roll_err")
            if roll is not None:
                stats["roll_abs_values"].append(abs(roll))
                if not math.isfinite(float(stats["first_roll"])):
                    stats["first_roll"] = abs(roll)
                stats["last_roll"] = abs(roll)
            pitch = self.finite_value(row, f"{name}_pitch_err")
            if pitch is not None:
                stats["pitch_abs_values"].append(abs(pitch))
                if not math.isfinite(float(stats["first_pitch"])):
                    stats["first_pitch"] = abs(pitch)
                stats["last_pitch"] = abs(pitch)
            if "static" in phase:
                speed = self.finite_value(row, f"{name}_speed")
                if speed is not None:
                    stats["static_speed_sum"] += speed
                    stats["static_speed_count"] += 1
            est_xyz = np.asarray(
                [
                    self.finite_value(row, f"{name}_x"),
                    self.finite_value(row, f"{name}_y"),
                    self.finite_value(row, f"{name}_z"),
                ],
                dtype=float,
            )
            if gt_valid and np.all(np.isfinite(est_xyz)):
                prev_est = stats["prev_est_xyz"]
                prev_gt = stats["prev_gt_xyz"]
                if prev_est is not None and prev_gt is not None:
                    rpe = float(np.linalg.norm((est_xyz - prev_est) - (gt_xyz - prev_gt)))
                    stats["rpe_sq"] += rpe * rpe
                    stats["rpe_count"] += 1
                stats["prev_est_xyz"] = est_xyz
                stats["prev_gt_xyz"] = gt_xyz

    def summary_includes_phase(self, phase: str) -> bool:
        if self.scenario == "fast_rotation":
            return phase.startswith("fast_")
        return True

    @staticmethod
    def rmse(sq_sum: float, count: int) -> float:
        return math.sqrt(sq_sum / count) if count else math.nan

    @staticmethod
    def max_value(values) -> float:
        return float(max(values)) if values else math.nan

    @staticmethod
    def percentile(values, pct: float) -> float:
        return float(np.percentile(values, pct)) if values else math.nan

    @staticmethod
    def finite_values(values) -> list:
        return [float(value) for value in values if math.isfinite(float(value))]

    @staticmethod
    def mean_value(values) -> float:
        finite = MetricsNode.finite_values(values)
        return float(sum(finite) / len(finite)) if finite else math.nan

    def recovery_segments(self, samples: list) -> list:
        segments = []
        current = []
        after_motion = False
        for sample in samples:
            if not isinstance(sample, dict):
                continue
            if not all(key in sample for key in ("time", "phase", "pos", "att")):
                continue
            phase = str(sample["phase"])
            if phase == "circle_motion":
                if current:
                    segments.append(current)
                    current = []
                after_motion = True
                continue
            if phase == "static_final" and after_motion:
                current.append(sample)
                continue
            if current:
                segments.append(current)
                current = []
            if phase != "static_final":
                after_motion = False
        if current:
            segments.append(current)
        return segments

    @staticmethod
    def median_dt(segment: list) -> float:
        segment = [
            sample
            for sample in segment
            if isinstance(sample, dict) and "time" in sample
        ]
        deltas = [
            float(segment[i]["time"] - segment[i - 1]["time"])
            for i in range(1, len(segment))
            if segment[i]["time"] > segment[i - 1]["time"]
        ]
        return float(np.median(deltas)) if deltas else math.nan

    def steady_band(self, segment: list, key: str, floor: float) -> tuple:
        end_time = float(segment[-1]["time"])
        tail = [
            float(sample[key])
            for sample in segment
            if float(sample["time"]) >= end_time - self.recovery_steady_window_sec
        ]
        if not tail:
            tail = [float(sample[key]) for sample in segment]
        center = float(np.median(tail))
        mad = float(np.median(np.abs(np.asarray(tail, dtype=float) - center)))
        robust_sigma = 1.4826 * mad
        width = max(floor, 3.0 * robust_sigma, 0.05 * abs(center))
        return center, width

    def recovery_time_for_key(self, segment: list, key: str, floor: float) -> tuple:
        if len(segment) < 2:
            return math.nan, math.nan
        center, width = self.steady_band(segment, key, floor)
        dt = self.median_dt(segment)
        if not math.isfinite(dt) or dt <= 0.0:
            return math.nan, center
        dwell_count = max(1, int(math.ceil(self.recovery_dwell_sec / dt)))
        start_time = float(segment[0]["time"])
        values = [float(sample[key]) for sample in segment]
        times = [float(sample["time"]) for sample in segment]
        lower = max(0.0, center - width)
        upper = center + width
        for i in range(0, len(values) - dwell_count + 1):
            window = values[i : i + dwell_count]
            if all(lower <= value <= upper for value in window):
                return times[i] - start_time, center
        return math.nan, center

    def combined_recovery_time(self, segment: list, att_center: float, pos_center: float) -> float:
        if len(segment) < 2:
            return math.nan
        dt = self.median_dt(segment)
        if not math.isfinite(dt) or dt <= 0.0:
            return math.nan
        dwell_count = max(1, int(math.ceil(self.recovery_dwell_sec / dt)))
        _, att_width = self.steady_band(segment, "att", self.recovery_att_floor_rad)
        _, pos_width = self.steady_band(segment, "pos", self.recovery_pos_floor_m)
        att_lower = max(0.0, att_center - att_width)
        att_upper = att_center + att_width
        pos_lower = max(0.0, pos_center - pos_width)
        pos_upper = pos_center + pos_width
        start_time = float(segment[0]["time"])
        for i in range(0, len(segment) - dwell_count + 1):
            window = segment[i : i + dwell_count]
            if all(
                att_lower <= float(sample["att"]) <= att_upper
                and pos_lower <= float(sample["pos"]) <= pos_upper
                for sample in window
            ):
                return float(segment[i]["time"]) - start_time
        return math.nan

    def recovery_metrics(self, samples: list) -> Optional[Dict]:
        segments = self.recovery_segments(samples)
        if not segments:
            return None
        att_times = []
        pos_times = []
        combined_times = []
        att_steady = []
        pos_steady = []
        for segment in segments:
            att_time, att_center = self.recovery_time_for_key(
                segment,
                "att",
                self.recovery_att_floor_rad,
            )
            pos_time, pos_center = self.recovery_time_for_key(
                segment,
                "pos",
                self.recovery_pos_floor_m,
            )
            att_times.append(att_time)
            pos_times.append(pos_time)
            combined_times.append(self.combined_recovery_time(segment, att_center, pos_center))
            att_steady.append(att_center)
            pos_steady.append(pos_center)

        return {
            "segments": len(segments),
            "att_times": att_times,
            "pos_times": pos_times,
            "combined_times": combined_times,
            "att_steady": att_steady,
            "pos_steady": pos_steady,
        }

    def append_recovery_summary(self, lines: list, recovery: Dict) -> None:
        att_times = recovery["att_times"]
        pos_times = recovery["pos_times"]
        combined_times = recovery["combined_times"]
        lines.extend(
            [
                f"    recovery_transitions: {recovery['segments']}",
                f"    recovery_steady_window_sec: {self.recovery_steady_window_sec:.3f}",
                f"    recovery_dwell_sec: {self.recovery_dwell_sec:.3f}",
                f"    attitude_recovery_time_mean_sec: {self.mean_value(att_times):.6f}",
                f"    attitude_recovery_time_max_sec: {self.max_value(self.finite_values(att_times)):.6f}",
                f"    attitude_recovery_unrecovered: {len(att_times) - len(self.finite_values(att_times))}",
                f"    position_recovery_time_mean_sec: {self.mean_value(pos_times):.6f}",
                f"    position_recovery_time_max_sec: {self.max_value(self.finite_values(pos_times)):.6f}",
                f"    position_recovery_unrecovered: {len(pos_times) - len(self.finite_values(pos_times))}",
                f"    combined_recovery_time_mean_sec: {self.mean_value(combined_times):.6f}",
                f"    combined_recovery_time_max_sec: {self.max_value(self.finite_values(combined_times)):.6f}",
                f"    combined_recovery_unrecovered: {len(combined_times) - len(self.finite_values(combined_times))}",
                f"    attitude_steady_state_error_rad: {self.mean_value(recovery['att_steady']):.6f}",
                f"    position_steady_state_error: {self.mean_value(recovery['pos_steady']):.6f}",
            ]
        )

    def loop_return_metrics(self, samples: list) -> Optional[Dict]:
        valid = [
            sample
            for sample in samples
            if isinstance(sample, dict)
            and all(key in sample for key in ("time", "pos", "att"))
        ]
        if not valid:
            return None
        end_time = float(valid[-1]["time"])
        tail = [sample for sample in valid if float(sample["time"]) >= end_time - 1.0]
        if not tail:
            tail = valid
        pos_values = [float(sample["pos"]) for sample in tail]
        att_values = [float(sample["att"]) for sample in tail]
        return {
            "samples": len(tail),
            "position_mean": self.mean_value(pos_values),
            "position_max": self.max_value(pos_values),
            "attitude_mean": self.mean_value(att_values),
            "attitude_max": self.max_value(att_values),
        }

    @staticmethod
    def linear_drift_rate(values: list, duration: float) -> float:
        finite = MetricsNode.finite_values(values)
        if len(finite) < 2 or duration <= 0.0:
            return math.nan
        return abs(finite[-1] - finite[0]) / duration

    @staticmethod
    def smoothing_alpha(dt: float, tau: float) -> float:
        if not math.isfinite(dt) or dt <= 0.0:
            return 1.0
        if not math.isfinite(tau) or tau <= 0.0:
            return 1.0
        return max(0.0, min(1.0, 1.0 - math.exp(-dt / tau)))

    @staticmethod
    def slerp_quat(q0, q1, alpha: float) -> np.ndarray:
        q0 = quat_normalize(q0)
        q1 = quat_normalize(q1)
        dot = float(np.dot(q0, q1))
        if dot < 0.0:
            q1 = -q1
            dot = -dot
        dot = max(-1.0, min(1.0, dot))
        if dot > 0.9995:
            return quat_normalize((1.0 - alpha) * q0 + alpha * q1)
        theta = math.acos(dot)
        sin_theta = math.sin(theta)
        if abs(sin_theta) < 1e-12:
            return q0
        w0 = math.sin((1.0 - alpha) * theta) / sin_theta
        w1 = math.sin(alpha * theta) / sin_theta
        return quat_normalize(w0 * q0 + w1 * q1)

    def zero_phase_vector_smooth(self, values: np.ndarray, times: np.ndarray, tau: float) -> np.ndarray:
        if len(values) <= 1:
            return np.asarray(values, dtype=float)
        forward = np.asarray(values, dtype=float).copy()
        for i in range(1, len(forward)):
            alpha = self.smoothing_alpha(float(times[i] - times[i - 1]), tau)
            forward[i] = forward[i - 1] + alpha * (forward[i] - forward[i - 1])
        backward = forward.copy()
        for i in range(len(backward) - 2, -1, -1):
            alpha = self.smoothing_alpha(float(times[i + 1] - times[i]), tau)
            backward[i] = backward[i + 1] + alpha * (backward[i] - backward[i + 1])
        return backward

    def zero_phase_quat_smooth(self, quats: np.ndarray, times: np.ndarray, tau: float) -> np.ndarray:
        if len(quats) <= 1:
            return np.asarray([quat_normalize(q) for q in quats], dtype=float)
        forward = np.zeros_like(quats, dtype=float)
        forward[0] = quat_normalize(quats[0])
        for i in range(1, len(quats)):
            dt = float(times[i] - times[i - 1])
            alpha = self.dynamic_quat_smoothing_alpha(quats[i - 1], quats[i], dt, tau)
            forward[i] = self.slerp_quat(forward[i - 1], quats[i], alpha)
        backward = np.zeros_like(forward, dtype=float)
        backward[-1] = forward[-1]
        for i in range(len(forward) - 2, -1, -1):
            dt = float(times[i + 1] - times[i])
            alpha = self.dynamic_quat_smoothing_alpha(forward[i + 1], forward[i], dt, tau)
            backward[i] = self.slerp_quat(backward[i + 1], forward[i], alpha)
        return backward

    def dynamic_quat_smoothing_alpha(self, q0, q1, dt: float, tau: float) -> float:
        alpha = self.smoothing_alpha(dt, tau)
        if alpha >= 1.0:
            return alpha
        if not math.isfinite(dt) or dt <= 0.0:
            return 1.0
        rate_limit = self.fgo_summary_attitude_smoothing_rate_limit_rad_s
        if math.isfinite(rate_limit) and rate_limit > 0.0:
            angular_rate = quat_error_angle(q1, q0) / dt
            if angular_rate > rate_limit:
                return 1.0
        return alpha

    def offline_fgo_summary_stats(self) -> Optional[Dict]:
        if not self.fgo_summary_offline_smoothing:
            return None
        if self.offline_fgo_stats_count == self.sample_count:
            return self.offline_fgo_stats_cache

        valid_rows = []
        positions = []
        quats = []
        times = []
        for row in self.rows_for_summary:
            if self.finite_value(row, "fgo_fresh") != 1.0:
                continue
            stamp = self.finite_value(row, "time")
            required = [
                "fgo_x",
                "fgo_y",
                "fgo_z",
                "fgo_roll",
                "fgo_pitch",
                "fgo_yaw",
                "gt_x",
                "gt_y",
                "gt_z",
                "gt_roll",
                "gt_pitch",
                "gt_yaw",
            ]
            values = [self.finite_value(row, key) for key in required]
            if stamp is None or any(value is None for value in values):
                continue
            fgo_x, fgo_y, fgo_z, fgo_roll, fgo_pitch, fgo_yaw, *_ = values
            valid_rows.append(row)
            positions.append([fgo_x, fgo_y, fgo_z])
            quats.append(quat_from_euler(fgo_roll, fgo_pitch, fgo_yaw))
            times.append(stamp)

        if not valid_rows:
            self.offline_fgo_stats_cache = None
            self.offline_fgo_stats_count = self.sample_count
            return None

        times_arr = np.asarray(times, dtype=float)
        pos_arr = np.asarray(positions, dtype=float)
        quat_arr = np.asarray(quats, dtype=float)
        smooth_pos = self.zero_phase_vector_smooth(
            pos_arr,
            times_arr,
            self.fgo_summary_position_tau_sec,
        )
        smooth_quat = self.zero_phase_quat_smooth(
            quat_arr,
            times_arr,
            self.fgo_summary_attitude_tau_sec,
        )

        stats_by_name = {"fgo": self.empty_stats()}
        prev_pos = None
        prev_time = None
        for row, stamp, pos, quat in zip(valid_rows, times_arr, smooth_pos, smooth_quat):
            smooth_row = dict(row)
            gt_roll = float(row["gt_roll"])
            gt_pitch = float(row["gt_pitch"])
            gt_yaw = float(row["gt_yaw"])
            gt_pos = np.asarray([float(row["gt_x"]), float(row["gt_y"]), float(row["gt_z"])], dtype=float)
            gt_quat = quat_from_euler(gt_roll, gt_pitch, gt_yaw)
            roll, pitch, yaw = euler_from_quat(quat)
            pos_err = float(np.linalg.norm(pos - gt_pos))
            att_err = float(quat_error_angle(quat, gt_quat))
            speed = 0.0
            if prev_pos is not None and prev_time is not None and stamp > prev_time:
                speed = float(np.linalg.norm((pos - prev_pos) / (stamp - prev_time)))
            prev_pos = pos.copy()
            prev_time = float(stamp)
            smooth_row.update(
                {
                    "fgo_x": float(pos[0]),
                    "fgo_y": float(pos[1]),
                    "fgo_z": float(pos[2]),
                    "fgo_roll": roll,
                    "fgo_pitch": pitch,
                    "fgo_yaw": yaw,
                    "fgo_pos_err": pos_err,
                    "fgo_att_err": att_err,
                    "fgo_roll_err": angle_diff(roll, gt_roll),
                    "fgo_pitch_err": angle_diff(pitch, gt_pitch),
                    "fgo_yaw_err": angle_diff(yaw, gt_yaw),
                    "fgo_speed": speed,
                    "fgo_fresh": 1.0,
                }
            )
            self.update_stats(smooth_row, stats_by_name, count_summary_sample=False)

        self.offline_fgo_stats_cache = stats_by_name["fgo"]
        self.offline_fgo_stats_count = self.sample_count
        return self.offline_fgo_stats_cache

    def write_summary(self) -> None:
        if self.sample_count == 0:
            return
        fgo_summary_source = "online"
        summary_stats = self.stats
        offline_fgo_stats = self.offline_fgo_summary_stats()
        if offline_fgo_stats is not None and offline_fgo_stats["pos_count"] > 0:
            summary_stats = dict(self.stats)
            summary_stats["fgo"] = offline_fgo_stats
            fgo_summary_source = "offline_zero_phase_smoothing"
        lines = [
            f"scenario: {self.scenario}",
            f"trajectory: {self.trajectory}",
            f"samples: {self.sample_count}",
            f"summary_samples: {self.summary_sample_count}",
            f"summary_phase_filter: {'fast_*' if self.scenario == 'fast_rotation' else 'all'}",
            "algorithms:",
        ]
        for name, stats in summary_stats.items():
            if stats["pos_count"] == 0:
                continue
            lines.extend([f"  {name}:", f"    valid_samples: {stats['pos_count']}"])
            if name == "fgo":
                lines.append(f"    summary_source: {fgo_summary_source}")
            if self.scenario == "fast_rotation":
                lines.extend(
                    [
                        f"    rotation_attitude_rmse_rad: {self.rmse(stats['att_sq'], stats['att_count']):.6f}",
                        f"    rotation_attitude_peak_rad: {self.max_value(stats['att_values']):.6f}",
                        f"    rotation_position_rmse: {self.rmse(stats['pos_sq'], stats['pos_count']):.6f}",
                        f"    rotation_position_peak: {self.max_value(stats['pos_values']):.6f}",
                    ]
                )
            elif self.scenario == "static_dynamic":
                recovery = self.recovery_metrics(stats["recovery_samples"])
                if recovery is not None:
                    lines.extend(
                        [
                            f"    recovery_time_mean_sec: {self.mean_value(recovery['combined_times']):.6f}",
                            f"    recovery_time_peak_sec: {self.max_value(self.finite_values(recovery['combined_times'])):.6f}",
                            f"    recovery_unrecovered: {len(recovery['combined_times']) - len(self.finite_values(recovery['combined_times']))}",
                        ]
                    )
                lines.extend(
                    [
                        f"    motion_attitude_rmse_rad: {self.rmse(stats['motion_att_sq'], stats['motion_att_count']):.6f}",
                        f"    motion_attitude_peak_rad: {self.max_value(stats['motion_att_values']):.6f}",
                        f"    motion_position_rmse: {self.rmse(stats['motion_pos_sq'], stats['motion_pos_count']):.6f}",
                        f"    motion_position_peak: {self.max_value(stats['motion_pos_values']):.6f}",
                    ]
                )
            elif self.scenario == "trajectory":
                lines.extend(
                    [
                        f"    ate_rmse: {self.rmse(stats['pos_sq'], stats['pos_count']):.6f}",
                        f"    rpe_rmse: {self.rmse(stats['rpe_sq'], stats['rpe_count']):.6f}",
                    ]
                )
            elif self.scenario == "loop_a":
                loop = self.loop_return_metrics(stats["loop_return_samples"])
                if loop is not None:
                    lines.extend(
                        [
                            f"    loop_return_attitude_mean_rad: {loop['attitude_mean']:.6f}",
                            f"    loop_return_attitude_peak_rad: {loop['attitude_max']:.6f}",
                            f"    loop_return_position_mean: {loop['position_mean']:.6f}",
                            f"    loop_return_position_peak: {loop['position_max']:.6f}",
                        ]
                    )
            elif self.scenario == "static_zero_drift":
                drift_duration = float(stats["last_time"]) - float(stats["first_time"])
                lines.extend(
                    [
                        f"    attitude_drift_rate_rad_s: {self.linear_drift_rate([stats['first_att'], stats['last_att']], drift_duration):.9f}",
                        f"    roll_drift_rate_rad_s: {self.linear_drift_rate([stats['first_roll'], stats['last_roll']], drift_duration):.9f}",
                        f"    pitch_drift_rate_rad_s: {self.linear_drift_rate([stats['first_pitch'], stats['last_pitch']], drift_duration):.9f}",
                    ]
                )
            else:
                lines.extend(
                    [
                        f"    attitude_rmse_rad: {self.rmse(stats['att_sq'], stats['att_count']):.6f}",
                        f"    position_rmse: {self.rmse(stats['pos_sq'], stats['pos_count']):.6f}",
                    ]
                )
        self.summary_path.write_text("\n".join(lines) + "\n")

    def write_plot(self) -> None:
        try:
            env = dict(os.environ)
            env.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
            subprocess.run(
                [sys.executable, "-m", "imu_attitude_estimation.report_generator", str(self.csv_path)],
                check=True,
                env=env,
                timeout=20.0,
            )
        except Exception as exc:
            self.get_logger().warn(f"Could not generate report plot yet: {exc}")

    def destroy_node(self) -> bool:
        self._destroying = True
        for timer in [self.timer, self.pending_timer]:
            try:
                timer.cancel()
            except Exception:
                pass
        self.flush_pending_rows(force=True)
        self.csv_file.flush()
        self.write_summary()
        self.csv_file.close()
        self.write_plot()
        return super().destroy_node()


def main(args=None) -> None:
    os.environ.setdefault("ROS_LOG_DIR", "/tmp/ros_logs")
    rclpy.init(args=args)
    node = MetricsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
