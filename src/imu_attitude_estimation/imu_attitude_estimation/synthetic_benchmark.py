import argparse
import csv
import os
from pathlib import Path

import numpy as np

from .estimators import make_estimators
from .math_utils import angle_diff, euler_from_quat, quat_error_angle
from .report_generator import generate_report
from .trajectory import imu_measurement_from_state, scenario_state
from .estimator_base import ImuSample


def run_synthetic(
    scenario: str,
    trajectory: str,
    duration: float,
    rate_hz: float,
    output_dir: Path,
    run_id: str,
) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / f"{run_id}.csv"
    rng = np.random.default_rng(7)
    gyro_bias = np.array([0.006, -0.004, 0.010])
    accel_bias = np.array([0.04, -0.02, 0.06])
    estimators = make_estimators(
        {
            "gravity": 9.80665,
            "enable_loop_closure": scenario == "loop_a",
            "loop_closure_after": 45.0,
            "static_gyro_threshold": 0.045,
            "static_accel_threshold": 0.22,
            "fgo_optimize_every": 50,
            "fgo_max_opt_states": 5,
            "fgo_max_iterations": 3,
        }
    )
    algorithms = ["raw", "ahrs", "eskf", "iekf", "fgo"]
    fields = ["time", "phase", "scenario", "trajectory", "gt_x", "gt_y", "gt_z", "gt_roll", "gt_pitch", "gt_yaw"]
    for name in ["raw_integrated", "ahrs", "eskf", "iekf", "fgo"]:
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
    dt = 1.0 / rate_hz
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        t = 0.0
        while t <= duration:
            state = scenario_state(t, scenario, trajectory)
            gyro, accel = imu_measurement_from_state(
                state, gyro_bias, accel_bias, rng, gyro_noise=0.004, accel_noise=0.08
            )
            sample = ImuSample(t, gyro, accel)
            gt_roll, gt_pitch, gt_yaw = euler_from_quat(state.orientation)
            row = {
                "time": t,
                "phase": state.phase,
                "scenario": scenario,
                "trajectory": trajectory,
                "gt_x": state.position[0],
                "gt_y": state.position[1],
                "gt_z": state.position[2],
                "gt_roll": gt_roll,
                "gt_pitch": gt_pitch,
                "gt_yaw": gt_yaw,
            }
            for key in algorithms:
                estimate = estimators[key].update(sample)
                out_key = "raw_integrated" if key == "raw" else key
                roll, pitch, yaw = euler_from_quat(estimate.orientation)
                row.update(
                    {
                        f"{out_key}_x": estimate.position[0],
                        f"{out_key}_y": estimate.position[1],
                        f"{out_key}_z": estimate.position[2],
                        f"{out_key}_roll": roll,
                        f"{out_key}_pitch": pitch,
                        f"{out_key}_yaw": yaw,
                        f"{out_key}_pos_err": float(np.linalg.norm(estimate.position - state.position)),
                        f"{out_key}_att_err": quat_error_angle(estimate.orientation, state.orientation),
                        f"{out_key}_roll_err": angle_diff(roll, gt_roll),
                        f"{out_key}_pitch_err": angle_diff(pitch, gt_pitch),
                        f"{out_key}_yaw_err": angle_diff(yaw, gt_yaw),
                        f"{out_key}_speed": float(np.linalg.norm(estimate.velocity)),
                    }
                )
            writer.writerow(row)
            t += dt
    plot_path = generate_report(csv_path)
    print(f"CSV: {csv_path}")
    print(f"Plot: {plot_path}")
    return csv_path


def main(args=None):
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", default="fast_rotation")
    parser.add_argument("--trajectory", default="circle")
    parser.add_argument("--duration", type=float, default=35.0)
    parser.add_argument("--rate-hz", type=float, default=200.0)
    parser.add_argument("--output-dir", type=Path, default=Path("results"))
    parser.add_argument("--run-id", default="")
    parsed = parser.parse_args(args=args)
    run_id = parsed.run_id or f"{parsed.scenario}_{parsed.trajectory}_synthetic"
    run_synthetic(
        parsed.scenario,
        parsed.trajectory,
        parsed.duration,
        parsed.rate_hz,
        parsed.output_dir,
        run_id,
    )
