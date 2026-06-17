import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


ALGORITHMS = ["raw_integrated", "ahrs", "eskf", "iekf", "fgo"]
ESTIMATED_ALGORITHMS = ["ahrs", "eskf", "iekf", "fgo"]


def read_csv(path: Path):
    import csv

    with path.open() as f:
        rows = list(csv.DictReader(f))
    return rows


def column(rows, name):
    values = []
    for row in rows:
        try:
            values.append(float(row[name]))
        except Exception:
            values.append(np.nan)
    return np.asarray(values, dtype=float)


def text_value(rows, name, default=""):
    for row in rows:
        value = row.get(name, "")
        if value:
            return value
    return default


def generate_report(csv_path: Path) -> Path:
    rows = read_csv(csv_path)
    if not rows:
        raise RuntimeError(f"No rows in {csv_path}")
    scenario = text_value(rows, "scenario")
    if scenario == "trajectory":
        return generate_trajectory_report(csv_path, rows)
    return generate_error_report(csv_path, rows)


def generate_error_report(csv_path: Path, rows) -> Path:
    t = column(rows, "time")
    out = csv_path.with_name(csv_path.stem + "_plots.png")
    fig, axes = plt.subplots(5, 1, figsize=(12, 16), sharex=True)
    for name in ALGORITHMS:
        axes[0].plot(t, column(rows, f"{name}_att_err"), label=name)
        axes[1].plot(t, column(rows, f"{name}_roll_err"), label=name)
        axes[2].plot(t, column(rows, f"{name}_pitch_err"), label=name)
        axes[3].plot(t, column(rows, f"{name}_yaw_err"), label=name)
        axes[4].plot(t, column(rows, f"{name}_pos_err"), label=name)
    axes[0].set_ylabel("att err [rad]")
    axes[1].set_ylabel("roll err [rad]")
    axes[2].set_ylabel("pitch err [rad]")
    axes[3].set_ylabel("yaw err [rad]")
    axes[4].set_ylabel("pos err [m]")
    axes[4].set_xlabel("time [s]")
    for ax in axes:
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", ncol=3, fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return out


def generate_trajectory_report(csv_path: Path, rows) -> Path:
    t = column(rows, "time")
    out = csv_path.with_name(csv_path.stem + "_plots.png")
    fig = plt.figure(figsize=(14, 11))
    ax_att = fig.add_subplot(2, 2, 1)
    ax_pos = fig.add_subplot(2, 2, 2)
    ax_3d = fig.add_subplot(2, 1, 2, projection="3d")

    for name in ALGORITHMS:
        ax_att.plot(t, column(rows, f"{name}_att_err"), label=name)
        ax_pos.plot(t, column(rows, f"{name}_pos_err"), label=name)

    gt_x = column(rows, "gt_x")
    gt_y = column(rows, "gt_y")
    gt_z = column(rows, "gt_z")
    ax_3d.plot(gt_x, gt_y, gt_z, "k--", linewidth=2.0, label="ground_truth")
    for name in ESTIMATED_ALGORITHMS:
        ax_3d.plot(
            column(rows, f"{name}_x"),
            column(rows, f"{name}_y"),
            column(rows, f"{name}_z"),
            label=name,
        )

    ax_att.set_ylabel("att err [rad]")
    ax_pos.set_ylabel("pos err [m]")
    ax_pos.set_xlabel("time [s]")
    ax_3d.set_xlabel("x [m]")
    ax_3d.set_ylabel("y [m]")
    ax_3d.set_zlabel("z [m]")
    ax_3d.set_title("3D trajectory")
    set_axes_equal_3d(ax_3d)
    for ax in [ax_att, ax_pos, ax_3d]:
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return out


def set_axes_equal_3d(ax) -> None:
    limits = np.asarray([ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d()], dtype=float)
    centers = np.mean(limits, axis=1)
    radius = 0.5 * np.max(limits[:, 1] - limits[:, 0])
    if not np.isfinite(radius) or radius <= 0.0:
        radius = 1.0
    ax.set_xlim3d([centers[0] - radius, centers[0] + radius])
    ax.set_ylim3d([centers[1] - radius, centers[1] + radius])
    ax.set_zlim3d([centers[2] - radius, centers[2] + radius])


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path", type=Path)
    parsed = parser.parse_args(args=args)
    out = generate_report(parsed.csv_path)
    print(out)


if __name__ == "__main__":
    main()
