import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


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


def generate_report(csv_path: Path) -> Path:
    rows = read_csv(csv_path)
    if not rows:
        raise RuntimeError(f"No rows in {csv_path}")
    t = column(rows, "time")
    algorithms = ["raw_integrated", "ahrs", "eskf", "iekf", "fgo"]
    out = csv_path.with_name(csv_path.stem + "_plots.png")
    fig, axes = plt.subplots(5, 1, figsize=(12, 16), sharex=True)
    for name in algorithms:
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


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path", type=Path)
    parsed = parser.parse_args(args=args)
    out = generate_report(parsed.csv_path)
    print(out)
