import argparse
import os
from pathlib import Path

from .synthetic_benchmark import run_synthetic


def main(args=None):
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, default=Path("results"))
    parser.add_argument("--rate-hz", type=float, default=100.0)
    parser.add_argument("--short", action="store_true")
    parsed = parser.parse_args(args=args)
    cases = [
        ("fast_rotation", "yaw", 12.0 if parsed.short else 30.0),
        ("static_zero_drift", "circle", 12.0 if parsed.short else 60.0),
        ("static_dynamic", "circle", 45.0),
        ("trajectory", "circle", 30.0 if parsed.short else 60.0),
        ("trajectory", "figure8", 30.0 if parsed.short else 60.0),
        ("trajectory", "spiral", 30.0 if parsed.short else 60.0),
        ("loop_a", "circle", 35.0 if parsed.short else 55.0),
    ]
    for scenario, trajectory, duration in cases:
        run_id = f"{scenario}_{trajectory}_batch"
        run_synthetic(
            scenario,
            trajectory,
            duration,
            parsed.rate_hz,
            parsed.output_dir,
            run_id,
        )
