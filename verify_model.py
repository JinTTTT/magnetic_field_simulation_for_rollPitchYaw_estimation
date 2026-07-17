#!/usr/bin/env python3
"""Verify the final model against the recorded verification session."""

import argparse
import csv
from datetime import datetime
import json
from pathlib import Path

import numpy as np

from magnetic_pose.config import (
    MODEL_PATH, ROOT, VERIFICATION_PATH, VERIFICATION_REPORT_PATH,
)
from magnetic_pose.estimator import PoseEstimator
from magnetic_pose.imu import wrap180
from magnetic_pose.model import CHANNELS


YAW_STATIONS = np.arange(-55.0, 56.0, 10.0)


def hat_basis(times, knots):
    basis = np.zeros((len(times), len(knots)))
    for row, value in enumerate(times):
        column = np.clip(np.searchsorted(knots, value) - 1, 0, len(knots) - 2)
        fraction = (value - knots[column]) / (knots[column + 1] - knots[column])
        basis[row, column] = 1.0 - fraction
        basis[row, column + 1] = fraction
    return basis


def error_metrics(errors):
    absolute = np.abs(errors)
    return {
        "mae": float(np.mean(absolute)),
        "rmse": float(np.sqrt(np.mean(errors ** 2))),
        "p95": float(np.percentile(absolute, 95)),
        "max": float(np.max(absolute)),
    }


def display_path(path):
    path = Path(path).resolve()
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def verify(data_path, model_path, knot_spacing, global_starts):
    with Path(data_path).open(newline="") as source:
        rows = sorted(csv.DictReader(source), key=lambda row: row["recorded_at_utc"])
    if not rows:
        raise ValueError(f"{data_path} contains no verification rows")

    start = datetime.fromisoformat(rows[0]["recorded_at_utc"])
    times = np.array([
        (datetime.fromisoformat(row["recorded_at_utc"]) - start).total_seconds()
        for row in rows
    ])
    labels = np.array([
        [float(row[name]) for name in ("yaw_deg", "pitch_deg", "roll_deg")]
        for row in rows
    ])
    fields = np.array([
        [float(row[f"{name}_corrected_mT"]) for name in CHANNELS]
        for row in rows
    ])

    estimator = PoseEstimator(model_path)
    estimator.widen_yaw_bounds(10.0)
    estimates = np.array([
        estimator.estimate(field, global_starts=global_starts)["angles_deg"]
        for field in fields
    ])

    # Separate smooth Xsens heading drift from repeatable station error.
    station_indexes = np.array([
        int(np.argmin(np.abs(YAW_STATIONS - yaw))) for yaw in labels[:, 0]
    ])
    knot_count = max(2, int(np.ceil(times[-1] / knot_spacing)) + 1)
    knots = np.linspace(0.0, times[-1], knot_count)
    time_basis = hat_basis(times, knots)
    station_basis = np.zeros((len(rows), len(YAW_STATIONS)))
    station_basis[np.arange(len(rows)), station_indexes] = 1.0
    constraint = np.zeros((1, knot_count + len(YAW_STATIONS)))
    constraint[0, knot_count:] = 1000.0
    design = np.vstack([np.hstack([time_basis, station_basis]), constraint])
    target = np.append(wrap180(estimates[:, 0] - labels[:, 0]), 0.0)
    coefficients, *_ = np.linalg.lstsq(design, target, rcond=None)
    heading_error = time_basis @ coefficients[:knot_count]

    truth = labels.copy()
    truth[:, 0] += heading_error
    errors = estimates - truth
    errors[:, 0] = wrap180(errors[:, 0])

    return {
        "schema_version": 1,
        "model": display_path(model_path),
        "data": display_path(data_path),
        "poses": len(rows),
        "note": (
            "Xsens yaw drift is estimated against the magnetic model; yaw metrics "
            "measure repeatability, not an independent absolute-yaw holdout."
        ),
        "error_deg": {
            name: error_metrics(errors[:, index])
            for index, name in enumerate(("yaw", "pitch", "roll"))
        },
        "xsens_heading_drift_deg": {
            "swing": float(np.ptp(heading_error)),
            "knot_times_s": knots.tolist(),
            "values": coefficients[:knot_count].tolist(),
        },
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=VERIFICATION_PATH)
    parser.add_argument("--model", type=Path, default=MODEL_PATH)
    parser.add_argument("--output", type=Path, default=VERIFICATION_REPORT_PATH)
    parser.add_argument("--knot-spacing", type=float, default=90.0)
    parser.add_argument("--global-starts", type=int, default=3)
    args = parser.parse_args()
    if args.knot_spacing <= 0 or args.global_starts < 1:
        parser.error("spacing and global starts must be positive")

    report = verify(args.data, args.model, args.knot_spacing, args.global_starts)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w") as output:
        json.dump(report, output, indent=2)
        output.write("\n")

    print(f"verified {report['poses']} poses")
    for axis, values in report["error_deg"].items():
        print(f"{axis:5s}: MAE {values['mae']:.3f}°   max {values['max']:.3f}°")
    print(f"report: {args.output}")


if __name__ == "__main__":
    main()
