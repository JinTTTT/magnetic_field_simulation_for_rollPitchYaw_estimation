#!/usr/bin/env python3
"""Estimate and remove the time-varying IMU heading error from verification yaw.

The verification session swept the same twelve yaw stations five times while
the IMU heading wandered. The per-row discrepancy between the corrected
magnetic estimate and the IMU yaw label is modeled as

    discrepancy(t, station) = e(t) + g(station) + noise

where e(t) is a piecewise-linear function of time (the heading error, smooth
over tens of seconds) and g(station) is a per-station term that repeats in
every sweep (pose-dependent model error, constrained to zero mean). The
compensated label is `yaw_imu + e(t)`.

The compensated set cannot certify absolute yaw accuracy (e(t) is measured
against the model itself); it is for diagnostics and residual-correction
development, not final holdout evidence.
"""

import argparse
import csv
from datetime import datetime
from pathlib import Path

import numpy as np

from measure_imu_yaw_reference import wrap180
from physical_estimator import PhysicalModelEstimator, inclusive_range
from physical_model import CHANNELS

STATIONS_DEG = np.arange(-55.0, 56.0, 10.0)


def hat_basis(times_s, knots_s):
    """Piecewise-linear interpolation basis: one hat function per knot."""
    basis = np.zeros((len(times_s), len(knots_s)))
    for column, time in enumerate(np.atleast_1d(times_s)):
        index = np.clip(np.searchsorted(knots_s, time) - 1, 0, len(knots_s) - 2)
        span = knots_s[index + 1] - knots_s[index]
        fraction = (time - knots_s[index]) / span
        basis[column, index] = 1.0 - fraction
        basis[column, index + 1] = fraction
    return basis


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=Path("verification_data.csv"))
    parser.add_argument("--model", type=Path, default=Path("physical_model.json"))
    parser.add_argument("--correction", type=Path,
                        default=Path("yaw_zero_correction.json"))
    parser.add_argument("--output", type=Path,
                        default=Path("verification_data_yaw_compensated.csv"))
    parser.add_argument("--knot-spacing-s", type=float, default=90.0)
    parser.add_argument("--yaw-margin-deg", type=float, default=10.0,
                        help="widen the yaw search bounds so poses whose "
                        "dial-frame yaw lies past the workspace edge (because "
                        "the wandering IMU placed them there) do not clip")
    parser.add_argument("--global-starts", type=int, default=3)
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.output.exists() and not args.force:
        raise SystemExit(f"refusing to overwrite {args.output}; pass --force")

    with args.data.open(newline="") as source:
        rows = list(csv.DictReader(source))
    rows.sort(key=lambda row: row["recorded_at_utc"])
    start = datetime.fromisoformat(rows[0]["recorded_at_utc"])
    times = np.array([
        (datetime.fromisoformat(row["recorded_at_utc"]) - start).total_seconds()
        for row in rows
    ])
    labels = np.array([
        [float(row[name]) for name in ("yaw_deg", "pitch_deg", "roll_deg")]
        for row in rows
    ])
    measured = np.array([
        [float(row[f"{name}_corrected_mT"]) for name in CHANNELS]
        for row in rows
    ])
    stations = np.array([
        int(np.argmin(np.abs(STATIONS_DEG - yaw))) for yaw in labels[:, 0]
    ])

    estimator = PhysicalModelEstimator(
        model_path=args.model, correction_path=args.correction
    )
    estimator.lower[0] -= args.yaw_margin_deg
    estimator.upper[0] += args.yaw_margin_deg
    yaw_values = inclusive_range(estimator.lower[0], estimator.upper[0], 10.0)
    estimator.grid_poses = np.asarray([
        (yaw, pitch, roll)
        for yaw in yaw_values
        for pitch in np.unique(estimator.grid_poses[:, 1])
        for roll in np.unique(estimator.grid_poses[:, 2])
    ])
    estimator.grid_fields_mT = estimator._predict(estimator.grid_poses)
    print(f"estimating {len(rows)} poses "
          f"(yaw zero correction {estimator.yaw_zero_offset_deg:+.3f} deg, "
          f"yaw search bounds ±{args.yaw_margin_deg:.0f} deg widened)...")
    estimates = np.array([
        estimator.estimate(field, global_starts=args.global_starts)["angles_deg"]
        for field in measured
    ])
    discrepancy = wrap180(estimates[:, 0] - labels[:, 0])

    knot_count = max(2, int(np.ceil(times[-1] / args.knot_spacing_s)) + 1)
    knots = np.linspace(0.0, times[-1], knot_count)
    time_basis = hat_basis(times, knots)
    station_basis = np.zeros((len(rows), len(STATIONS_DEG)))
    station_basis[np.arange(len(rows)), stations] = 1.0
    # zero-mean constraint on g so the shared constant belongs to e(t)
    constraint = np.zeros((1, len(knots) + len(STATIONS_DEG)))
    constraint[0, len(knots):] = 1000.0
    design = np.vstack([np.hstack([time_basis, station_basis]), constraint])
    target = np.append(discrepancy, 0.0)
    coefficients, *_ = np.linalg.lstsq(design, target, rcond=None)

    heading_error = time_basis @ coefficients[:len(knots)]
    station_error = coefficients[len(knots):]
    noise = discrepancy - heading_error - station_error[stations]
    compensated = labels[:, 0] + heading_error

    print(f"\nheading error e(t) at {args.knot_spacing_s:.0f} s knots [deg]:")
    for knot, value in zip(knots, coefficients[:len(knots)]):
        print(f"  t={knot:5.0f} s: {value:+6.2f}")
    print("\npose-dependent term g(station) [deg]:")
    for station, value in zip(STATIONS_DEG, station_error):
        count = int(np.sum(stations == np.argmin(np.abs(STATIONS_DEG - station))))
        print(f"  yaw {station:+5.0f}: {value:+6.2f}  (n={count})")
    print(f"\nunmodeled noise stddev: {np.std(noise):.3f} deg")

    residual = discrepancy - heading_error
    print("\nyaw error vs compensated labels (= g + noise):")
    print(f"  MAE {np.mean(np.abs(residual)):.3f}  "
          f"median {np.median(np.abs(residual)):.3f}  "
          f"max {np.max(np.abs(residual)):.3f} deg")
    for name, column in (("pitch", 1), ("roll", 2)):
        errors = estimates[:, column] - labels[:, column]
        print(f"  {name} vs IMU (unchanged): MAE {np.mean(np.abs(errors)):.3f}  "
              f"max {np.max(np.abs(errors)):.3f} deg")

    fieldnames = list(rows[0].keys()) + [
        "heading_error_estimate_deg", "yaw_compensated_deg",
        "yaw_magnetic_estimate_deg",
    ]
    with args.output.open("w", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=fieldnames)
        writer.writeheader()
        for row, error, label, estimate in zip(
                rows, heading_error, compensated, estimates[:, 0]):
            writer.writerow({
                **row,
                "heading_error_estimate_deg": f"{error:.6f}",
                "yaw_compensated_deg": f"{label:.6f}",
                "yaw_magnetic_estimate_deg": f"{estimate:.6f}",
            })
    print(f"\nwrote {args.output}")


if __name__ == "__main__":
    main()
