#!/usr/bin/env python3
"""Remove the IMU heading wander from calibration yaw labels using the model.

Same decomposition as compensate_verification_yaw.py: the discrepancy between
the magnetic estimate and the yaw label is split into a piecewise-linear
function of time e(t) (heading wander) and a zero-mean per-station term
(pose-dependent model error). Labels are compensated with e(t) only, and e(t)
is forced to zero mean so the session's average frame is preserved (the
constant frame offset stays in yaw_zero_correction.json).

Intended use is iterative: compensate against the current model, refit the
model on the compensated labels, and repeat until e(t) stops changing.
Always run against the ORIGINAL calibration file so each pass yields the
cumulative heading estimate.
"""

import argparse
import csv
from datetime import datetime
import json
from pathlib import Path

import numpy as np

from measure_imu_yaw_reference import wrap180
from physical_estimator import PhysicalModelEstimator
from physical_model import CHANNELS

STATIONS_DEG = np.arange(-60.0, 61.0, 10.0)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration", type=Path,
                        default=Path("calibration_data.csv"))
    parser.add_argument("--model", type=Path, default=Path("physical_model.json"))
    parser.add_argument("--correction", type=Path,
                        default=Path("yaw_zero_correction.json"),
                        help="ignored if the file does not exist (model frame)")
    parser.add_argument("--output", type=Path,
                        default=Path("calibration_data_heading_compensated.csv"))
    parser.add_argument("--summary-output", type=Path,
                        default=Path("calibration_heading_wander.json"))
    parser.add_argument("--knot-spacing-s", type=float, default=300.0)
    parser.add_argument("--yaw-margin-deg", type=float, default=8.0)
    parser.add_argument("--global-starts", type=int, default=2)
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    existing = [path for path in (args.output, args.summary_output) if path.exists()]
    if existing and not args.force:
        raise SystemExit(
            "refusing to overwrite output(s): " + ", ".join(map(str, existing))
            + "; pass --force"
        )

    with args.calibration.open(newline="") as source:
        rows = list(csv.DictReader(source))
    order = np.argsort([row["recorded_at_utc"] for row in rows])
    start = datetime.fromisoformat(rows[order[0]]["recorded_at_utc"])
    times = np.array([
        (datetime.fromisoformat(rows[i]["recorded_at_utc"]) - start).total_seconds()
        for i in order
    ])
    labels = np.array([
        [float(rows[i][name]) for name in ("yaw_deg", "pitch_deg", "roll_deg")]
        for i in order
    ])
    measured = np.array([
        [float(rows[i][f"{name}_corrected_mT"]) for name in CHANNELS]
        for i in order
    ])

    estimator = PhysicalModelEstimator(
        model_path=args.model, correction_path=args.correction
    )
    estimator.widen_yaw_bounds(args.yaw_margin_deg)
    print(f"estimating {len(rows)} poses against {args.model} "
          f"(frame offset {estimator.yaw_zero_offset_deg:+.3f} deg)...")

    discrepancy = np.empty(len(rows))
    for index, field in enumerate(measured):
        result = estimator.estimate(field, global_starts=args.global_starts)
        discrepancy[index] = wrap180(result["angles_deg"][0] - labels[index, 0])
    # work in the model's own frame so e(t) centers near zero
    discrepancy += estimator.yaw_zero_offset_deg

    stations = np.array([
        int(np.argmin(np.abs(STATIONS_DEG - yaw))) for yaw in labels[:, 0]
    ])
    knot_count = max(2, int(np.ceil(times[-1] / args.knot_spacing_s)) + 1)
    knots = np.linspace(0.0, times[-1], knot_count)
    time_basis = np.zeros((len(rows), len(knots)))
    for row_index, time in enumerate(times):
        j = min(np.searchsorted(knots, time, side="right") - 1, len(knots) - 2)
        fraction = (time - knots[j]) / (knots[j + 1] - knots[j])
        time_basis[row_index, j] = 1.0 - fraction
        time_basis[row_index, j + 1] = fraction
    station_basis = np.zeros((len(rows), len(STATIONS_DEG)))
    station_basis[np.arange(len(rows)), stations] = 1.0
    constraint = np.zeros((1, len(knots) + len(STATIONS_DEG)))
    constraint[0, len(knots):] = 1000.0
    design = np.vstack([np.hstack([time_basis, station_basis]), constraint])
    target = np.append(discrepancy, 0.0)
    coefficients, *_ = np.linalg.lstsq(design, target, rcond=None)

    heading = time_basis @ coefficients[:len(knots)]
    heading -= heading.mean()  # preserve the session-average frame
    station_term = coefficients[len(knots):]
    noise = discrepancy - discrepancy.mean() - heading - (
        station_term[stations] - station_term[stations].mean()
    )
    print(f"heading wander swing: {heading.max() - heading.min():.2f} deg "
          f"(std {heading.std():.2f})")
    print(f"station term range: {station_term.min():+.2f} .. "
          f"{station_term.max():+.2f} deg")
    print(f"unmodeled noise std: {np.std(noise):.3f} deg")

    fieldnames = list(rows[0].keys()) + [
        "yaw_imu_original_deg", "heading_error_estimate_deg",
    ]
    with args.output.open("w", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=fieldnames)
        writer.writeheader()
        for position, row_index in enumerate(order):
            row = dict(rows[row_index])
            original = labels[position, 0]
            row["yaw_imu_original_deg"] = f"{original:.6f}"
            row["heading_error_estimate_deg"] = f"{heading[position]:.6f}"
            row["yaw_deg"] = f"{original + heading[position]:.6f}"
            writer.writerow(row)

    summary = {
        "schema_version": 1,
        "created_at_utc": datetime.now().astimezone().isoformat(),
        "model_path": str(args.model),
        "calibration_path": str(args.calibration),
        "knot_spacing_s": args.knot_spacing_s,
        "knot_times_s": knots.tolist(),
        "heading_error_at_knots_deg": (
            coefficients[:len(knots)]
            - (time_basis @ coefficients[:len(knots)]).mean()
        ).tolist(),
        "heading_swing_deg": float(heading.max() - heading.min()),
        "station_term_deg": {
            f"{station:+.0f}": float(value)
            for station, value in zip(STATIONS_DEG, station_term)
        },
        "unmodeled_noise_std_deg": float(np.std(noise)),
    }
    with args.summary_output.open("w") as output:
        json.dump(summary, output, indent=2)
        output.write("\n")
    print(f"wrote {args.output}")
    print(f"wrote {args.summary_output}")


if __name__ == "__main__":
    main()
