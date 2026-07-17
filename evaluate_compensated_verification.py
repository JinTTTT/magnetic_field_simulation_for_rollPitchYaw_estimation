#!/usr/bin/env python3
"""Evaluate the yaw-corrected model against the heading-compensated labels.

This is NOT an untouched holdout evaluation: the compensated yaw labels were
built by removing a model-derived smooth-in-time heading error
(see compensate_verification_yaw.py). Yaw numbers therefore measure the
model's pose-dependent and random error only; absolute yaw accuracy must come
from dial-labeled data.
"""

import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path

import numpy as np

from evaluate_physical_model import metrics, sha256_file
from physical_estimator import PhysicalModelEstimator, inclusive_range
from physical_model import CHANNELS


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path,
                        default=Path("verification_data_yaw_compensated.csv"))
    parser.add_argument("--model", type=Path, default=Path("physical_model.json"))
    parser.add_argument("--correction", type=Path,
                        default=Path("yaw_zero_correction.json"))
    parser.add_argument("--output", type=Path,
                        default=Path("physical_model_compensated_verification_report.json"))
    parser.add_argument("--predictions-output", type=Path,
                        default=Path("physical_model_compensated_verification_predictions.csv"))
    parser.add_argument("--yaw-margin-deg", type=float, default=10.0)
    parser.add_argument("--global-starts", type=int, default=3)
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    existing = [path for path in (args.output, args.predictions_output)
                if path.exists()]
    if existing and not args.force:
        raise SystemExit(
            "refusing to overwrite output(s): " + ", ".join(map(str, existing))
            + "; pass --force"
        )

    with args.data.open(newline="") as source:
        rows = list(csv.DictReader(source))
    truth = np.asarray([
        [float(row["yaw_compensated_deg"]), float(row["pitch_deg"]),
         float(row["roll_deg"])]
        for row in rows
    ])
    measured = np.asarray([
        [float(row[f"{name}_corrected_mT"]) for name in CHANNELS]
        for row in rows
    ])

    estimator = PhysicalModelEstimator(
        model_path=args.model, correction_path=args.correction
    )
    # compensated truth extends past the ±60 workspace because the wandering
    # IMU physically placed the "±55" stations there; widen so nothing clips
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

    estimates, model_rms, evaluations = [], [], []
    for measurement in measured:
        result = estimator.estimate(
            measurement, seed=None, global_starts=args.global_starts
        )
        estimates.append(result["angles_deg"])
        model_rms.append(result["model_rms_mT"])
        evaluations.append(result["function_evaluations"])
    estimates = np.asarray(estimates)
    model_rms = np.asarray(model_rms)
    errors = estimates - truth
    errors[:, 0] = (errors[:, 0] + 180.0) % 360.0 - 180.0
    absolute = np.abs(errors)
    worst_axis = np.max(absolute, axis=1)
    euclidean = np.linalg.norm(errors, axis=1)

    report = {
        "schema_version": 1,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "report_type": "yaw_corrected_model_vs_heading_compensated_labels",
        "not_an_untouched_holdout": (
            "yaw labels were compensated with a model-derived smooth-in-time "
            "heading error; yaw metrics cover pose-dependent and random error "
            "only, not absolute frame accuracy"
        ),
        "residual_correction_active": False,
        "inputs": {
            "data_path": str(args.data),
            "data_sha256": sha256_file(args.data),
            "rows": len(rows),
            "model_path": str(args.model),
            "model_sha256": sha256_file(args.model),
            "yaw_correction_path": str(args.correction),
            "yaw_correction_sha256": sha256_file(args.correction),
            "yaw_zero_offset_deg": estimator.yaw_zero_offset_deg,
        },
        "method": {
            "truth_used_as_estimator_seed": False,
            "recording_order_used_for_tracking": False,
            "yaw_search_margin_deg": args.yaw_margin_deg,
            "coarse_grid_points": len(estimator.grid_poses),
            "global_starts_per_pose": args.global_starts,
            "local_optimizer": "bounded scipy.optimize.least_squares",
            "median_function_evaluations": float(np.median(evaluations)),
        },
        "angle_error_deg": {
            name: metrics(errors[:, index])
            for index, name in enumerate(("yaw", "pitch", "roll"))
        },
        "worst_axis_error_deg": metrics(worst_axis, already_absolute=True),
        "euclidean_angle_error_deg": metrics(euclidean, already_absolute=True),
        "optimized_field_rms_mT": metrics(model_rms, already_absolute=True),
    }

    with args.predictions_output.open("w", newline="") as output:
        writer = csv.writer(output)
        writer.writerow((
            "pose_id", "session_id",
            "yaw_truth_deg", "pitch_truth_deg", "roll_truth_deg",
            "yaw_estimate_deg", "pitch_estimate_deg", "roll_estimate_deg",
            "yaw_error_deg", "pitch_error_deg", "roll_error_deg",
            "worst_axis_error_deg", "model_rms_mT",
        ))
        for row, target, estimate, error, worst, rms in zip(
                rows, truth, estimates, errors, worst_axis, model_rms):
            writer.writerow((
                row["pose_id"], row["session_id"],
                *(f"{value:.9f}" for value in target),
                *(f"{value:.9f}" for value in estimate),
                *(f"{value:.9f}" for value in error),
                f"{worst:.9f}", f"{rms:.9f}",
            ))
    with args.output.open("w") as output:
        json.dump(report, output, indent=2)
        output.write("\n")

    print("Yaw-corrected model vs heading-compensated labels")
    print(f"poses: {len(rows)}")
    for name in ("yaw", "pitch", "roll"):
        item = report["angle_error_deg"][name]
        print(
            f"{name:5s}: MAE {item['mae']:.3f}  median {item['median_absolute']:.3f}  "
            f"p95 {item['p95_absolute']:.3f}  max {item['maximum_absolute']:.3f} deg"
        )
    item = report["worst_axis_error_deg"]
    print(
        f"worst: median {item['median_absolute']:.3f}  "
        f"p95 {item['p95_absolute']:.3f}  max {item['maximum_absolute']:.3f} deg"
    )
    print(f"wrote report:      {args.output}")
    print(f"wrote predictions: {args.predictions_output}")


if __name__ == "__main__":
    main()
