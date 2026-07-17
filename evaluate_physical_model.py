#!/usr/bin/env python3
"""Evaluate the model against a labeled pose dataset.

Default mode evaluates the manifest-locked verification file (consuming it as
development evidence). Passing --data evaluates any other labeled CSV instead,
for example the heading-compensated verification set with
--yaw-column yaw_compensated_deg; that mode is for diagnostics, not final
holdout evidence, and the report says so.

Estimates are reported in the dial frame whenever yaw_zero_correction.json is
present, so the truth labels must be in the dial frame too.
"""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import numpy as np

from physical_estimator import PhysicalModelEstimator, inclusive_range
from physical_model import CHANNELS


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def metrics(values, already_absolute=False):
    values = np.asarray(values, dtype=float)
    absolute = values if already_absolute else np.abs(values)
    return {
        "mae": float(np.mean(absolute)),
        "rmse": float(np.sqrt(np.mean(values ** 2))),
        "median_absolute": float(np.median(absolute)),
        "p95_absolute": float(np.percentile(absolute, 95)),
        "maximum_absolute": float(np.max(absolute)),
    }


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=Path("dataset_manifest.json"))
    parser.add_argument("--data", type=Path, default=None,
                        help="labeled CSV to evaluate instead of the "
                        "manifest-locked verification file")
    parser.add_argument("--yaw-column", default="yaw_deg",
                        help="truth yaw column, e.g. yaw_compensated_deg")
    parser.add_argument("--model", type=Path, default=Path("physical_model.json"))
    parser.add_argument("--correction", type=Path,
                        default=Path("yaw_zero_correction.json"))
    parser.add_argument("--output", type=Path,
                        default=Path("physical_model_verification_report.json"))
    parser.add_argument("--predictions-output", type=Path,
                        default=Path("physical_model_verification_predictions.csv"))
    parser.add_argument("--yaw-margin-deg", type=float, default=0.0,
                        help="widen the yaw search bounds, needed when truth "
                        "extends past the workspace edge")
    parser.add_argument("--global-starts", type=int, default=1)
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    existing = [path for path in (args.output, args.predictions_output) if path.exists()]
    if existing and not args.force:
        raise SystemExit(
            "refusing to overwrite output(s): " + ", ".join(map(str, existing))
            + "; pass --force"
        )
    if args.global_starts < 1:
        raise SystemExit("--global-starts must be at least 1")

    if args.data is None:
        with args.manifest.open() as source:
            manifest = json.load(source)
        entry = manifest["files"]["verification"]
        data_path = Path(entry["path"])
        data_hash = sha256_file(data_path)
        if data_hash != entry["sha256"]:
            raise ValueError("verification data no longer matches the locked manifest")
        report_type = "locked_verification_evaluation"
        status_note = (
            "consumed_development_evidence; a new untouched set is required "
            "for any later final claim"
        )
    else:
        data_path = args.data
        data_hash = sha256_file(data_path)
        report_type = "labeled_dataset_evaluation"
        status_note = (
            "diagnostic evaluation of a supplied dataset; not untouched "
            "holdout evidence"
        )
    with data_path.open(newline="") as source:
        rows = list(csv.DictReader(source))

    estimator = PhysicalModelEstimator(
        model_path=args.model, correction_path=args.correction
    )
    if args.yaw_margin_deg:
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

    truth = np.asarray([
        [float(row[name])
         for name in (args.yaw_column, "pitch_deg", "roll_deg")]
        for row in rows
    ])
    measured = np.asarray([
        [float(row[f"{name}_corrected_mT"]) for name in CHANNELS]
        for row in rows
    ])

    estimates = []
    model_rms = []
    evaluations = []
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
        "report_type": report_type,
        "status_note": status_note,
        "residual_correction_active": False,
        "inputs": {
            "data_path": str(data_path),
            "data_sha256": data_hash,
            "rows": len(rows),
            "yaw_truth_column": args.yaw_column,
            "model_path": str(args.model),
            "model_sha256": sha256_file(args.model),
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

    print(report_type)
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
