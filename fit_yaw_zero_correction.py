#!/usr/bin/env python3
"""Fit the constant yaw offset between the accepted model and the mechanical dial.

The physical model reports yaw in the frame of its calibration labels. Those
labels came from the IMU heading, which had shifted relative to the mechanical
dial by the time the calibration set was recorded. This script measures that
constant rotation from poses set at known dial angles and stores it so the
estimator can report yaw in the mechanical frame.
"""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

from physical_model import CHANNELS, load_model, predict_mT


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def fit_pose_offset(model, dial_yaw_deg, imu_pitch_deg, imu_roll_deg,
                    measured_mT, tilt_bounds_deg):
    def residual(parameters):
        offset, pitch, roll = parameters
        pose = (dial_yaw_deg + offset, pitch, roll)
        return predict_mT([pose], model)[0] - measured_mT

    result = least_squares(
        residual,
        (0.0, imu_pitch_deg, imu_roll_deg),
        bounds=((-20.0, -tilt_bounds_deg, -tilt_bounds_deg),
                (20.0, tilt_bounds_deg, tilt_bounds_deg)),
        xtol=1e-9, ftol=1e-9, gtol=1e-9,
    )
    return result.x[0], float(np.sqrt(np.mean(result.fun ** 2)))


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=Path("yaw_bias_diagnostic.csv"),
                        help="poses recorded at known mechanical dial angles")
    parser.add_argument("--dial-yaws", default="0,-60,-30,0,30,60",
                        help="comma-separated dial yaw per row of --data")
    parser.add_argument("--model", type=Path, default=Path("physical_model.json"))
    parser.add_argument("--geometry", type=Path, default=Path("geometry_priors.json"))
    parser.add_argument("--output", type=Path, default=Path("yaw_zero_correction.json"))
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.output.exists() and not args.force:
        raise SystemExit(f"refusing to overwrite {args.output}; pass --force")

    dial_yaws = [float(value) for value in args.dial_yaws.split(",")]
    with args.data.open(newline="") as source:
        rows = list(csv.DictReader(source))
    if len(rows) != len(dial_yaws):
        raise SystemExit(
            f"{args.data} has {len(rows)} rows but --dial-yaws lists {len(dial_yaws)}"
        )

    model = load_model(args.model)
    with args.geometry.open() as source:
        workspace = json.load(source)["workspace_deg"]
    tilt_bound = max(abs(bound) for axis in ("pitch", "roll")
                     for bound in workspace[axis])

    offsets, field_rms = [], []
    for row, dial_yaw in zip(rows, dial_yaws):
        measured = np.asarray(
            [float(row[f"{name}_corrected_mT"]) for name in CHANNELS]
        )
        offset, rms = fit_pose_offset(
            model, dial_yaw, float(row["pitch_deg"]), float(row["roll_deg"]),
            measured, tilt_bound,
        )
        offsets.append(offset)
        field_rms.append(rms)
        print(f"dial yaw {dial_yaw:+6.1f}: model-frame offset {offset:+.3f} deg "
              f"(field rms {rms:.4f} mT)")

    offsets = np.asarray(offsets)
    payload = {
        "schema_version": 1,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "correction_type": "constant_model_frame_to_dial_frame_yaw_offset",
        "yaw_rule": "dial_yaw = model_yaw - yaw_zero_offset_deg",
        "assumption": (
            "The model's calibration labels are rotated from the mechanical "
            "dial frame by one constant yaw offset; the dial angles below are "
            "treated as truth."
        ),
        "inputs": {
            "data_path": str(args.data),
            "data_sha256": sha256_file(args.data),
            "model_path": str(args.model),
            "model_sha256": sha256_file(args.model),
            "dial_yaws_deg": dial_yaws,
        },
        "per_pose_offset_deg": [float(value) for value in offsets],
        "per_pose_field_rms_mT": field_rms,
        "yaw_zero_offset_deg": float(np.mean(offsets)),
        "offset_stddev_deg": float(np.std(offsets, ddof=1)),
    }
    with args.output.open("w") as output:
        json.dump(payload, output, indent=2)
        output.write("\n")
    print(f"\nyaw zero offset: {payload['yaw_zero_offset_deg']:+.3f} deg "
          f"(stddev {payload['offset_stddev_deg']:.3f} deg over {len(offsets)} poses)")
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
