#!/usr/bin/env python3
"""Express heading-compensated calibration yaw labels in the dial frame.

This is a coordinate-frame conversion, not another heading-wander correction.
It preserves the source yaw in ``yaw_model_frame_deg`` and subtracts the
constant model-frame-to-dial-frame offset from ``yaw_deg``. The resulting CSV
can be frozen in a manifest and used to refit a model whose native yaw origin
is mechanical dial zero.
"""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def wrap180(angle):
    return (angle + 180.0) % 360.0 - 180.0


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input", type=Path,
        default=Path("calibration_data_heading_compensated_iter3.csv"),
    )
    parser.add_argument(
        "--correction", type=Path,
        default=Path("yaw_zero_correction_model_frame_backup.json"),
    )
    parser.add_argument(
        "--model", type=Path,
        default=Path("physical_model_model_frame_backup.json"),
    )
    parser.add_argument(
        "--output", type=Path, default=Path("calibration_data_dial_aligned.csv")
    )
    parser.add_argument(
        "--report-output", type=Path,
        default=Path("calibration_dial_alignment.json"),
    )
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    existing = [path for path in (args.output, args.report_output) if path.exists()]
    if existing and not args.force:
        raise SystemExit(
            "refusing to overwrite output(s): " + ", ".join(map(str, existing))
            + "; pass --force"
        )

    with args.correction.open() as source:
        correction = json.load(source)
    if correction.get("schema_version") != 1:
        raise ValueError(f"unsupported correction schema in {args.correction}")
    if correction.get("correction_type") != (
            "constant_model_frame_to_dial_frame_yaw_offset"):
        raise ValueError(f"{args.correction} is not a model-to-dial correction")
    model_hash = sha256_file(args.model)
    expected_model_hash = correction.get("inputs", {}).get("model_sha256")
    if model_hash != expected_model_hash:
        raise ValueError(
            f"{args.correction} was fitted for a different physical model"
        )
    offset_deg = float(correction["yaw_zero_offset_deg"])

    with args.input.open(newline="") as source:
        reader = csv.DictReader(source)
        rows = list(reader)
        fieldnames = list(reader.fieldnames or ())
    if not rows or "yaw_deg" not in fieldnames:
        raise ValueError(f"{args.input} has no calibration yaw rows")
    added = ("yaw_model_frame_deg", "dial_frame_offset_applied_deg")
    if any(name in fieldnames for name in added):
        raise ValueError(f"{args.input} is already dial-frame derived")

    with args.output.open("w", newline="") as output:
        writer = csv.DictWriter(
            output, fieldnames=fieldnames + list(added), lineterminator="\n"
        )
        writer.writeheader()
        for source_row in rows:
            row = dict(source_row)
            model_yaw = float(row["yaw_deg"])
            row["yaw_model_frame_deg"] = f"{model_yaw:.6f}"
            row["dial_frame_offset_applied_deg"] = f"{offset_deg:.12f}"
            row["yaw_deg"] = f"{wrap180(model_yaw - offset_deg):.6f}"
            writer.writerow(row)

    report = {
        "schema_version": 1,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "derivation_type": "constant_model_frame_to_dial_frame_yaw_conversion",
        "yaw_rule": "yaw_deg = wrap180(yaw_model_frame_deg - offset_deg)",
        "offset_deg": offset_deg,
        "rows": len(rows),
        "inputs": {
            "source_path": str(args.input),
            "source_sha256": sha256_file(args.input),
            "correction_path": str(args.correction),
            "correction_sha256": sha256_file(args.correction),
            "model_path": str(args.model),
            "model_sha256": model_hash,
        },
        "output": {
            "path": str(args.output),
            "sha256": sha256_file(args.output),
        },
    }
    with args.report_output.open("w") as output:
        json.dump(report, output, indent=2)
        output.write("\n")

    print(f"converted {len(rows)} yaw labels to the mechanical dial frame")
    print(f"subtracted offset: {offset_deg:+.12f} deg")
    print(f"wrote calibration: {args.output}")
    print(f"wrote provenance:  {args.report_output}")


if __name__ == "__main__":
    main()
