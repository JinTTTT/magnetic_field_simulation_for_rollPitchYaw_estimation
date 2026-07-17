#!/usr/bin/env python3
"""Create a checksum manifest that freezes fitting and holdout inputs."""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path


INPUTS = (
    ("calibration", "calibration_data.csv", "physical_fit_and_residual_training"),
    ("verification", "verification_data.csv", "untouched_final_evaluation_only"),
    ("geometry_priors", "geometry_priors.json", "physical_fit_priors"),
    ("sensor_offsets", "sensor_offsets.json", "measurement_preprocessing_provenance"),
    ("imu_yaw_reference", "imu_yaw_reference.json", "pose_reference_provenance"),
)


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def csv_metadata(path):
    with path.open(newline="") as source:
        reader = csv.DictReader(source)
        rows = list(reader)
        header = reader.fieldnames
    if not header:
        raise ValueError(f"{path} has no CSV header")
    sessions = sorted({row.get("session_id", "") for row in rows})
    return {
        "rows": len(rows),
        "columns": header,
        "session_ids": sessions,
    }


def describe(path, role, policy):
    if not path.exists():
        raise FileNotFoundError(path)
    result = {
        "role": role,
        "policy": policy,
        "path": str(path),
        "size_bytes": path.stat().st_size,
        "sha256": sha256_file(path),
    }
    if path.suffix.lower() == ".csv":
        result.update(csv_metadata(path))
    else:
        with path.open() as source:
            payload = json.load(source)
        result["schema_version"] = payload.get("schema_version")
        result["created_at_utc"] = payload.get("created_at_utc")
    return result


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("dataset_manifest.json"))
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.output.exists() and not args.force:
        raise SystemExit(f"refusing to overwrite {args.output}; pass --force")

    files = {}
    for role, filename, policy in INPUTS:
        files[role] = describe(Path(filename), role, policy)

    manifest = {
        "schema_version": 1,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "purpose": "Freeze clean-pipeline inputs before any physical-model fitting.",
        "separation_policy": {
            "calibration": "May be used for fitting and calibration-only diagnostics.",
            "verification": (
                "Must not be loaded by fitting, feature selection, residual training, "
                "regularization selection, or model acceptance before final evaluation."
            ),
        },
        "files": files,
    }
    with args.output.open("w") as output:
        json.dump(manifest, output, indent=2)
        output.write("\n")

    print(f"wrote locked input manifest: {args.output}")
    for role, item in files.items():
        suffix = f", {item['rows']} rows" if "rows" in item else ""
        print(f"{role:18s} {item['sha256'][:12]}...{suffix}")


if __name__ == "__main__":
    main()
