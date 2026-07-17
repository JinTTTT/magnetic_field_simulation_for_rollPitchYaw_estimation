#!/usr/bin/env python3
"""Compare coherent TLV493D stability at home and approximately +90 yaw."""

import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import statistics
import time

from tools.tlv493d_coherent import (
    COHERENCE_RULE,
    READER_TYPE,
    open_sensor_pair,
    prime_sensor_pair,
    read_pair_mT,
)


CHANNELS = ("S1_Bx", "S1_By", "S1_Bz", "S2_Bx", "S2_By", "S2_Bz")
POSES = (("home", "mechanical home"), ("yaw_plus_90", "approximately +90 deg yaw"))


def acquire_pose(sensors, pose_index, label, samples, sample_delay, started):
    rows = []
    for sample_index in range(samples):
        fields = read_pair_mT(sensors)
        rows.append({
            "pose_index": pose_index,
            "pose": label,
            "sample": sample_index,
            "elapsed_s": time.monotonic() - started,
            "fields": fields,
            "attempts": [sensor.last_attempts for sensor in sensors],
            "frame_counters": [sensor.last_frame_counter for sensor in sensors],
            "registers_hex": [sensor.last_registers.hex() for sensor in sensors],
        })
        if sample_delay:
            time.sleep(sample_delay)
    return rows


def summarize(rows, max_stddev_mT):
    summaries = []
    for label, _description in POSES:
        selected = [row for row in rows if row["pose"] == label]
        if not selected:
            continue
        columns = [
            [row["fields"][channel] for row in selected]
            for channel in range(6)
        ]
        means = [statistics.fmean(values) for values in columns]
        stddevs = [
            statistics.stdev(values) if len(values) > 1 else 0.0
            for values in columns
        ]
        minima = [min(values) for values in columns]
        maxima = [max(values) for values in columns]
        rejected = [
            sum(row["attempts"][sensor_index] - 1 for row in selected)
            for sensor_index in range(2)
        ]
        summaries.append({
            "pose": label,
            "means_mT": means,
            "stddev_mT": stddevs,
            "min_mT": minima,
            "max_mT": maxima,
            "rejected_incomplete_frames": rejected,
            "maximum_stddev_mT": max(stddevs),
            "passed": max(stddevs) <= max_stddev_mT,
        })
    return summaries


def write_raw_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as output:
        writer = csv.writer(output)
        writer.writerow((
            "pose_index", "pose", "sample", "elapsed_s", *CHANNELS,
            "S1_attempts", "S1_frame_counter", "S1_registers_hex",
            "S2_attempts", "S2_frame_counter", "S2_registers_hex",
        ))
        for row in rows:
            writer.writerow((
                row["pose_index"], row["pose"], row["sample"],
                f"{row['elapsed_s']:.6f}",
                *(f"{value:.6f}" for value in row["fields"]),
                row["attempts"][0], row["frame_counters"][0], row["registers_hex"][0],
                row["attempts"][1], row["frame_counters"][1], row["registers_hex"][1],
            ))


def split_sensors(values):
    return {"S1": values[:3], "S2": values[3:]}


def write_report(path, args, summaries):
    path.parent.mkdir(parents=True, exist_ok=True)
    report_summaries = []
    for summary in summaries:
        report_summaries.append({
            "pose": summary["pose"],
            "means_mT": split_sensors(summary["means_mT"]),
            "stddev_mT": split_sensors(summary["stddev_mT"]),
            "min_mT": split_sensors(summary["min_mT"]),
            "max_mT": split_sensors(summary["max_mT"]),
            "rejected_incomplete_frames": {
                "S1": summary["rejected_incomplete_frames"][0],
                "S2": summary["rejected_incomplete_frames"][1],
            },
            "maximum_stddev_mT": summary["maximum_stddev_mT"],
            "passed": summary["passed"],
        })
    report = {
        "schema_version": 1,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "test_type": "coherent_tlv493d_pose_stability",
        "channel_order": list(CHANNELS),
        "field_unit": "mT",
        "sensor_reader": {
            "type": READER_TYPE,
            "coherence_rule": COHERENCE_RULE,
        },
        "acquisition": {
            "sensor_1_i2c_bus": args.bus1,
            "sensor_2_i2c_bus": args.bus2,
            "samples_per_pose": args.samples,
            "sample_delay_s": args.sample_delay,
            "settle_time_per_pose_s": args.settle_seconds,
            "discarded_priming_reads_per_pose": 1,
            "raw_samples_file": str(args.raw_output),
        },
        "acceptance": {
            "maximum_allowed_channel_stddev_mT": args.max_stddev_mT,
            "passed": all(summary["passed"] for summary in summaries),
        },
        "poses": report_summaries,
    }
    with path.open("w") as output:
        json.dump(report, output, indent=2)
        output.write("\n")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path,
                        default=Path("sensor_stability_test.json"))
    parser.add_argument("--raw-output", type=Path,
                        default=Path("sensor_stability_samples.csv"))
    parser.add_argument("--bus1", type=int, default=3)
    parser.add_argument("--bus2", type=int, default=4)
    parser.add_argument("--samples", type=int, default=128)
    parser.add_argument("--sample-delay", type=float, default=0.03)
    parser.add_argument("--settle-seconds", type=float, default=2.0)
    parser.add_argument("--max-stddev-mT", type=float, default=0.2)
    parser.add_argument("--force", action="store_true",
                        help="replace existing output files")
    args = parser.parse_args()

    if args.samples < 2:
        parser.error("--samples must be at least 2")
    if min(args.sample_delay, args.settle_seconds, args.max_stddev_mT) < 0:
        parser.error("delays and maximum standard deviation cannot be negative")
    if args.output.resolve() == args.raw_output.resolve():
        parser.error("--output and --raw-output must be different files")
    existing = [path for path in (args.output, args.raw_output) if path.exists()]
    if existing and not args.force:
        parser.error(
            "refusing to overwrite existing file(s): "
            + ", ".join(str(path) for path in existing)
            + "; pass --force to replace them"
        )
    return args


def main():
    args = parse_args()
    _i2c_buses, sensors = open_sensor_pair(args.bus1, args.bus2)
    print("Coherent TLV493D stability test")
    print(f"target: every channel stddev <= {args.max_stddev_mT:g} mT")
    rows = []
    started = time.monotonic()
    for pose_index, (label, description) in enumerate(POSES):
        response = input(
            f"Place the stationary rig at {description}, then press ENTER (q=quit): "
        ).strip().lower()
        if response == "q":
            print("cancelled; no output written")
            return
        print(f"settling for {args.settle_seconds:g} s...")
        time.sleep(args.settle_seconds)
        prime_sensor_pair(sensors)
        pose_rows = acquire_pose(
            sensors, pose_index, label, args.samples, args.sample_delay, started
        )
        rows.extend(pose_rows)
        pose_summary = summarize(pose_rows, args.max_stddev_mT)[0]
        print("stddev: " + " ".join(
            f"{name}={value:.4f}" for name, value in zip(CHANNELS, pose_summary["stddev_mT"])
        ) + " mT")

    summaries = summarize(rows, args.max_stddev_mT)
    write_raw_csv(args.raw_output, rows)
    write_report(args.output, args, summaries)
    print()
    for summary in summaries:
        verdict = "PASS" if summary["passed"] else "FAIL"
        print(f"{summary['pose']}: max stddev {summary['maximum_stddev_mT']:.5f} mT -> {verdict}")
    overall = all(summary["passed"] for summary in summaries)
    print(f"overall: {'PASS' if overall else 'FAIL'}")
    print(f"wrote raw frames: {args.raw_output}")
    print(f"wrote report:     {args.output}")


if __name__ == "__main__":
    main()
