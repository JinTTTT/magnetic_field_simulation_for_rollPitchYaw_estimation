#!/usr/bin/env python3
"""Measure static six-channel magnetic offsets at mechanical home.

Remove the main magnet, keep it far from the rig, and place the mechanism at
the (0, 0, 0) home pose before running this script. The resulting six means are
subtracted from all later magnet-in readings.

The script preserves every raw sample in CSV and writes the offsets, per-sample
standard deviations, and batch-to-batch spreads to JSON.
"""

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


def acquire(sensors, batches, samples_per_batch, sample_delay, batch_delay):
    """Acquire raw rows as (batch, sample, elapsed_s, six fields)."""
    rows = []
    started = time.monotonic()
    for batch_index in range(batches):
        for sample_index in range(samples_per_batch):
            fields = read_pair_mT(sensors)
            elapsed = time.monotonic() - started
            rows.append((batch_index, sample_index, elapsed, *fields))
            if sample_delay:
                time.sleep(sample_delay)

        batch_rows = rows[-samples_per_batch:]
        batch_means = [
            statistics.fmean(row[3 + channel] for row in batch_rows)
            for channel in range(6)
        ]
        print(
            f"batch {batch_index + 1}/{batches}: "
            + " ".join(f"{name}={value:+.4f}" for name, value in zip(CHANNELS, batch_means))
            + " mT"
        )
        if batch_index + 1 < batches and batch_delay:
            time.sleep(batch_delay)
    return rows


def summarize(rows, batches):
    """Calculate global and batch stability statistics for the six channels."""
    columns = [[row[3 + channel] for row in rows] for channel in range(6)]
    means = [statistics.fmean(values) for values in columns]
    stddevs = [statistics.stdev(values) if len(values) > 1 else 0.0 for values in columns]

    batch_means = []
    for batch_index in range(batches):
        batch_rows = [row for row in rows if row[0] == batch_index]
        batch_means.append([
            statistics.fmean(row[3 + channel] for row in batch_rows)
            for channel in range(6)
        ])
    batch_spreads = [
        max(values) - min(values)
        for values in zip(*batch_means)
    ]
    return means, stddevs, batch_means, batch_spreads


def write_raw_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as output:
        writer = csv.writer(output)
        writer.writerow(("batch", "sample", "elapsed_s", *CHANNELS))
        for row in rows:
            writer.writerow((row[0], row[1], f"{row[2]:.6f}",
                             *(f"{value:.6f}" for value in row[3:])))


def write_result_json(path, args, means, stddevs, batch_means, batch_spreads):
    path.parent.mkdir(parents=True, exist_ok=True)
    result = {
        "schema_version": 1,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "calibration_type": "static_home_pose_magnet_out_offset",
        "sensor_reader": {
            "type": READER_TYPE,
            "coherence_rule": COHERENCE_RULE,
        },
        "assumption": "The six magnet-out channel values are static and are subtracted from every later reading.",
        "conditions": {
            "main_magnet": "removed_and_kept_far_from_rig",
            "mechanical_pose_deg": {"yaw": 0.0, "pitch": 0.0, "roll": 0.0}
        },
        "channel_order": list(CHANNELS),
        "field_unit": "mT",
        "acquisition": {
            "sensor_1_i2c_bus": args.bus1,
            "sensor_2_i2c_bus": args.bus2,
            "batches": args.batches,
            "samples_per_batch": args.samples,
            "total_samples_per_channel": args.batches * args.samples,
            "sample_delay_s": args.sample_delay,
            "settle_time_s": args.settle_seconds,
            "raw_samples_file": str(args.raw_output)
        },
        "offsets_mT": {
            "S1": means[:3],
            "S2": means[3:]
        },
        "sample_stddev_mT": {
            "S1": stddevs[:3],
            "S2": stddevs[3:]
        },
        "batch_means_mT": [
            {"S1": values[:3], "S2": values[3:]}
            for values in batch_means
        ],
        "batch_mean_spread_mT": {
            "S1": batch_spreads[:3],
            "S2": batch_spreads[3:]
        }
    }
    with path.open("w") as output:
        json.dump(result, output, indent=2)
        output.write("\n")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("sensor_offsets.json"))
    parser.add_argument("--raw-output", type=Path,
                        default=Path("sensor_offset_samples.csv"))
    parser.add_argument("--bus1", type=int, default=3)
    parser.add_argument("--bus2", type=int, default=4)
    parser.add_argument("--batches", type=int, default=3)
    parser.add_argument("--samples", type=int, default=128,
                        help="samples per sensor per batch (default: 128)")
    parser.add_argument("--sample-delay", type=float, default=0.03)
    parser.add_argument("--batch-delay", type=float, default=0.5)
    parser.add_argument("--settle-seconds", type=float, default=2.0)
    parser.add_argument("--yes", action="store_true",
                        help="skip the magnet-removed/home-pose confirmation")
    parser.add_argument("--force", action="store_true",
                        help="replace existing output files")
    args = parser.parse_args()

    if args.batches < 1 or args.samples < 1:
        parser.error("--batches and --samples must be at least 1")
    if min(args.sample_delay, args.batch_delay, args.settle_seconds) < 0:
        parser.error("delays and settle time cannot be negative")
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
    print("Static TLV493D offset measurement")
    print("Required condition: main magnet removed and rig held at mechanical home.")
    if not args.yes:
        response = input("Press ENTER to confirm and begin, or type q to quit: ").strip().lower()
        if response == "q":
            print("cancelled")
            return

    _i2c_buses, sensors = open_sensor_pair(args.bus1, args.bus2)
    print(f"sensors opened on I2C buses {args.bus1} and {args.bus2}")
    print(f"settling for {args.settle_seconds:g} s...")
    time.sleep(args.settle_seconds)
    prime_sensor_pair(sensors)

    rows = acquire(
        sensors,
        batches=args.batches,
        samples_per_batch=args.samples,
        sample_delay=args.sample_delay,
        batch_delay=args.batch_delay,
    )
    means, stddevs, batch_means, batch_spreads = summarize(rows, args.batches)
    write_raw_csv(args.raw_output, rows)
    write_result_json(
        args.output, args, means, stddevs, batch_means, batch_spreads
    )

    print("\nsummary")
    print(f"{'channel':>7} {'offset (mT)':>12} {'sample std':>12} {'batch spread':>14}")
    for name, mean, stddev, spread in zip(CHANNELS, means, stddevs, batch_spreads):
        print(f"{name:>7} {mean:12.5f} {stddev:12.5f} {spread:14.5f}")
    print(f"\nwrote raw samples: {args.raw_output}")
    print(f"wrote offsets:     {args.output}")


if __name__ == "__main__":
    main()
