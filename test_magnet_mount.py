#!/usr/bin/env python3
"""Test magnet-mount repeatability from home-pose magnetic readings."""

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import time

from tools.tlv493d_coherent import (
    READER_TYPE,
    open_sensor_pair,
    prime_sensor_pair,
    read_pair_mT,
)


CHANNELS = ("S1_Bx", "S1_By", "S1_Bz", "S2_Bx", "S2_By", "S2_Bz")


def load_offsets(path):
    with path.open() as source:
        data = json.load(source)
    if data.get("sensor_reader", {}).get("type") != READER_TYPE:
        raise ValueError(
            f"{path} predates the coherent TLV493D reader; "
            "repeat measure_sensor_offsets.py first"
        )
    try:
        offsets = data["offsets_mT"]["S1"] + data["offsets_mT"]["S2"]
    except (KeyError, TypeError) as error:
        raise ValueError(f"{path} is not a static sensor-offset calibration") from error
    if len(offsets) != 6 or not all(math.isfinite(value) for value in offsets):
        raise ValueError(f"{path} must contain six finite offsets in mT")
    return [float(value) for value in offsets], data


def acquire_trial(sensors, trial_index, samples, sample_delay, offsets, started):
    rows = []
    for sample_index in range(samples):
        raw = read_pair_mT(sensors)
        corrected = [value - offset for value, offset in zip(raw, offsets)]
        rows.append((trial_index, sample_index, time.monotonic() - started,
                     *raw, *corrected))
        if sample_delay:
            time.sleep(sample_delay)
    return rows


def summarize(rows, trials):
    trial_means = []
    trial_stddevs = []
    for trial_index in range(trials):
        selected = [row for row in rows if row[0] == trial_index]
        corrected_columns = [
            [row[9 + channel] for row in selected]
            for channel in range(6)
        ]
        trial_means.append([statistics.fmean(values) for values in corrected_columns])
        trial_stddevs.append([
            statistics.stdev(values) if len(values) > 1 else 0.0
            for values in corrected_columns
        ])

    channel_spreads = [
        max(values) - min(values)
        for values in zip(*trial_means)
    ]
    return trial_means, trial_stddevs, channel_spreads


def write_raw_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    raw_names = tuple(f"{name}_raw_mT" for name in CHANNELS)
    corrected_names = tuple(f"{name}_corrected_mT" for name in CHANNELS)
    with path.open("w", newline="") as output:
        writer = csv.writer(output)
        writer.writerow(("trial", "sample", "elapsed_s", *raw_names, *corrected_names))
        for row in rows:
            writer.writerow((row[0], row[1], f"{row[2]:.6f}",
                             *(f"{value:.6f}" for value in row[3:])))


def split_sensors(values):
    return {"S1": values[:3], "S2": values[3:]}


def write_result_json(path, args, offsets_data, trial_means, trial_stddevs,
                      channel_spreads):
    path.parent.mkdir(parents=True, exist_ok=True)
    evaluated = args.trials >= 2
    passed = evaluated and max(channel_spreads) <= args.max_spread_mT
    result = {
        "schema_version": 1,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "test_type": "magnet_mount_repeatability_at_mechanical_home",
        "mechanical_pose_deg": {"yaw": 0.0, "pitch": 0.0, "roll": 0.0},
        "field_unit": "mT",
        "channel_order": list(CHANNELS),
        "sensor_offsets_created_at_utc": offsets_data.get("created_at_utc"),
        "acquisition": {
            "sensor_1_i2c_bus": args.bus1,
            "sensor_2_i2c_bus": args.bus2,
            "trials": args.trials,
            "samples_per_trial": args.samples,
            "sample_delay_s": args.sample_delay,
            "settle_time_per_trial_s": args.settle_seconds,
            "raw_samples_file": str(args.raw_output),
        },
        "trial_means_corrected_mT": [split_sensors(values) for values in trial_means],
        "trial_sample_stddev_mT": [split_sensors(values) for values in trial_stddevs],
        "channel_spread_mT": split_sensors(channel_spreads),
        "maximum_channel_spread_mT": max(channel_spreads),
        "acceptance": {
            "maximum_allowed_channel_spread_mT": args.max_spread_mT,
            "evaluated": evaluated,
            "passed": passed if evaluated else None,
        },
    }
    with path.open("w") as output:
        json.dump(result, output, indent=2)
        output.write("\n")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--offsets", type=Path, default=Path("sensor_offsets.json"))
    parser.add_argument("--output", type=Path, default=Path("magnet_mount_test.json"))
    parser.add_argument("--raw-output", type=Path,
                        default=Path("magnet_mount_samples.csv"))
    parser.add_argument("--bus1", type=int, default=3)
    parser.add_argument("--bus2", type=int, default=4)
    parser.add_argument("--trials", type=int, default=5,
                        help="separate magnet installations (default: 5)")
    parser.add_argument("--samples", type=int, default=128,
                        help="samples per sensor per trial (default: 128)")
    parser.add_argument("--sample-delay", type=float, default=0.03)
    parser.add_argument("--settle-seconds", type=float, default=2.0)
    parser.add_argument("--max-spread-mT", type=float, default=0.1,
                        help="maximum accepted spread of trial means (default: 0.1 mT)")
    parser.add_argument("--yes", action="store_true",
                        help="record trials sequentially without confirmation prompts")
    parser.add_argument("--force", action="store_true",
                        help="replace existing output files")
    args = parser.parse_args()

    if args.trials < 1 or args.samples < 1:
        parser.error("--trials and --samples must be at least 1")
    if min(args.sample_delay, args.settle_seconds, args.max_spread_mT) < 0:
        parser.error("delays and maximum spread cannot be negative")
    if args.output.resolve() == args.raw_output.resolve():
        parser.error("--output and --raw-output must be different files")
    if not args.offsets.exists():
        parser.error(f"offset calibration not found: {args.offsets}")
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
    offsets, offsets_data = load_offsets(args.offsets)
    _i2c_buses, sensors = open_sensor_pair(args.bus1, args.bus2)
    print("Magnet-mount repeatability test")
    print(f"target: every channel spread <= {args.max_spread_mT:g} mT")

    rows = []
    started = time.monotonic()
    for trial_index in range(args.trials):
        if not args.yes:
            response = input(
                f"Trial {trial_index + 1}/{args.trials}: install the magnet, "
                "place the rig at home, then press ENTER (q=quit): "
            ).strip().lower()
            if response == "q":
                print("cancelled; no output written")
                return
        print(f"settling for {args.settle_seconds:g} s...")
        time.sleep(args.settle_seconds)
        prime_sensor_pair(sensors)
        trial_rows = acquire_trial(
            sensors, trial_index, args.samples, args.sample_delay, offsets, started
        )
        rows.extend(trial_rows)
        means = [
            statistics.fmean(row[9 + channel] for row in trial_rows)
            for channel in range(6)
        ]
        print("corrected mean: " + " ".join(
            f"{name}={value:+.4f}" for name, value in zip(CHANNELS, means)
        ) + " mT")
        if trial_index + 1 < args.trials and not args.yes:
            print("Remove the magnet before preparing the next installation.")

    trial_means, trial_stddevs, channel_spreads = summarize(rows, args.trials)
    write_raw_csv(args.raw_output, rows)
    write_result_json(
        args.output, args, offsets_data, trial_means, trial_stddevs,
        channel_spreads
    )

    print("\nrepeatability summary")
    print(f"{'channel':>7} {'spread (mT)':>13} {'limit (mT)':>12}")
    for name, spread in zip(CHANNELS, channel_spreads):
        print(f"{name:>7} {spread:13.5f} {args.max_spread_mT:12.5f}")
    if args.trials < 2:
        print("result: not evaluated; at least two installations are required")
    elif max(channel_spreads) <= args.max_spread_mT:
        print("result: PASS")
    else:
        print("result: FAIL")
    print(f"wrote raw samples: {args.raw_output}")
    print(f"wrote report:      {args.output}")


if __name__ == "__main__":
    main()
