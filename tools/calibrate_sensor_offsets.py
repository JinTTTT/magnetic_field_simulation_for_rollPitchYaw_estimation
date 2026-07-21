#!/usr/bin/env python3
"""Recalibrate magnetic sensor offsets with the main magnet removed."""

import argparse
import json
from pathlib import Path
import statistics
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from magnetic_pose.config import OFFSETS_PATH
from magnetic_pose.model import CHANNELS
from magnetic_pose.tlv493d import READER_TYPE, open_sensor_pair, read_pair_mT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OFFSETS_PATH)
    parser.add_argument("--bus1", type=int, default=3)
    parser.add_argument("--bus2", type=int, default=4)
    parser.add_argument("--samples", type=int, default=384)
    parser.add_argument("--sample-delay", type=float, default=0.03)
    parser.add_argument("--yes", action="store_true")
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    if args.samples < 2 or args.sample_delay < 0:
        parser.error("use at least two samples and a non-negative delay")
    if args.output.exists() and not args.force:
        parser.error(f"{args.output} exists; pass --force to replace it")

    print("Required: remove the main magnet and hold the rig at home.")
    if not args.yes and input("Press ENTER to continue, or q to quit: ").strip().lower() == "q":
        return

    _buses, sensors = open_sensor_pair(args.bus1, args.bus2)
    rows = []
    for index in range(args.samples):
        rows.append(read_pair_mT(sensors))
        if args.sample_delay:
            time.sleep(args.sample_delay)
        if (index + 1) % 50 == 0:
            print(f"{index + 1}/{args.samples} samples", end="\r", flush=True)
    print()

    offsets = [statistics.fmean(column) for column in zip(*rows)]
    result = {
        "schema_version": 1,
        "reader_type": READER_TYPE,
        "channel_order": list(CHANNELS),
        "field_unit": "mT",
        "offsets_mT": {"S1": offsets[:3], "S2": offsets[3:]},
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w") as output:
        json.dump(result, output, indent=2)
        output.write("\n")
    print(" ".join(f"{name}={value:+.5f}" for name, value in zip(CHANNELS, offsets)))
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
