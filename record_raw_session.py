#!/usr/bin/env python3
"""Open the live comparison while recording raw Xsens and magnetic data."""

import argparse
from pathlib import Path

from live_estimation_vs_imu import run
from magnetic_pose.config import LOOKUP_PATH, MODEL_PATH, OFFSETS_PATH
from magnetic_pose.filtering import DEFAULT_EMA_ALPHA


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument(
        "--duration", type=float, default=0.0,
        help="recording seconds; 0 records until the figure closes",
    )
    parser.add_argument("--model", type=Path, default=MODEL_PATH)
    parser.add_argument("--offsets", type=Path, default=OFFSETS_PATH)
    parser.add_argument("--lookup-table", type=Path, default=LOOKUP_PATH)
    parser.add_argument("--imu-zero-seconds", type=float, default=1.0)
    parser.add_argument("--imu-zero-max-stddev-deg", type=float, default=0.25)
    parser.add_argument("--ema-alpha", type=float, default=DEFAULT_EMA_ALPHA)
    parser.add_argument("--sample-delay", type=float, default=0.03)
    parser.add_argument("--refresh-ms", type=int, default=100)
    parser.add_argument("--bus1", type=int, default=3)
    parser.add_argument("--bus2", type=int, default=4)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    if args.duration < 0 or args.sample_delay < 0:
        parser.error("duration and sample delay cannot be negative")
    if not 0.0 < args.ema_alpha <= 1.0:
        parser.error("--ema-alpha must be greater than zero and at most one")
    if args.imu_zero_seconds <= 0 or args.imu_zero_max_stddev_deg < 0:
        parser.error("invalid IMU zeroing settings")
    args.record_raw = args.output
    args.record_duration = args.duration
    return args


if __name__ == "__main__":
    run(parse_args())
