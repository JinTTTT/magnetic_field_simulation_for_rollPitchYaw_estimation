#!/usr/bin/env python3
"""Build the precomputed magnetic-field pose lookup table."""

import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from magnetic_pose.config import LOOKUP_PATH, MODEL_PATH
from magnetic_pose.lookup import (
    DEFAULT_TILT_STEP_DEG, DEFAULT_YAW_STEP_DEG, build_lookup_table,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=MODEL_PATH)
    parser.add_argument("--output", type=Path, default=LOOKUP_PATH)
    parser.add_argument("--yaw-step", type=float, default=DEFAULT_YAW_STEP_DEG)
    parser.add_argument("--tilt-step", type=float, default=DEFAULT_TILT_STEP_DEG)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    if min(args.yaw_step, args.tilt_step) <= 0:
        parser.error("grid steps must be positive")
    if args.output.exists() and not args.force:
        parser.error(f"{args.output} exists; pass --force to replace it")

    count = build_lookup_table(
        args.output, args.model, args.yaw_step, args.tilt_step
    )
    print(
        f"wrote {count} poses at yaw {args.yaw_step:g} deg, "
        f"pitch/roll {args.tilt_step:g} deg to {args.output}"
    )


if __name__ == "__main__":
    main()
