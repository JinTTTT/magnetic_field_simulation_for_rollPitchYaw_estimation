#!/usr/bin/env python3
"""Measure the Xsens yaw reference while the rig is stationary at home."""

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import time


def wrap180(angle):
    """Wrap an angle in degrees to [-180, 180)."""
    return (angle + 180.0) % 360.0 - 180.0


def circular_mean_deg(values):
    """Circular mean in degrees, safe across the +/-180-degree boundary."""
    sine = statistics.fmean(math.sin(math.radians(value)) for value in values)
    cosine = statistics.fmean(math.cos(math.radians(value)) for value in values)
    return math.degrees(math.atan2(sine, cosine))


def yaw_stddev_deg(values, mean):
    """Sample standard deviation of wrapped differences from the mean."""
    differences = [wrap180(value - mean) for value in values]
    return statistics.stdev(differences) if len(differences) > 1 else 0.0


def orientation_from_payload(xsens, payload):
    """Decode one MTData2 payload into yaw, pitch, roll in degrees."""
    data = xsens.parse_mtdata2(payload)
    quaternion = None
    if isinstance(data.get("Quaternion"), list):
        quaternion = data["Quaternion"]
    elif isinstance(data.get("EulerAngles"), list):
        quaternion = xsens.euler_to_quat(*data["EulerAngles"])
    if quaternion is None:
        return None

    quaternion = xsens.quat_mult(quaternion, xsens.MOUNT_QUAT)
    quaternion = xsens.quat_mult(
        xsens.AXIS_FIX,
        xsens.quat_mult(quaternion, xsens.AXIS_FIX),
    )
    roll, pitch, yaw = xsens.quat_to_rpy(quaternion)
    return yaw, pitch, roll


def drain_serial(serial_port, duration):
    """Discard buffered frames so the recorded samples are fresh."""
    serial_port.timeout = 0
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        serial_port.read(65536)
        time.sleep(0.01)
    serial_port.timeout = 0.5


def acquire(serial_port, xsens, sample_count):
    """Return rows of elapsed time, raw yaw, raw pitch, and raw roll."""
    rows = []
    started = time.monotonic()
    for message_id, payload in xsens.frames(serial_port):
        if message_id != xsens.MID_MTDATA2:
            continue
        orientation = orientation_from_payload(xsens, payload)
        if orientation is None:
            continue
        rows.append((time.monotonic() - started, *orientation))
        if len(rows) % 25 == 0 or len(rows) == sample_count:
            print(f"recorded {len(rows)}/{sample_count} IMU samples", end="\r", flush=True)
        if len(rows) >= sample_count:
            break
    print()
    return rows


def summarize(rows):
    yaws = [row[1] for row in rows]
    pitches = [row[2] for row in rows]
    rolls = [row[3] for row in rows]
    yaw0 = circular_mean_deg(yaws)
    return {
        "yaw0_deg": yaw0,
        "yaw_sample_stddev_deg": yaw_stddev_deg(yaws, yaw0),
        "pitch_home_mean_deg": statistics.fmean(pitches),
        "pitch_sample_stddev_deg": statistics.stdev(pitches) if len(pitches) > 1 else 0.0,
        "roll_home_mean_deg": statistics.fmean(rolls),
        "roll_sample_stddev_deg": statistics.stdev(rolls) if len(rolls) > 1 else 0.0,
    }


def write_raw_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as output:
        writer = csv.writer(output)
        writer.writerow(("sample", "elapsed_s", "yaw_raw_deg", "pitch_raw_deg", "roll_raw_deg"))
        for sample_index, row in enumerate(rows):
            writer.writerow((sample_index, *(f"{value:.6f}" for value in row)))


def write_result_json(path, args, summary):
    path.parent.mkdir(parents=True, exist_ok=True)
    result = {
        "schema_version": 1,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "calibration_type": "imu_yaw_reference_at_mechanical_home",
        "mechanical_pose_deg": {"yaw": 0.0, "pitch": 0.0, "roll": 0.0},
        "angle_unit": "degree",
        "yaw_rule": "yaw = wrap180(yaw_raw - yaw0)",
        "pitch_rule": "use raw IMU pitch directly",
        "roll_rule": "use raw IMU roll directly",
        "yaw0_deg": summary["yaw0_deg"],
        "stationary_statistics": {
            "yaw_sample_stddev_deg": summary["yaw_sample_stddev_deg"],
            "pitch_home_mean_deg": summary["pitch_home_mean_deg"],
            "pitch_sample_stddev_deg": summary["pitch_sample_stddev_deg"],
            "roll_home_mean_deg": summary["roll_home_mean_deg"],
            "roll_sample_stddev_deg": summary["roll_sample_stddev_deg"],
        },
        "acquisition": {
            "port": args.port,
            "baud": args.baud,
            "samples": args.samples,
            "buffer_drain_seconds": args.drain_seconds,
            "raw_samples_file": str(args.raw_output),
        },
    }
    with path.open("w") as output:
        json.dump(result, output, indent=2)
        output.write("\n")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--samples", type=int, default=200)
    parser.add_argument("--drain-seconds", type=float, default=0.3)
    parser.add_argument("--output", type=Path,
                        default=Path("imu_yaw_reference.json"))
    parser.add_argument("--raw-output", type=Path,
                        default=Path("imu_home_samples.csv"))
    parser.add_argument("--yes", action="store_true",
                        help="skip the mechanical-home confirmation")
    parser.add_argument("--force", action="store_true",
                        help="replace existing output files")
    args = parser.parse_args()

    if args.samples < 1:
        parser.error("--samples must be at least 1")
    if args.drain_seconds < 0:
        parser.error("--drain-seconds cannot be negative")
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
    print("Xsens yaw-reference measurement")
    print("Required condition: rig stationary at mechanical (0, 0, 0) home.")
    if not args.yes:
        response = input("Press ENTER to confirm and begin, or type q to quit: ").strip().lower()
        if response == "q":
            print("cancelled")
            return

    import serial
    from tools import xsens_mti630_reader as xsens

    serial_port = serial.Serial(args.port, args.baud, timeout=0.5)
    try:
        drain_serial(serial_port, args.drain_seconds)
        rows = acquire(serial_port, xsens, args.samples)
    finally:
        serial_port.close()

    summary = summarize(rows)
    write_raw_csv(args.raw_output, rows)
    write_result_json(args.output, args, summary)

    print(f"yaw0:       {summary['yaw0_deg']:+.6f} deg")
    print(f"yaw std:     {summary['yaw_sample_stddev_deg']:.6f} deg")
    print(f"home pitch: {summary['pitch_home_mean_deg']:+.6f} deg "
          f"(std {summary['pitch_sample_stddev_deg']:.6f})")
    print(f"home roll:  {summary['roll_home_mean_deg']:+.6f} deg "
          f"(std {summary['roll_sample_stddev_deg']:.6f})")
    print(f"wrote raw samples: {args.raw_output}")
    print(f"wrote yaw reference: {args.output}")


if __name__ == "__main__":
    main()
