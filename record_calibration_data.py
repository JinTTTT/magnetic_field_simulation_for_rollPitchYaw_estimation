#!/usr/bin/env python3
"""Interactively record synchronized IMU and magnetic calibration poses."""

import argparse
from collections import deque
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import select
import statistics
import sys
import threading
import time

from measure_imu_yaw_reference import (
    circular_mean_deg,
    orientation_from_payload,
    wrap180,
    yaw_stddev_deg,
)
from tools.tlv493d_coherent import (
    READER_TYPE,
    open_sensor_pair,
    prime_sensor_pair,
    read_pair_mT,
)


CHANNELS = ("S1_Bx", "S1_By", "S1_Bz", "S2_Bx", "S2_By", "S2_Bz")
CSV_HEADER = (
    "pose_id", "session_id", "recorded_at_utc",
    "yaw_deg", "pitch_deg", "roll_deg",
    "yaw_raw_deg", "yaw0_deg",
    "yaw_stddev_deg", "pitch_stddev_deg", "roll_stddev_deg",
    "imu_samples", "magnetic_samples",
    *(f"{name}_raw_mT" for name in CHANNELS),
    *(f"{name}_stddev_mT" for name in CHANNELS),
    *(f"{name}_corrected_mT" for name in CHANNELS),
)


def load_sensor_offsets(path):
    with path.open() as source:
        data = json.load(source)
    if data.get("sensor_reader", {}).get("type") != READER_TYPE:
        raise ValueError(
            f"{path} predates the coherent TLV493D reader; "
            "repeat measure_sensor_offsets.py first"
        )
    try:
        values = data["offsets_mT"]["S1"] + data["offsets_mT"]["S2"]
    except (KeyError, TypeError) as error:
        raise ValueError(f"{path} is not a static sensor-offset calibration") from error
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise ValueError(f"{path} must contain six finite offsets in mT")
    return [float(value) for value in values]


def load_yaw_reference(path):
    with path.open() as source:
        data = json.load(source)
    try:
        yaw0 = float(data["yaw0_deg"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"{path} is not an IMU yaw-reference calibration") from error
    if not math.isfinite(yaw0):
        raise ValueError(f"{path} must contain a finite yaw0_deg")
    return yaw0


class LiveIMU:
    """Continuously retain timestamped corrected-axis Xsens orientations."""

    def __init__(self, serial_port, xsens, max_samples=10000):
        self.serial_port = serial_port
        self.xsens = xsens
        self.samples = deque(maxlen=max_samples)
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.error = None
        self.thread = threading.Thread(target=self._read, daemon=True)

    def start(self):
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=1.0)

    def _read(self):
        try:
            for message_id, payload in self.xsens.frames(self.serial_port):
                if self.stop_event.is_set():
                    break
                if message_id != self.xsens.MID_MTDATA2:
                    continue
                orientation = orientation_from_payload(self.xsens, payload)
                if orientation is not None:
                    with self.lock:
                        self.samples.append((time.monotonic(), *orientation))
        except Exception as error:
            if not self.stop_event.is_set():
                self.error = error

    def latest(self):
        with self.lock:
            return None if not self.samples else self.samples[-1][1:]

    def wait_for_sample(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.error is not None:
                raise RuntimeError("IMU reader stopped") from self.error
            latest = self.latest()
            if latest is not None:
                return latest
            time.sleep(0.02)
        raise TimeoutError("no orientation frame received from the IMU")

    def samples_between(self, start, end):
        with self.lock:
            selected = [sample[1:] for sample in self.samples
                        if start <= sample[0] <= end]
        if not selected:
            raise RuntimeError("no IMU samples were received during magnetic acquisition")
        return selected


def summarize_imu(samples, yaw0):
    yaws = [sample[0] for sample in samples]
    pitches = [sample[1] for sample in samples]
    rolls = [sample[2] for sample in samples]
    yaw_raw = circular_mean_deg(yaws)
    return {
        "yaw_raw": yaw_raw,
        "yaw": wrap180(yaw_raw - yaw0),
        "pitch": statistics.fmean(pitches),
        "roll": statistics.fmean(rolls),
        "yaw_stddev": yaw_stddev_deg(yaws, yaw_raw),
        "pitch_stddev": statistics.stdev(pitches) if len(pitches) > 1 else 0.0,
        "roll_stddev": statistics.stdev(rolls) if len(rolls) > 1 else 0.0,
        "count": len(samples),
    }


def acquire_pose(sensors, imu, offsets, magnetic_samples, sample_delay, yaw0):
    prime_sensor_pair(sensors)
    start = time.monotonic()
    magnetic_rows = []
    for _ in range(magnetic_samples):
        magnetic_rows.append(read_pair_mT(sensors))
        if sample_delay:
            time.sleep(sample_delay)
    end = time.monotonic()

    imu_summary = summarize_imu(imu.samples_between(start, end), yaw0)
    columns = list(zip(*magnetic_rows))
    raw_means = [statistics.fmean(values) for values in columns]
    field_stddevs = [
        statistics.stdev(values) if len(values) > 1 else 0.0
        for values in columns
    ]
    corrected = [value - offset for value, offset in zip(raw_means, offsets)]
    return imu_summary, raw_means, field_stddevs, corrected


def existing_pose_count(path):
    if not path.exists() or path.stat().st_size == 0:
        return 0
    with path.open(newline="") as source:
        reader = csv.reader(source)
        header = tuple(next(reader))
        if header != CSV_HEADER:
            raise ValueError(f"{path} has an incompatible CSV schema")
        return sum(1 for row in reader if row)


def append_pose(path, pose_id, session_id, yaw0, imu_summary,
                magnetic_samples, raw_means, field_stddevs, corrected):
    new_file = not path.exists() or path.stat().st_size == 0
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", newline="") as output:
        writer = csv.writer(output)
        if new_file:
            writer.writerow(CSV_HEADER)
        writer.writerow((
            pose_id,
            session_id,
            datetime.now(timezone.utc).isoformat(),
            f"{imu_summary['yaw']:.6f}",
            f"{imu_summary['pitch']:.6f}",
            f"{imu_summary['roll']:.6f}",
            f"{imu_summary['yaw_raw']:.6f}",
            f"{yaw0:.6f}",
            f"{imu_summary['yaw_stddev']:.6f}",
            f"{imu_summary['pitch_stddev']:.6f}",
            f"{imu_summary['roll_stddev']:.6f}",
            imu_summary["count"],
            magnetic_samples,
            *(f"{value:.6f}" for value in raw_means),
            *(f"{value:.6f}" for value in field_stddevs),
            *(f"{value:.6f}" for value in corrected),
        ))


def clear_status_line():
    print("\r" + " " * 130 + "\r", end="", flush=True)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("calibration_data.csv"))
    parser.add_argument("--offsets", type=Path, default=Path("sensor_offsets.json"))
    parser.add_argument("--yaw-reference", type=Path,
                        default=Path("imu_yaw_reference.json"))
    parser.add_argument("--session-id", default=None)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--bus1", type=int, default=3)
    parser.add_argument("--bus2", type=int, default=4)
    parser.add_argument("--samples", type=int, default=32,
                        help="magnetic samples averaged per pose (default: 32)")
    parser.add_argument("--sample-delay", type=float, default=0.03)
    args = parser.parse_args()

    if args.samples < 1:
        parser.error("--samples must be at least 1")
    if args.sample_delay < 0:
        parser.error("--sample-delay cannot be negative")
    for path, label in ((args.offsets, "sensor offsets"),
                        (args.yaw_reference, "IMU yaw reference")):
        if not path.exists():
            parser.error(f"{label} not found: {path}")
    if args.session_id is None:
        args.session_id = datetime.now(timezone.utc).strftime("cal-%Y%m%dT%H%M%SZ")
    return args


def main():
    args = parse_args()
    offsets = load_sensor_offsets(args.offsets)
    yaw0 = load_yaw_reference(args.yaw_reference)
    pose_id = existing_pose_count(args.output)

    import serial
    from tools import xsens_mti630_reader as xsens

    _i2c_buses, sensors = open_sensor_pair(args.bus1, args.bus2)
    serial_port = serial.Serial(args.port, args.baud, timeout=0.1)
    serial_port.reset_input_buffer()
    imu = LiveIMU(serial_port, xsens)
    imu.start()
    imu.wait_for_sample()

    print(f"session: {args.session_id}")
    print(f"fixed yaw0: {yaw0:+.6f} deg from {args.yaw_reference}")
    print(f"recording to {args.output}; next pose_id {pose_id}")
    print("hold a pose and press ENTER to record; type q then ENTER to quit")

    try:
        while True:
            yaw_raw, pitch, roll = imu.wait_for_sample()
            yaw = wrap180(yaw_raw - yaw0)
            status = (
                f"\rIMU  yaw {yaw:8.2f}  pitch {pitch:8.2f}  roll {roll:8.2f} deg"
                f"   recorded {pose_id}   [ENTER=record, q+ENTER=quit]"
            )
            print(status.ljust(130), end="", flush=True)
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not ready:
                continue
            command = sys.stdin.readline()
            if command == "" or command.strip().lower() == "q":
                break

            clear_status_line()
            print(f"recording pose {pose_id}...")
            imu_summary, raw_means, field_stddevs, corrected = acquire_pose(
                sensors, imu, offsets, args.samples, args.sample_delay, yaw0
            )
            append_pose(
                args.output, pose_id, args.session_id, yaw0, imu_summary,
                args.samples, raw_means, field_stddevs, corrected
            )
            print(
                f"saved pose {pose_id}: yaw {imu_summary['yaw']:+.2f}, "
                f"pitch {imu_summary['pitch']:+.2f}, roll {imu_summary['roll']:+.2f} deg"
                f" from {imu_summary['count']} IMU and {args.samples} magnetic samples"
            )
            pose_id += 1
    except KeyboardInterrupt:
        pass
    finally:
        clear_status_line()
        imu.stop()
        serial_port.close()
        print(f"finished; {pose_id} total poses in {args.output}")


if __name__ == "__main__":
    main()
