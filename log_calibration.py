#!/usr/bin/env python3
"""Record calibration poses: IMU angles (ground truth) + both TLV493D sensors.

On startup the script reads the current yaw and remembers it as the
reference: the starting pose is yaw = 0, and every recorded yaw is
(measured yaw - start yaw). So START THE SCRIPT AT THE HOME POSE.
Pitch and roll come from gravity and are used as-is.

The IMU ground-truth angles are displayed continuously. Place the rig at a
pose, hold it still, and press ENTER. The script averages N_SAMPLES fresh
magnetic readings and all IMU frames from the same acquisition window, then
appends one row to calibration_data.csv:

    pose_id, yaw_truth, pitch_truth, roll_truth, B1x,B1y,B1z, B2x,B2y,B2z, temp_C

Angles in deg (same corrections as tools/xsens_mti630_reader.py), fields in mT.

Run on the Pi:
    env/bin/python log_calibration.py            # ENTER=record, q=quit
    env/bin/python log_calibration.py --check    # one snapshot, no CSV
"""
import argparse
from collections import deque
import math
import os
import select
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "tools"))
import xsens_mti630_reader as xsens  # noqa: E402  (XBus parser + mount fixes)

import serial  # noqa: E402
from adafruit_extended_bus import ExtendedI2C  # noqa: E402
import adafruit_tlv493d  # noqa: E402

CSV_PATH = "calibration_data.csv"
CSV_HEADER = ("pose_id,yaw_truth,pitch_truth,roll_truth,"
              "B1x,B1y,B1z,B2x,B2y,B2z,temp_C\n")
SENSOR_BUSES = [("S1", 3), ("S2", 4)]   # same wiring as tools/read_tlv493d.py
N_SAMPLES = 16


def mean_deg(values):
    """Circular mean in degrees, safe around the +/-180 boundary."""
    s = sum(math.sin(math.radians(v)) for v in values)
    c = sum(math.cos(math.radians(v)) for v in values)
    return math.degrees(math.atan2(s, c))


def orientation_from_payload(payload):
    """Decode one MTData2 payload into (yaw, pitch, roll), or return None."""
    data = xsens.parse_mtdata2(payload)
    quat = None
    if isinstance(data.get("Quaternion"), list):
        quat = data["Quaternion"]
    elif isinstance(data.get("EulerAngles"), list):
        quat = xsens.euler_to_quat(*data["EulerAngles"])
    if quat is None:
        return None

    quat = xsens.quat_mult(quat, xsens.MOUNT_QUAT)
    quat = xsens.quat_mult(
        xsens.AXIS_FIX, xsens.quat_mult(quat, xsens.AXIS_FIX)
    )
    roll, pitch, yaw = xsens.quat_to_rpy(quat)
    return yaw, pitch, roll


def read_imu_angles(ser, n=N_SAMPLES):
    """(yaw, pitch, roll) in deg, averaged over n samples taken right now.

    The IMU streams continuously, so the port buffers hold OLD frames from
    before ENTER was pressed (a single reset_input_buffer is not enough --
    verified 10+ s stale). Read-and-discard for 0.3 s: buffered data drains
    at memory speed, far faster than the ~5 kB/s the IMU produces, so after
    this every frame is live.
    """
    ser.timeout = 0
    end = time.time() + 0.3
    while time.time() < end:
        ser.read(65536)
        time.sleep(0.01)
    ser.timeout = 0.5

    angles = []
    for mid, payload in xsens.frames(ser):
        if mid != xsens.MID_MTDATA2:
            continue
        angle = orientation_from_payload(payload)
        if angle is None:
            continue
        angles.append(angle)
        if len(angles) >= n:
            break

    yaws, pitches, rolls = zip(*angles)
    return mean_deg(yaws), mean_deg(pitches), mean_deg(rolls)


class LiveIMU:
    """Continuously consume the serial stream and retain timestamped angles."""

    def __init__(self, ser, max_samples=10000):
        self.ser = ser
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
            for mid, payload in xsens.frames(self.ser):
                if self.stop_event.is_set():
                    break
                if mid != xsens.MID_MTDATA2:
                    continue
                angles = orientation_from_payload(payload)
                if angles is not None:
                    with self.lock:
                        self.samples.append((time.monotonic(), *angles))
        except Exception as exc:
            if not self.stop_event.is_set():
                self.error = exc

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

    def latest(self):
        with self.lock:
            return None if not self.samples else self.samples[-1][1:]

    def average_between(self, start, end):
        with self.lock:
            selected = [sample[1:] for sample in self.samples
                        if start <= sample[0] <= end]
        if not selected:
            latest = self.latest()
            if latest is None:
                raise RuntimeError("no IMU samples available for averaging")
            selected = [latest]
        yaws, pitches, rolls = zip(*selected)
        return mean_deg(yaws), mean_deg(pitches), mean_deg(rolls)


def read_fields(sensors, n=N_SAMPLES, delay=0.03):
    """6 values [B1x..B2z] in mT, each the average of n samples."""
    sums = [0.0] * 6
    for _ in range(n):
        row = []
        for _, chip in sensors:
            bx, by, bz = chip.magnetic          # microtesla
            row += [bx / 1000.0, by / 1000.0, bz / 1000.0]
        sums = [s + v for s, v in zip(sums, row)]
        time.sleep(delay)
    return [s / n for s in sums]


def wrap180(a):
    """Wrap an angle in degrees to (-180, 180]."""
    return (a - 180.0) % -360.0 + 180.0


def snapshot(ser, sensors, yaw0=0.0):
    yaw, pitch, roll = read_imu_angles(ser)
    angles = (wrap180(yaw - yaw0), pitch, roll)
    fields = read_fields(sensors)
    print(f"  IMU  yaw {angles[0]:8.2f}  pitch {angles[1]:7.2f}  "
          f"roll {angles[2]:7.2f}  deg")
    print(f"  S1   Bx {fields[0]:7.3f}  By {fields[1]:7.3f}  Bz {fields[2]:7.3f}  mT")
    print(f"  S2   Bx {fields[3]:7.3f}  By {fields[4]:7.3f}  Bz {fields[5]:7.3f}  mT")
    return angles, fields


def next_pose_id(path):
    if not os.path.exists(path):
        return 0
    with open(path) as f:
        return max(0, sum(1 for line in f if line.strip()) - 1)


def clear_live_line():
    print("\r" + " " * 130 + "\r", end="", flush=True)


def main(default_csv=CSV_PATH, default_target=50, dataset_name="calibration"):
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--csv", default=default_csv)
    ap.add_argument("--target", type=int, default=default_target,
                    help="displayed pose-count target; recording may continue past it")
    ap.add_argument("--samples", type=int, default=N_SAMPLES,
                    help="fresh magnetic samples averaged for each recorded pose")
    ap.add_argument("--sample-delay", type=float, default=0.03)
    ap.add_argument("--check", action="store_true",
                    help="print one snapshot and exit (nothing written)")
    args = ap.parse_args()

    if args.samples < 1:
        raise SystemExit("--samples must be at least 1")

    ser = serial.Serial(args.port, args.baud, timeout=0.5)
    sensors = [(name, adafruit_tlv493d.TLV493D(ExtendedI2C(bus)))
               for name, bus in SENSOR_BUSES]

    if args.check:
        try:
            snapshot(ser, sensors)
        finally:
            ser.close()
        return

    ser.reset_input_buffer()
    imu = LiveIMU(ser)
    imu.start()
    imu.wait_for_sample()
    zero_start = time.monotonic()
    time.sleep(0.5)
    yaw0, _, _ = imu.average_between(zero_start, time.monotonic())
    print(f"start yaw {yaw0:+.2f} deg captured as reference -- this pose is yaw 0")

    pose_id = next_pose_id(args.csv)
    print(f"logging {dataset_name} poses to {args.csv} (next pose_id {pose_id})")
    print("angles update live; hold still and press ENTER to record; type q then ENTER to quit")

    try:
        while True:
            yaw, pitch, roll = imu.wait_for_sample()
            yaw = wrap180(yaw - yaw0)
            status = (
                f"\rIMU truth  yaw {yaw:8.2f}  pitch {pitch:7.2f}  roll {roll:7.2f} deg"
                f"   recorded {pose_id}/{args.target}   [ENTER=record, q+ENTER=quit]"
            )
            print(status.ljust(130), end="", flush=True)

            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not ready:
                continue
            command = sys.stdin.readline()
            if command == "":
                break
            if command.strip().lower() == "q":
                break

            clear_live_line()
            start = time.monotonic()
            fields = read_fields(sensors, n=args.samples, delay=args.sample_delay)
            end = time.monotonic()
            yaw, pitch, roll = imu.average_between(start, end)
            angles = (wrap180(yaw - yaw0), pitch, roll)

            new_file = not os.path.exists(args.csv)
            with open(args.csv, "a") as f:
                if new_file:
                    f.write(CSV_HEADER)
                f.write(f"{pose_id},{angles[0]:.3f},{angles[1]:.3f},{angles[2]:.3f},"
                        + ",".join(f"{v:.4f}" for v in fields) + ",\n")
            print(f"recorded {pose_id}: yaw {angles[0]:.2f}, pitch {angles[1]:.2f}, "
                  f"roll {angles[2]:.2f} deg ({args.samples} magnetic samples)")
            pose_id += 1
    except KeyboardInterrupt:
        pass
    finally:
        clear_live_line()
        imu.stop()
        ser.close()
        print(f"finished with {pose_id} poses in {args.csv}")


if __name__ == "__main__":
    main()
