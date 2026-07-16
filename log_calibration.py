#!/usr/bin/env python3
"""Record calibration poses: IMU angles (ground truth) + both TLV493D sensors.

On startup the script reads the current yaw and remembers it as the
reference: the starting pose is yaw = 0, and every recorded yaw is
(measured yaw - start yaw). So START THE SCRIPT AT THE HOME POSE.
Pitch and roll come from gravity and are used as-is.

Then: place the rig at a pose, hold it still, press ENTER -- the script reads
the IMU and both sensors AT THAT MOMENT (averaging N_SAMPLES fresh samples)
and appends one row to calibration_data.csv:

    pose_id, yaw_truth, pitch_truth, roll_truth, B1x,B1y,B1z, B2x,B2y,B2z, temp_C

Angles in deg (same corrections as tools/xsens_mti630_reader.py), fields in mT.

Run on the Pi:
    env/bin/python log_calibration.py            # ENTER=record, q=quit
    env/bin/python log_calibration.py --check    # one snapshot, no CSV
"""
import argparse
import math
import os
import sys
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

    rpys = []
    for mid, payload in xsens.frames(ser):
        if mid != xsens.MID_MTDATA2:
            continue
        d = xsens.parse_mtdata2(payload)
        quat = None
        if isinstance(d.get("Quaternion"), list):
            quat = d["Quaternion"]
        elif isinstance(d.get("EulerAngles"), list):
            quat = xsens.euler_to_quat(*d["EulerAngles"])
        if quat is None:
            continue
        q = xsens.quat_mult(quat, xsens.MOUNT_QUAT)
        q = xsens.quat_mult(xsens.AXIS_FIX, xsens.quat_mult(q, xsens.AXIS_FIX))
        rpys.append(xsens.quat_to_rpy(q))
        if len(rpys) >= n:
            break

    def mean_deg(vals):                 # circular mean, safe near +-180
        s = sum(math.sin(math.radians(v)) for v in vals)
        c = sum(math.cos(math.radians(v)) for v in vals)
        return math.degrees(math.atan2(s, c))

    rolls, pitches, yaws = zip(*rpys)
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--csv", default=CSV_PATH)
    ap.add_argument("--check", action="store_true",
                    help="print one snapshot and exit (nothing written)")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.5)
    sensors = [(name, adafruit_tlv493d.TLV493D(ExtendedI2C(bus)))
               for name, bus in SENSOR_BUSES]

    if args.check:
        snapshot(ser, sensors)
        return

    yaw0, _, _ = read_imu_angles(ser)
    print(f"start yaw {yaw0:+.2f} deg captured as reference -- this pose is yaw 0")

    pose_id = next_pose_id(args.csv)
    print(f"logging to {args.csv} (next pose_id {pose_id})")
    print("hold the rig still at a pose, press ENTER to record it; q=quit")

    while True:
        try:
            cmd = input("\n> ").strip().lower()
        except EOFError:
            break
        if cmd == "q":
            break
        angles, fields = snapshot(ser, sensors, yaw0)
        new_file = not os.path.exists(args.csv)
        with open(args.csv, "a") as f:
            if new_file:
                f.write(CSV_HEADER)
            f.write(f"{pose_id},{angles[0]:.3f},{angles[1]:.3f},{angles[2]:.3f},"
                    + ",".join(f"{v:.4f}" for v in fields) + ",\n")
        print(f"  recorded pose_id {pose_id}")
        pose_id += 1


if __name__ == "__main__":
    main()
