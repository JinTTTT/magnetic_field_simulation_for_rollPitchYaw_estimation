#!/usr/bin/env python3
"""Estimate yaw, pitch and roll from recorded or live TLV493D readings.

The hardware reports mT, while estimation.estimate expects T. This script owns
that boundary: it subtracts the magnet-out offsets, converts mT to T, runs the
current lookup-table/model estimator, and reports the model residual in mT.

Examples:
    env/bin/python live_estimation.py --replay calibration_data.csv
    env/bin/python live_estimation.py
    env/bin/python live_estimation.py --track
    env/bin/python live_estimation.py --interactive
    env/bin/python live_estimation.py --compare-imu
"""
import argparse
import json
import time

import numpy as np

import simulation as sim
from estimation import GRID_POSES, GRID_READINGS, estimate


def load_offsets(path):
    with open(path) as f:
        data = json.load(f)
    offsets = np.asarray(data["S1"] + data["S2"], dtype=float)
    if offsets.shape != (6,):
        raise ValueError(f"{path} must contain three values for S1 and S2")
    return offsets


def check_table_matches_model():
    """Refuse to evaluate with a table built from different geometry."""
    indexes = (0, len(GRID_POSES) // 2, len(GRID_POSES) - 1)
    expected = np.array([sim.predict_readings(*GRID_POSES[i]) for i in indexes])
    if not np.allclose(GRID_READINGS[list(indexes)], expected, rtol=1e-7, atol=1e-10):
        raise SystemExit(
            "lookup_table.npz does not match the current model; run "
            "env/bin/python build_lookup_table.py"
        )


def estimate_fields(raw_fields_mT, offsets_mT, seed=None):
    """Return (angles, model_rms_mT, corrected_fields_mT)."""
    raw = np.asarray(raw_fields_mT, dtype=float)
    if raw.shape != (6,) or not np.all(np.isfinite(raw)):
        raise ValueError("a reading must contain six finite field values")

    corrected_mT = raw - offsets_mT
    measured_T = corrected_mT * 1e-3
    angles = estimate(measured_T, seed=seed)
    residual_mT = (sim.predict_readings(*angles) - measured_T) * 1e3
    model_rms_mT = float(np.sqrt(np.mean(residual_mT ** 2)))
    return angles, model_rms_mT, corrected_mT


def angle_errors(estimate_angles, truth_angles):
    """Absolute wrapped errors for yaw, pitch and roll."""
    delta = (np.asarray(estimate_angles) - np.asarray(truth_angles) + 180.0) % 360.0 - 180.0
    return np.abs(delta)


def print_result(index, angles, model_rms_mT, corrected_mT, truth=None):
    est_text = " ".join(f"{v:8.2f}" for v in angles)
    fields_text = " ".join(f"{v:7.3f}" for v in corrected_mT)
    if truth is None:
        print(f"{index:3d}  est {est_text}  | rms {model_rms_mT:6.3f} mT"
              f"  | B {fields_text}")
        return

    truth_text = " ".join(f"{v:8.2f}" for v in truth)
    errors = angle_errors(angles, truth)
    error_text = " ".join(f"{v:7.2f}" for v in errors)
    print(f"{index:3d}  truth {truth_text}  | est {est_text}"
          f"  | err {error_text}  | rms {model_rms_mT:6.3f} mT")


def replay(path, offsets_mT, track=False):
    rows = np.genfromtxt(path, delimiter=",", skip_header=1)
    rows = np.atleast_2d(rows)
    if rows.shape[1] < 10:
        raise ValueError(f"{path} must contain pose, truth angles, and six fields")

    print(" id  truth     yaw    pitch     roll  | est       yaw    pitch     roll"
          "  | err     yaw   pitch    roll  | model residual")
    all_errors = []
    all_residuals = []
    previous = None
    for row in rows:
        truth = row[1:4]
        seed = previous if track else None
        angles, rms, corrected = estimate_fields(row[4:10], offsets_mT, seed=seed)
        print_result(int(row[0]), angles, rms, corrected, truth=truth)
        all_errors.append(angle_errors(angles, truth))
        all_residuals.append(rms)
        previous = angles

    errors = np.asarray(all_errors)
    worst = errors.max(axis=1)
    print("\nsummary (degrees)")
    print("  per-axis median y/p/r: " + " ".join(f"{v:.2f}" for v in np.median(errors, axis=0)))
    print("  per-axis p95    y/p/r: " + " ".join(f"{v:.2f}" for v in np.percentile(errors, 95, axis=0)))
    print(f"  worst-axis median/p95/max: {np.median(worst):.2f} / "
          f"{np.percentile(worst, 95):.2f} / {worst.max():.2f}")
    print(f"  model RMS median/p95: {np.median(all_residuals):.3f} / "
          f"{np.percentile(all_residuals, 95):.3f} mT")


def run_live(args, offsets_mT):
    # Hardware-only imports keep --replay usable on a development machine.
    import log_calibration as hardware

    sensors = [
        ("S1", hardware.adafruit_tlv493d.TLV493D(hardware.ExtendedI2C(args.bus1))),
        ("S2", hardware.adafruit_tlv493d.TLV493D(hardware.ExtendedI2C(args.bus2))),
    ]
    serial_port = None
    yaw0 = 0.0
    if args.compare_imu:
        serial_port = hardware.serial.Serial(args.port, args.baud, timeout=0.5)
        yaw0, _, _ = hardware.read_imu_angles(serial_port, n=args.samples)
        print(f"IMU yaw reference {yaw0:+.2f} deg captured; start at the home pose")

    if args.verbose or args.compare_imu or not args.continuous:
        print("angles are yaw, pitch, roll in degrees; rms is model-to-measurement mismatch")
    if not args.continuous:
        print("hold the rig still and press ENTER to estimate; q=quit")

    previous = None
    index = 0
    try:
        while True:
            if not args.continuous:
                try:
                    command = input("\n> ").strip().lower()
                except EOFError:
                    break
                if command == "q":
                    break

            truth = None
            if serial_port is not None:
                yaw, pitch, roll = hardware.read_imu_angles(serial_port, n=args.samples)
                truth = np.array([hardware.wrap180(yaw - yaw0), pitch, roll])

            raw_fields = hardware.read_fields(
                sensors, n=args.samples, delay=args.sample_delay
            )
            seed = previous if args.track else None
            angles, rms, corrected = estimate_fields(raw_fields, offsets_mT, seed=seed)
            if args.verbose or truth is not None or not args.continuous:
                print_result(index, angles, rms, corrected, truth=truth)
            else:
                print(f"\ryaw {angles[0]:8.2f} deg   pitch {angles[1]:8.2f} deg   "
                      f"roll {angles[2]:8.2f} deg", end="", flush=True)
            previous = angles
            index += 1

            if args.continuous and args.interval > 0:
                time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        if args.continuous and not args.verbose and serial_port is None:
            print()
        if serial_port is not None:
            serial_port.close()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--replay", metavar="CSV", help="estimate every row in an existing calibration CSV")
    parser.add_argument("--offsets", default="sensor_offsets.json")
    parser.add_argument("--track", action="store_true", help="seed each estimate with the previous result")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--continuous", dest="continuous", action="store_true", default=True,
                      help="estimate continuously (default)")
    mode.add_argument("--interactive", dest="continuous", action="store_false",
                      help="wait for ENTER before each estimate")
    parser.add_argument("--verbose", action="store_true",
                        help="print fields and model residual for every estimate")
    parser.add_argument("--compare-imu", action="store_true", help="also print Xsens truth and angle errors")
    parser.add_argument("--samples", type=int, default=8)
    parser.add_argument("--sample-delay", type=float, default=0.03)
    parser.add_argument("--interval", type=float, default=0.0, help="extra delay between continuous estimates")
    parser.add_argument("--bus1", type=int, default=3)
    parser.add_argument("--bus2", type=int, default=4)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=921600)
    return parser.parse_args()


def main():
    args = parse_args()
    if args.samples < 1:
        raise SystemExit("--samples must be at least 1")
    check_table_matches_model()
    offsets_mT = load_offsets(args.offsets)
    if args.replay:
        replay(args.replay, offsets_mT, track=args.track)
    else:
        run_live(args, offsets_mT)


if __name__ == "__main__":
    main()
