#!/usr/bin/env python3
"""Show magnetic estimation beside an Xsens reference zeroed at startup."""

import argparse
from pathlib import Path
import threading
import time

import numpy as np

from magnetic_pose.config import (
    LOOKUP_PATH, MODEL_PATH, OFFSETS_PATH, load_sensor_offsets,
)
from magnetic_pose.imu import LiveIMU, circular_mean_deg, wrap180, yaw_stddev_deg
from magnetic_pose.lookup import PoseEstimator
from magnetic_pose.plotting import angle_title, configure_panel, set_orientation
from magnetic_pose.tlv493d import open_sensor_pair, prime_sensor_pair, read_pair_mT


XSENS_TITLE = "Ground truth reference: Xsens MTi-630 IMU"


class ComparisonSource:
    def __init__(self, args):
        import serial

        self.args = args
        self.offsets = load_sensor_offsets(args.offsets)
        self.estimator = PoseEstimator(args.model, args.lookup_table)
        _buses, self.sensors = open_sensor_pair(args.bus1, args.bus2)

        self.serial_port = serial.Serial(args.port, args.baud, timeout=0.1)
        self.serial_port.reset_input_buffer()
        self.imu = LiveIMU(self.serial_port)
        self.imu_yaw_zero = None

        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.estimate = np.zeros(3)
        self.reference = np.zeros(3)
        self.error = None

    def start(self):
        try:
            self.imu.start()
            self.imu.wait_for_sample()
            print(
                "zeroing Xsens yaw at mechanical home; "
                f"keep the rig stationary for {self.args.imu_zero_seconds:g} s"
            )
            started = time.monotonic()
            time.sleep(self.args.imu_zero_seconds)
            samples = self.imu.samples_between(started, time.monotonic())
            yaws = [sample[0] for sample in samples]
            if len(yaws) < 2:
                raise RuntimeError("fewer than two fresh IMU samples were received")
            self.imu_yaw_zero = circular_mean_deg(yaws)
            stddev = yaw_stddev_deg(yaws, self.imu_yaw_zero)
            if stddev > self.args.imu_zero_max_stddev_deg:
                raise RuntimeError(
                    f"IMU moved during startup calibration: {stddev:.3f}° stddev"
                )
            print(
                f"startup IMU yaw zero: {self.imu_yaw_zero:+.6f} deg "
                f"from {len(yaws)} samples (stddev {stddev:.4f} deg)"
            )
        except Exception:
            self.imu.stop()
            self.serial_port.close()
            raise

        print("magnetic model yaw frame: mechanical dial")
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=2.0)
        self.imu.stop()
        self.serial_port.close()

    def snapshot(self):
        with self.lock:
            return (
                self.estimate.copy(), self.reference.copy(), self.error,
            )

    def _read_fields(self):
        prime_sensor_pair(self.sensors)
        samples = []
        for _ in range(self.args.samples):
            samples.append(read_pair_mT(self.sensors))
            if self.args.sample_delay:
                time.sleep(self.args.sample_delay)
        return np.mean(samples, axis=0) - self.offsets

    def _run(self):
        try:
            while not self.stop_event.is_set():
                fields = self._read_fields()
                raw_yaw, pitch, roll = self.imu.latest()
                reference = np.array([
                    wrap180(raw_yaw - self.imu_yaw_zero), pitch, roll
                ])
                result = self.estimator.estimate(fields)
                with self.lock:
                    self.estimate = result["angles_deg"]
                    self.reference = reference
        except Exception as error:
            with self.lock:
                self.error = error


def run(args):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    source = ComparisonSource(args)
    source.start()
    figure = plt.figure(figsize=(13, 6.8))
    estimate_axis = figure.add_subplot(1, 2, 1, projection="3d")
    reference_axis = figure.add_subplot(1, 2, 2, projection="3d")
    estimate_artists = configure_panel(estimate_axis, "Magnetic estimate")
    reference_artists = configure_panel(reference_axis, XSENS_TITLE)
    status = figure.text(0.5, 0.025, "Waiting for measurements...", ha="center")
    figure.subplots_adjust(left=0.03, right=0.97, bottom=0.1, top=0.9, wspace=0.08)

    def update(_frame):
        estimate, reference, error = source.snapshot()
        if error:
            status.set_text(f"Acquisition stopped: {error}")
            status.set_color("#b00020")
            return ()
        set_orientation(estimate_artists, estimate)
        set_orientation(reference_artists, reference)
        estimate_axis.set_title(angle_title("Magnetic estimate", estimate), pad=12)
        reference_axis.set_title(angle_title(XSENS_TITLE, reference), pad=12)
        error_deg = np.abs((estimate - reference + 180.0) % 360.0 - 180.0)
        status.set_text(
            f"Error: yaw {error_deg[0]:.2f}°   pitch {error_deg[1]:.2f}°   "
            f"roll {error_deg[2]:.2f}°"
        )
        return ()

    closed = False
    def close(_event=None):
        nonlocal closed
        if not closed:
            closed = True
            source.stop()

    figure.canvas.mpl_connect("close_event", close)
    animation = FuncAnimation(
        figure, update, interval=args.refresh_ms, cache_frame_data=False
    )
    try:
        plt.show()
    finally:
        close()
    return animation


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=MODEL_PATH)
    parser.add_argument("--offsets", type=Path, default=OFFSETS_PATH)
    parser.add_argument("--lookup-table", type=Path, default=LOOKUP_PATH)
    parser.add_argument("--imu-zero-seconds", type=float, default=1.0)
    parser.add_argument("--imu-zero-max-stddev-deg", type=float, default=0.25)
    parser.add_argument("--samples", type=int, default=8)
    parser.add_argument("--sample-delay", type=float, default=0.03)
    parser.add_argument("--refresh-ms", type=int, default=100)
    parser.add_argument("--bus1", type=int, default=3)
    parser.add_argument("--bus2", type=int, default=4)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=921600)
    args = parser.parse_args()
    if args.samples < 1:
        parser.error("--samples must be positive")
    if args.imu_zero_seconds <= 0 or args.imu_zero_max_stddev_deg < 0:
        parser.error("invalid IMU zeroing settings")
    if args.sample_delay < 0:
        parser.error("--sample-delay cannot be negative")
    return args


if __name__ == "__main__":
    run(parse_args())
