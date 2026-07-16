#!/usr/bin/env python3
"""Show live magnetic pose estimation and Xsens ground truth in two 3D plots.

Start the script with the rig at its mechanical home pose. The initial Xsens
yaw becomes zero, matching the calibration and live-estimation convention.

    env/bin/python live_3d.py
"""
import argparse
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation

from live_estimation import estimate_fields, load_offsets


AXIS_COLORS = ("#d62728", "#2ca02c", "#1f77b4")
AXIS_NAMES = ("X", "Y", "Z")


class LivePoseSource:
    """Acquire synchronized field estimates and IMU truth off the GUI thread."""

    def __init__(self, args):
        import log_calibration as hardware

        self.hardware = hardware
        self.args = args
        self.offsets = load_offsets(args.offsets)
        self.sensors = [
            ("S1", hardware.adafruit_tlv493d.TLV493D(
                hardware.ExtendedI2C(args.bus1))),
            ("S2", hardware.adafruit_tlv493d.TLV493D(
                hardware.ExtendedI2C(args.bus2))),
        ]
        self.serial = hardware.serial.Serial(args.port, args.baud, timeout=0.5)
        self.serial.reset_input_buffer()
        self.imu = hardware.LiveIMU(self.serial)
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.estimate = np.zeros(3)
        self.truth = np.zeros(3)
        self.model_rms_mT = np.nan
        self.error = None
        self.yaw0 = 0.0

    def start(self):
        self.imu.start()
        self.imu.wait_for_sample()
        zero_start = time.monotonic()
        time.sleep(0.5)
        self.yaw0, _, _ = self.imu.average_between(
            zero_start, time.monotonic())
        print(f"IMU yaw reference {self.yaw0:+.2f} deg captured at home")
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=2.0)
        self.imu.stop()
        self.serial.close()

    def snapshot(self):
        with self.lock:
            return (self.estimate.copy(), self.truth.copy(),
                    self.model_rms_mT, self.error)

    def _run(self):
        previous = None
        try:
            while not self.stop_event.is_set():
                start = time.monotonic()
                fields = self.hardware.read_fields(
                    self.sensors, n=self.args.samples,
                    delay=self.args.sample_delay)
                end = time.monotonic()

                yaw, pitch, roll = self.imu.average_between(start, end)
                truth = np.array([
                    self.hardware.wrap180(yaw - self.yaw0), pitch, roll
                ])
                seed = None if self.args.cold_start else previous
                estimate, rms, _ = estimate_fields(fields, self.offsets, seed=seed)
                previous = estimate
                with self.lock:
                    self.estimate = np.asarray(estimate)
                    self.truth = truth
                    self.model_rms_mT = rms
        except Exception as exc:
            with self.lock:
                self.error = exc


def configure_panel(axis, title):
    axis.set_title(title, fontsize=12, pad=12)
    axis.set_xlim(-1.15, 1.15)
    axis.set_ylim(-1.15, 1.15)
    axis.set_zlim(-1.15, 1.15)
    axis.set_box_aspect((1, 1, 1))
    axis.set_xlabel("world X")
    axis.set_ylabel("world Y")
    axis.set_zlabel("world Z")
    axis.view_init(elev=25, azim=-55)
    axis.grid(True, alpha=0.25)

    origin = np.zeros(3)
    basis = np.eye(3)
    for index, name in enumerate(AXIS_NAMES):
        endpoint = basis[index]
        axis.plot(
            [origin[0], endpoint[0]], [origin[1], endpoint[1]],
            [origin[2], endpoint[2]], color="#9a9a9a", linestyle="--",
            linewidth=1.0, alpha=0.7)
        axis.text(*(endpoint * 1.08), f"{name}w", color="#777777", fontsize=9)

    body_lines = []
    body_labels = []
    for name, color in zip(AXIS_NAMES, AXIS_COLORS):
        line, = axis.plot(
            [0, 0], [0, 0], [0, 0], color=color, linewidth=3,
            marker="o", markevery=[1], markersize=5, label=f"body {name}")
        body_lines.append(line)
        body_labels.append(axis.text(0, 0, 0, name, color=color,
                                     fontsize=11, fontweight="bold"))
    axis.legend(loc="upper left", fontsize=8, framealpha=0.9)
    return body_lines, body_labels


def set_orientation(body_lines, body_labels, angles):
    rotation = Rotation.from_euler("ZYX", angles, degrees=True)
    endpoints = rotation.apply(np.eye(3))
    for line, label, endpoint in zip(body_lines, body_labels, endpoints):
        line.set_data_3d([0, endpoint[0]], [0, endpoint[1]], [0, endpoint[2]])
        label.set_position_3d(endpoint * 1.08)


def angle_title(name, angles):
    return (f"{name}\nYaw {angles[0]:7.2f} deg   "
            f"Pitch {angles[1]:7.2f} deg   Roll {angles[2]:7.2f} deg")


def build_figure():
    import matplotlib.pyplot as plt

    figure = plt.figure(figsize=(13, 6.8))
    estimated_axis = figure.add_subplot(1, 2, 1, projection="3d")
    truth_axis = figure.add_subplot(1, 2, 2, projection="3d")
    estimated_artists = configure_panel(estimated_axis, "Magnetic estimate")
    truth_artists = configure_panel(truth_axis, "Xsens ground truth")
    figure.suptitle("Ball-joint orientation", fontsize=15)
    status = figure.text(0.5, 0.025, "Waiting for measurements...",
                         ha="center", fontsize=10)
    figure.subplots_adjust(left=0.03, right=0.97, bottom=0.1, top=0.88, wspace=0.08)
    return figure, estimated_axis, truth_axis, estimated_artists, truth_artists, status


def run(args):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    source = LivePoseSource(args)
    source.start()
    figure, estimated_axis, truth_axis, estimated_artists, truth_artists, status = (
        build_figure())
    closed = False

    def update(_frame):
        estimate, truth, rms, error = source.snapshot()
        if error is not None:
            status.set_text(f"Acquisition stopped: {error}")
            status.set_color("#b00020")
            return ()

        set_orientation(*estimated_artists, estimate)
        set_orientation(*truth_artists, truth)
        estimated_axis.set_title(angle_title("Magnetic estimate", estimate),
                                 fontsize=12, pad=12)
        truth_axis.set_title(angle_title("Xsens ground truth", truth),
                             fontsize=12, pad=12)
        angle_error = np.abs((estimate - truth + 180.0) % 360.0 - 180.0)
        status.set_text(
            f"Absolute error: yaw {angle_error[0]:.2f} deg   "
            f"pitch {angle_error[1]:.2f} deg   roll {angle_error[2]:.2f} deg"
            f"     model RMS {rms:.3f} mT")
        return ()

    def close(_event=None):
        nonlocal closed
        if not closed:
            closed = True
            source.stop()

    figure.canvas.mpl_connect("close_event", close)
    animation = FuncAnimation(
        figure, update, interval=args.refresh_ms, cache_frame_data=False)
    try:
        plt.show()
    finally:
        close()
    return animation


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--offsets", default="sensor_offsets.json")
    parser.add_argument("--samples", type=int, default=8)
    parser.add_argument("--sample-delay", type=float, default=0.03)
    parser.add_argument("--refresh-ms", type=int, default=100)
    parser.add_argument("--cold-start", action="store_true",
                        help="use lookup starts every frame instead of tracking")
    parser.add_argument("--bus1", type=int, default=3)
    parser.add_argument("--bus2", type=int, default=4)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=921600)
    args = parser.parse_args()
    if args.samples < 1:
        parser.error("--samples must be at least 1")
    if args.refresh_ms < 10:
        parser.error("--refresh-ms must be at least 10")
    return args


if __name__ == "__main__":
    run(parse_args())
