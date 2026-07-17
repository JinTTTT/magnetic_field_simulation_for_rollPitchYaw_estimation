#!/usr/bin/env python3
"""Compare live physical-model magnetic estimation with the fixed-reference IMU."""

import argparse
import hashlib
import json
from pathlib import Path
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation

from measure_imu_yaw_reference import wrap180
from physical_estimator import PhysicalModelEstimator
from record_calibration_data import LiveIMU
from tools.tlv493d_coherent import (
    READER_TYPE,
    open_sensor_pair,
    prime_sensor_pair,
    read_pair_mT,
)


AXIS_COLORS = ("#d62728", "#2ca02c", "#1f77b4")
AXIS_NAMES = ("X", "Y", "Z")


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_locked_json(path, manifest, role):
    entry = manifest["files"][role]
    if sha256_file(path) != entry["sha256"]:
        raise ValueError(f"{path} does not match locked {role} input")
    with Path(path).open() as source:
        return json.load(source)


class LivePoseSource:
    def __init__(self, args):
        import serial
        from tools import xsens_mti630_reader as xsens

        self.args = args
        with args.manifest.open() as source:
            manifest = json.load(source)
        load_locked_json(args.geometry, manifest, "geometry_priors")
        offsets_data = load_locked_json(args.offsets, manifest, "sensor_offsets")
        yaw_data = load_locked_json(args.yaw_reference, manifest, "imu_yaw_reference")
        if offsets_data.get("sensor_reader", {}).get("type") != READER_TYPE:
            raise ValueError("sensor offsets do not use the coherent TLV reader")
        self.offsets_mT = np.asarray(
            offsets_data["offsets_mT"]["S1"] + offsets_data["offsets_mT"]["S2"],
            dtype=float,
        )
        self.yaw0_deg = float(yaw_data["yaw0_deg"])
        self.estimator = PhysicalModelEstimator(
            model_path=args.model, geometry_path=args.geometry,
            correction_path=args.correction,
        )
        identifiers = self.estimator.model.get("input_identifiers", {})
        if identifiers.get("calibration_sha256") != manifest["files"]["calibration"]["sha256"]:
            raise ValueError("physical model does not match the locked calibration data")
        if identifiers.get("geometry_priors_sha256") != manifest["files"]["geometry_priors"]["sha256"]:
            raise ValueError("physical model does not match the locked geometry priors")

        _buses, self.sensors = open_sensor_pair(args.bus1, args.bus2)
        self.serial_port = serial.Serial(args.port, args.baud, timeout=0.1)
        self.serial_port.reset_input_buffer()
        self.imu = LiveIMU(self.serial_port, xsens)
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.estimate = np.zeros(3)
        self.truth = np.zeros(3)
        self.model_rms_mT = np.nan
        self.reacquired = False
        self.error = None

    def start(self):
        self.imu.start()
        self.imu.wait_for_sample()
        print(f"using fixed IMU yaw0 {self.yaw0_deg:+.6f} deg")
        print(f"yaw zero correction: {self.estimator.yaw_zero_offset_deg:+.3f} deg "
              "(model frame -> dial frame)")
        print(f"coarse estimator grid: {len(self.estimator.grid_poses)} poses")
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=2.0)
        self.imu.stop()
        self.serial_port.close()

    def snapshot(self):
        with self.lock:
            return (
                self.estimate.copy(), self.truth.copy(), self.model_rms_mT,
                self.reacquired, self.error,
            )

    def _acquire_fields(self):
        prime_sensor_pair(self.sensors)
        start = time.monotonic()
        samples = []
        for _ in range(self.args.samples):
            samples.append(read_pair_mT(self.sensors))
            if self.args.sample_delay:
                time.sleep(self.args.sample_delay)
        end = time.monotonic()
        return np.mean(samples, axis=0), start, end

    def _run(self):
        previous = None
        try:
            while not self.stop_event.is_set():
                raw_mT, _start, _end = self._acquire_fields()
                yaw_raw, pitch, roll = self.imu.latest()
                truth = np.asarray((
                    wrap180(yaw_raw - self.yaw0_deg), pitch, roll,
                ))
                corrected_mT = raw_mT - self.offsets_mT
                result = self.estimator.estimate(
                    corrected_mT,
                    seed=None if self.args.cold_start else previous,
                    global_starts=self.args.global_starts,
                    reacquire_threshold_mT=self.args.reacquire_threshold_mT,
                )
                previous = result["angles_deg"]
                with self.lock:
                    self.estimate = result["angles_deg"]
                    self.truth = truth
                    self.model_rms_mT = result["model_rms_mT"]
                    self.reacquired = result["reacquired"]
        except Exception as error:
            with self.lock:
                self.error = error


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

    for index, name in enumerate(AXIS_NAMES):
        endpoint = np.eye(3)[index]
        axis.plot((0, endpoint[0]), (0, endpoint[1]), (0, endpoint[2]),
                  color="#999999", linestyle="--", linewidth=1, alpha=0.7)
        axis.text(*(endpoint * 1.08), f"{name}w", color="#777777", fontsize=9)

    lines, labels = [], []
    for name, color in zip(AXIS_NAMES, AXIS_COLORS):
        line, = axis.plot((0, 0), (0, 0), (0, 0), color=color, linewidth=3,
                          marker="o", markevery=[1], markersize=5,
                          label=f"body {name}")
        lines.append(line)
        labels.append(axis.text(0, 0, 0, name, color=color, fontsize=11,
                                fontweight="bold"))
    axis.legend(loc="upper left", fontsize=8, framealpha=0.9)
    return lines, labels


def set_orientation(artists, angles):
    lines, labels = artists
    endpoints = Rotation.from_euler("ZYX", angles, degrees=True).apply(np.eye(3))
    for line, label, endpoint in zip(lines, labels, endpoints):
        line.set_data_3d((0, endpoint[0]), (0, endpoint[1]), (0, endpoint[2]))
        label.set_position_3d(endpoint * 1.08)


def angle_title(name, angles):
    return (f"{name}\nYaw {angles[0]:7.2f}°   Pitch {angles[1]:7.2f}°   "
            f"Roll {angles[2]:7.2f}°")


def run(args):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    source = LivePoseSource(args)
    source.start()
    figure = plt.figure(figsize=(13, 6.8))
    estimate_axis = figure.add_subplot(1, 2, 1, projection="3d")
    truth_axis = figure.add_subplot(1, 2, 2, projection="3d")
    estimate_artists = configure_panel(estimate_axis, "Magnetic estimate")
    truth_artists = configure_panel(truth_axis, "Xsens reference (yaw may drift)")
    figure.suptitle("Ball-joint orientation — physical model only", fontsize=15)
    status = figure.text(0.5, 0.025, "Waiting for measurements...",
                         ha="center", fontsize=10)
    figure.subplots_adjust(left=0.03, right=0.97, bottom=0.1, top=0.88, wspace=0.08)
    closed = False

    def update(_frame):
        estimate, truth, rms, reacquired, error = source.snapshot()
        if error is not None:
            status.set_text(f"Acquisition stopped: {error}")
            status.set_color("#b00020")
            return ()
        set_orientation(estimate_artists, estimate)
        set_orientation(truth_artists, truth)
        estimate_axis.set_title(angle_title("Magnetic estimate", estimate), pad=12)
        truth_axis.set_title(angle_title("Xsens reference (yaw may drift)", truth), pad=12)
        absolute = np.abs((estimate - truth + 180.0) % 360.0 - 180.0)
        mode = "global reacquisition" if reacquired else "tracking"
        status.set_text(
            f"Absolute error: yaw {absolute[0]:.2f}°   pitch {absolute[1]:.2f}°   "
            f"roll {absolute[2]:.2f}°     model RMS {rms:.3f} mT     {mode}"
        )
        return ()

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
    parser.add_argument("--model", type=Path, default=Path("physical_model.json"))
    parser.add_argument("--correction", type=Path,
                        default=Path("yaw_zero_correction.json"))
    parser.add_argument("--geometry", type=Path, default=Path("geometry_priors.json"))
    parser.add_argument("--manifest", type=Path, default=Path("dataset_manifest.json"))
    parser.add_argument("--offsets", type=Path, default=Path("sensor_offsets.json"))
    parser.add_argument("--yaw-reference", type=Path,
                        default=Path("imu_yaw_reference.json"))
    parser.add_argument("--samples", type=int, default=8)
    parser.add_argument("--sample-delay", type=float, default=0.03)
    parser.add_argument("--refresh-ms", type=int, default=100)
    parser.add_argument("--global-starts", type=int, default=3)
    parser.add_argument("--reacquire-threshold-mT", type=float, default=0.25)
    parser.add_argument("--cold-start", action="store_true",
                        help="perform a global search for every frame")
    parser.add_argument("--bus1", type=int, default=3)
    parser.add_argument("--bus2", type=int, default=4)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=921600)
    args = parser.parse_args()
    if args.samples < 1 or args.global_starts < 1:
        parser.error("--samples and --global-starts must be at least 1")
    if min(args.sample_delay, args.reacquire_threshold_mT) < 0:
        parser.error("delays and thresholds cannot be negative")
    if args.refresh_ms < 10:
        parser.error("--refresh-ms must be at least 10")
    return args


if __name__ == "__main__":
    run(parse_args())
