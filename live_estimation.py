#!/usr/bin/env python3
"""Live 3D orientation from the two magnetic sensors alone — no IMU."""

import argparse
import json
from pathlib import Path
import threading
import time

import numpy as np

from live_estimation_vs_imu import (
    configure_panel,
    set_orientation,
    angle_title,
    load_locked_json,
)
from physical_estimator import PhysicalModelEstimator
from tools.tlv493d_coherent import (
    READER_TYPE,
    open_sensor_pair,
    prime_sensor_pair,
    read_pair_mT,
)


class MagneticPoseSource:
    def __init__(self, args):
        self.args = args
        with args.manifest.open() as source:
            manifest = json.load(source)
        load_locked_json(args.geometry, manifest, "geometry_priors")
        offsets_data = load_locked_json(args.offsets, manifest, "sensor_offsets")
        if offsets_data.get("sensor_reader", {}).get("type") != READER_TYPE:
            raise ValueError("sensor offsets do not use the coherent TLV reader")
        self.offsets_mT = np.asarray(
            offsets_data["offsets_mT"]["S1"] + offsets_data["offsets_mT"]["S2"],
            dtype=float,
        )
        self.estimator = PhysicalModelEstimator(
            model_path=args.model, geometry_path=args.geometry,
            correction_path=args.correction,
        )
        identifiers = self.estimator.model.get("input_identifiers", {})
        if identifiers.get("calibration_sha256") != manifest["files"]["calibration"]["sha256"]:
            raise ValueError("physical model does not match the locked calibration data")

        _buses, self.sensors = open_sensor_pair(args.bus1, args.bus2)
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.estimate = np.zeros(3)
        self.model_rms_mT = np.nan
        self.reacquired = False
        self.error = None

    def start(self):
        print(f"yaw zero correction: {self.estimator.yaw_zero_offset_deg:+.3f} deg "
              "(dial frame)")
        print(f"coarse estimator grid: {len(self.estimator.grid_poses)} poses")
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=2.0)

    def snapshot(self):
        with self.lock:
            return (
                self.estimate.copy(), self.model_rms_mT, self.reacquired,
                self.error,
            )

    def _run(self):
        previous = None
        try:
            while not self.stop_event.is_set():
                prime_sensor_pair(self.sensors)
                samples = []
                for _ in range(self.args.samples):
                    samples.append(read_pair_mT(self.sensors))
                    if self.args.sample_delay:
                        time.sleep(self.args.sample_delay)
                corrected_mT = np.mean(samples, axis=0) - self.offsets_mT
                result = self.estimator.estimate(
                    corrected_mT,
                    seed=None if self.args.cold_start else previous,
                    global_starts=self.args.global_starts,
                    reacquire_threshold_mT=self.args.reacquire_threshold_mT,
                )
                previous = result["angles_deg"]
                with self.lock:
                    self.estimate = result["angles_deg"]
                    self.model_rms_mT = result["model_rms_mT"]
                    self.reacquired = result["reacquired"]
        except Exception as error:
            with self.lock:
                self.error = error


def run(args):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    source = MagneticPoseSource(args)
    source.start()
    figure = plt.figure(figsize=(7.2, 6.8))
    axis = figure.add_subplot(1, 1, 1, projection="3d")
    artists = configure_panel(axis, "Magnetic estimate")
    figure.suptitle("Ball-joint orientation — magnetic only", fontsize=15)
    status = figure.text(0.5, 0.025, "Waiting for measurements...",
                         ha="center", fontsize=10)
    figure.subplots_adjust(left=0.05, right=0.95, bottom=0.1, top=0.86)
    closed = False

    def update(_frame):
        estimate, rms, reacquired, error = source.snapshot()
        if error is not None:
            status.set_text(f"Acquisition stopped: {error}")
            status.set_color("#b00020")
            return ()
        set_orientation(artists, estimate)
        axis.set_title(angle_title("Magnetic estimate", estimate), pad=12)
        mode = "global reacquisition" if reacquired else "tracking"
        status.set_text(f"model RMS {rms:.3f} mT     {mode}")
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
    parser.add_argument("--samples", type=int, default=8)
    parser.add_argument("--sample-delay", type=float, default=0.03)
    parser.add_argument("--refresh-ms", type=int, default=100)
    parser.add_argument("--global-starts", type=int, default=3)
    parser.add_argument("--reacquire-threshold-mT", type=float, default=0.25)
    parser.add_argument("--cold-start", action="store_true",
                        help="perform a global search for every frame")
    parser.add_argument("--bus1", type=int, default=3)
    parser.add_argument("--bus2", type=int, default=4)
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
