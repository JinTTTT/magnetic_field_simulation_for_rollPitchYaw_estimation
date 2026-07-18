#!/usr/bin/env python3
"""Show live yaw, pitch, and roll from the two magnetic sensors."""

import argparse
from pathlib import Path
import threading
import time

import numpy as np

from magnetic_pose.config import (
    LOOKUP_PATH, MODEL_PATH, OFFSETS_PATH, load_sensor_offsets,
)
from magnetic_pose.lookup import PoseEstimator
from magnetic_pose.plotting import angle_title, configure_panel, set_orientation
from magnetic_pose.tlv493d import open_sensor_pair, prime_sensor_pair, read_pair_mT


class MagneticSource:
    def __init__(self, args):
        self.args = args
        self.offsets = load_sensor_offsets(args.offsets)
        self.estimator = PoseEstimator(args.model, args.lookup_table)
        _buses, self.sensors = open_sensor_pair(args.bus1, args.bus2)

        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.angles = np.zeros(3)
        self.error = None

    def start(self):
        print("magnetic model yaw frame: mechanical dial")
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=2.0)

    def snapshot(self):
        with self.lock:
            return self.angles.copy(), self.error

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
                result = self.estimator.estimate(fields)
                with self.lock:
                    self.angles = result["angles_deg"]
        except Exception as error:
            with self.lock:
                self.error = error


def run(args):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    source = MagneticSource(args)
    source.start()
    figure = plt.figure(figsize=(7.2, 6.8))
    axis = figure.add_subplot(1, 1, 1, projection="3d")
    artists = configure_panel(axis, "Magnetic estimate")
    status = figure.text(0.5, 0.025, "Waiting for measurements...", ha="center")
    figure.subplots_adjust(left=0.05, right=0.95, bottom=0.1, top=0.9)

    def update(_frame):
        angles, error = source.snapshot()
        if error:
            status.set_text(f"Acquisition stopped: {error}")
            status.set_color("#b00020")
            return ()
        set_orientation(artists, angles)
        axis.set_title(angle_title("Magnetic estimate", angles), pad=12)
        status.set_text("")
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
    parser.add_argument("--samples", type=int, default=8)
    parser.add_argument("--sample-delay", type=float, default=0.03)
    parser.add_argument("--refresh-ms", type=int, default=100)
    parser.add_argument("--bus1", type=int, default=3)
    parser.add_argument("--bus2", type=int, default=4)
    args = parser.parse_args()
    if args.samples < 1:
        parser.error("--samples must be positive")
    if args.sample_delay < 0:
        parser.error("--sample-delay cannot be negative")
    return args


if __name__ == "__main__":
    run(parse_args())
