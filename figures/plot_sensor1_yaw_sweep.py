#!/usr/bin/env python3
"""Sensor 1's magnetic field vector across a 360 deg yaw sweep, for slides.

Uses an idealized rig geometry (not the fitted model): magnet centered on the
yaw axis at (0, 0, 24) mm with its pole axis perfectly aligned to world X,
and Sensor 1 at (0, -24, 24) mm with no chip misalignment. Pitch/roll are
zero; only yaw (rotation about world Z) sweeps 0-360 deg. Each vector is
Sensor 1's field in its own (co-rotating) frame.
"""

import os
import sys
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/magnetic_orientation_matplotlib")

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import magpylib as magpy
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

OUTPUT_PATH = Path(__file__).resolve().parent / "sensor1_yaw_sweep.png"

MAGNET_CENTER_MM = np.array([0.0, 0.0, 24.0])
MAGNET_DIMENSION_MM = (10.0, 5.0)  # diameter, height; same magnet as the fitted model
MAGNET_POLARIZATION_T = -1.1199896964866292
SENSOR_1_HOME_MM = np.array([0.0, -24.0, 24.0])

CURVE_STEP_DEG = 1
VIEW = dict(elev=89.9, azim=0)
LINE_COLOR = "#2c5f9e"


def build_ideal_magnet():
    orientation = Rotation.from_euler("y", 90.0, degrees=True)  # local axis -> world X
    return magpy.magnet.Cylinder(
        position=MAGNET_CENTER_MM / 1000.0,
        orientation=orientation,
        dimension=np.asarray(MAGNET_DIMENSION_MM) / 1000.0,
        polarization=(0.0, 0.0, MAGNET_POLARIZATION_T),
    )


def field_at_yaw(source, yaw_deg):
    pose = Rotation.from_euler("z", yaw_deg, degrees=True)
    position_m = pose.apply(SENSOR_1_HOME_MM / 1000.0)
    field_world_t = np.asarray(source.getB(position_m.reshape(1, 3))).reshape(3)
    return pose.inv().apply(field_world_t) * 1000.0


def main():
    source = build_ideal_magnet()
    yaws_curve = np.arange(0.0, 360.0, CURVE_STEP_DEG)
    fields_curve = np.array([field_at_yaw(source, yaw) for yaw in yaws_curve])
    closed = np.vstack((fields_curve, fields_curve[:1]))

    fig = plt.figure(figsize=(6.5, 6.5))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(closed[:, 0], closed[:, 1], closed[:, 2], color=LINE_COLOR, linewidth=2.5)

    span = 5.0
    ax.set_xlim(-span, span)
    ax.set_ylim(-span, span)
    ax.set_zlim(-span, span)
    ax.set_box_aspect((1, 1, 1))
    ax.set_xlabel("Bx (mT)")
    ax.set_ylabel("By (mT)")
    ax.set_zlabel("Bz (mT)")
    ax.view_init(**VIEW)

    fig.tight_layout()
    fig.savefig(OUTPUT_PATH, dpi=200, bbox_inches="tight")
    print(f"saved {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
