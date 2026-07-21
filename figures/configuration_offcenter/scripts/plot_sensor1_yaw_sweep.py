#!/usr/bin/env python3
"""Sensor 1's magnetic-field vector with the magnet raised 10 mm.

Pitch and roll are zero. Sensor 1 rotates about the joint Z axis; every
plotted vector is expressed in Sensor 1's co-rotating frame.
"""

import os
import sys
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/magnetic_orientation_matplotlib")

SCRIPT_DIR = Path(__file__).resolve().parent
CONFIG_DIR = SCRIPT_DIR.parent
ROOT = CONFIG_DIR.parents[1]

sys.path.insert(0, str(ROOT))

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Line3DCollection

from offcenter_configuration import build_offcenter_magnet, sensor_1_field_at_yaw_mT
from sweep_plotting import add_rotation_triad, add_sweep_header, style_field_axes


OUTPUT_PATH = CONFIG_DIR / "plots" / "sensor1_yaw_sweep.png"

CURVE_STEP_DEG = 1
LINE_COLOR = "#2c5f9e"


def draw_direction_arrow(ax, fields, yaw_deg):
    index = int(yaw_deg / CURVE_STEP_DEG)
    tail = fields[index - 3]
    delta = fields[index + 3] - tail
    ax.quiver(
        *tail,
        *delta,
        color=LINE_COLOR,
        linewidth=2.2,
        arrow_length_ratio=0.65,
        length=1.0,
        normalize=False,
        zorder=6,
    )


def draw_field_vector(ax, field, yaw_deg, color, label_offset):
    ax.quiver(
        0.0,
        0.0,
        0.0,
        *field,
        color=color,
        linewidth=2.0,
        arrow_length_ratio=0.12,
        length=1.0,
        normalize=False,
        alpha=0.9,
        zorder=5,
    )
    label_position = field + np.asarray(label_offset)
    ax.text(
        *label_position,
        rf"$\psi={yaw_deg}^\circ$",
        color=color,
        fontsize=10,
        fontweight="bold" if yaw_deg == 0 else "normal",
        ha="center",
        va="center",
        zorder=8,
    )


def main():
    source = build_offcenter_magnet()
    yaws_curve = np.arange(0.0, 360.0, CURVE_STEP_DEG)
    fields_curve = np.array(
        [sensor_1_field_at_yaw_mT(source, yaw) for yaw in yaws_curve]
    )
    closed = np.vstack((fields_curve, fields_curve[:1]))

    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d")

    segments = np.stack((closed[:-1], closed[1:]), axis=1)
    line = Line3DCollection(segments, colors=LINE_COLOR, linewidths=2.7, zorder=4)
    ax.add_collection3d(line)

    # Cardinal samples make the curve read as a sweep of vectors, not only a path.
    cardinal_styles = (
        (0, "#c0392b", (0.45, -0.15, 0.35)),
        (90, "0.35", (0.25, 0.35, 0.30)),
        (180, "0.35", (-0.45, 0.05, 0.30)),
        (270, "0.35", (-0.25, -0.45, 0.30)),
    )
    for yaw_deg, color, offset in cardinal_styles:
        draw_field_vector(ax, fields_curve[yaw_deg], yaw_deg, color, offset)

    yaw_zero = fields_curve[0]
    ax.scatter(
        *yaw_zero,
        s=55,
        color="#c0392b",
        edgecolor="white",
        linewidth=0.8,
        depthshade=False,
        zorder=9,
    )
    for yaw_deg in (35, 125, 215, 305):
        draw_direction_arrow(ax, fields_curve, yaw_deg)

    ax.scatter(0.0, 0.0, 0.0, s=18, color="0.2", depthshade=False, zorder=7)

    style_field_axes(ax, span=5.7)
    ax.set_zlim(-3.15, 3.15)
    ax.set_box_aspect((1, 1, 0.72))
    fig.tight_layout()
    add_sweep_header(ax, "Yaw", "Z", r"\psi", "increases along arrows")
    add_rotation_triad(fig, "z", r"\psi")

    fig.savefig(OUTPUT_PATH, dpi=200, bbox_inches="tight")
    print(f"saved {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
