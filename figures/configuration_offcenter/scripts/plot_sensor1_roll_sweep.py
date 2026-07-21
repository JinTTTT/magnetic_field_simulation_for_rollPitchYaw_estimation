#!/usr/bin/env python3
"""Sensor 1's roll response with the magnet raised 10 mm.

Yaw and pitch are zero. Sensor 1 rotates about the joint X axis; every plotted
vector is expressed in Sensor 1's co-rotating frame.
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

from offcenter_configuration import build_offcenter_magnet, sensor_1_field_at_roll_mT
from sweep_plotting import add_rotation_triad, add_sweep_header, style_field_axes


OUTPUT_PATH = CONFIG_DIR / "plots" / "sensor1_roll_sweep.png"

CURVE_STEP_DEG = 1
LINE_COLOR = "#2c5f9e"


def draw_direction_arrow(ax, fields, roll_deg):
    index = int(roll_deg / CURVE_STEP_DEG)
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


def draw_field_vector(ax, field, label, color, label_offset):
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
        label,
        color=color,
        fontsize=10,
        fontweight="bold" if color == "#c0392b" else "normal",
        ha="center",
        va="center",
        zorder=8,
    )


def main():
    source = build_offcenter_magnet()
    rolls_curve = np.arange(0.0, 360.0, CURVE_STEP_DEG)
    fields_curve = np.array(
        [sensor_1_field_at_roll_mT(source, roll) for roll in rolls_curve]
    )
    closed = np.vstack((fields_curve, fields_curve[:1]))

    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d")

    segments = np.stack((closed[:-1], closed[1:]), axis=1)
    line = Line3DCollection(segments, colors=LINE_COLOR, linewidths=2.7, zorder=4)
    ax.add_collection3d(line)

    # Cardinal samples make the curve read as a sweep of vectors, not only a path.
    cardinal_styles = (
        (270, r"$\phi=270^\circ$", "0.35", (0.45, 0.0, 0.45)),
        (90, r"$\phi=90^\circ$", "0.35", (-0.20, 0.65, -0.20)),
        (0, r"$\phi=0^\circ=180^\circ$", "#c0392b", (0.35, -0.50, 0.50)),
    )
    for roll_deg, label, color, offset in cardinal_styles:
        draw_field_vector(ax, fields_curve[roll_deg], label, color, offset)

    roll_zero = fields_curve[0]
    ax.scatter(
        *roll_zero,
        s=55,
        color="#c0392b",
        edgecolor="white",
        linewidth=0.8,
        depthshade=False,
        zorder=9,
    )
    for roll_deg in (35, 135, 225, 315):
        draw_direction_arrow(ax, fields_curve, roll_deg)

    ax.scatter(0.0, 0.0, 0.0, s=18, color="0.2", depthshade=False, zorder=7)

    style_field_axes(ax, span=15.5)
    ax.set_xlim(-1.5, 15.5)
    ax.set_ylim(-5.0, 5.0)
    ax.set_zlim(-5.0, 5.0)
    ax.set_box_aspect((1.7, 1, 1))
    fig.tight_layout()
    add_sweep_header(ax, "Roll", "X", r"\phi", "increases along arrows")
    add_rotation_triad(fig, "x", r"\phi")

    fig.savefig(OUTPUT_PATH, dpi=200, bbox_inches="tight")
    print(f"saved {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
