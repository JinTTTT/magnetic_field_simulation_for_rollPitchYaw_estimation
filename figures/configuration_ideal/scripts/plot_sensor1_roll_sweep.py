#!/usr/bin/env python3
"""Sensor 1's magnetic-field vector across an ideal 360 deg roll sweep.

Yaw and pitch are zero. Positive roll rotates Sensor 1 about the magnet's
symmetry axis X; every vector is expressed in Sensor 1's co-rotating frame.
"""

import os
import sys
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/magnetic_orientation_matplotlib")

SCRIPT_DIR = Path(__file__).resolve().parent
IDEAL_DIR = SCRIPT_DIR.parent
ROOT = IDEAL_DIR.parents[1]

sys.path.insert(0, str(ROOT))

import matplotlib.pyplot as plt
import numpy as np

from ideal_configuration import build_ideal_magnet, sensor_1_field_at_roll_mT
from sweep_plotting import add_rotation_triad, add_sweep_header, style_field_axes


OUTPUT_PATH = IDEAL_DIR / "plots" / "sensor1_roll_sweep.png"

CURVE_STEP_DEG = 1
VECTOR_COLOR = "#c0392b"


def main():
    source = build_ideal_magnet()
    rolls_curve = np.arange(0.0, 360.0, CURVE_STEP_DEG)
    fields_curve = np.array(
        [sensor_1_field_at_roll_mT(source, roll) for roll in rolls_curve]
    )
    field = fields_curve[0]

    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d")

    # All 360 samples coincide because roll is about the magnet's symmetry axis.
    ax.quiver(
        0.0,
        0.0,
        0.0,
        *field,
        color=VECTOR_COLOR,
        linewidth=3.0,
        arrow_length_ratio=0.12,
        length=1.0,
        normalize=False,
        zorder=6,
    )
    ax.scatter(0.0, 0.0, 0.0, s=18, color="0.2", depthshade=False, zorder=7)
    ax.scatter(
        *field,
        s=150,
        facecolor="white",
        edgecolor="#2c5f9e",
        linewidth=2.0,
        depthshade=False,
        zorder=8,
    )
    ax.scatter(
        *field,
        s=48,
        color=VECTOR_COLOR,
        depthshade=False,
        zorder=9,
    )
    label_position = field + np.array([0.25, 0.0, 0.30])
    ax.text(
        *label_position,
        r"$\phi=0^\circ=90^\circ=180^\circ=270^\circ$",
        color=VECTOR_COLOR,
        fontsize=10,
        fontweight="bold",
        ha="center",
        va="center",
        zorder=10,
    )

    style_field_axes(ax, span=3.4)
    fig.tight_layout()
    add_sweep_header(ax, "Roll", "X", r"\phi", "produces no change")
    add_rotation_triad(fig, "x", r"\phi")

    fig.savefig(OUTPUT_PATH, dpi=200, bbox_inches="tight")
    print(f"saved {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
