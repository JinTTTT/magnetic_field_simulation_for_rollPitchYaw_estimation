#!/usr/bin/env python3
"""Ideal centered magnet + Sensor 1 configuration, for slides.

Draws the centered, untilted magnet and Sensor 1 at the same height, with the
sensor's own x/y/z axes drawn at the box center. This is the ideal reference
configuration before introducing an off-center magnet.
"""

import itertools
import os
import sys
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/magnetic_orientation_matplotlib")

SCRIPT_DIR = Path(__file__).resolve().parent
IDEAL_DIR = SCRIPT_DIR.parent
FIGURES_DIR = IDEAL_DIR.parent
ROOT = FIGURES_DIR.parent

sys.path.insert(0, str(FIGURES_DIR))
sys.path.insert(0, str(ROOT))

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from magnetic_pose.plotting import AXES, COLORS
from ideal_configuration import IDEAL_MODEL, SENSOR_1_HOME_MM
import plot_field_lines as field_plot
from plot_field_lines import (
    COLOR_VMAX_MT,
    COLOR_VMIN_MT,
    CMAP_NAME,
    compute_field_lines,
    draw_disc,
    draw_field_lines,
    magnet_geometry,
)

OUTPUT_PATH = IDEAL_DIR / "plots" / "configuration_ideal.png"

BOX_SIZE_MM = np.array([6.0, 6.0, 2.0])
AXIS_LENGTH_MM = 9.0
BOUND_HALF_RANGE_MM = 34.0
VIEW = dict(elev=45, azim=205)

# Match the off-center schematic's clean, quick-to-render field-line density.
field_plot.RADIUS_FRACS = (0.40, 0.75)
field_plot.NUM_AZIMUTH = 8
field_plot.MAX_STEPS = 700
field_plot.MAX_ARC_LENGTH_MM = 50.0


def draw_box(ax, center_mm, size_mm, facecolor="#dcdcdc", edgecolor="0.35"):
    half = size_mm / 2.0
    xs, ys, zs = ((center_mm[i] - half[i], center_mm[i] + half[i]) for i in range(3))
    corners = np.array(list(itertools.product(xs, ys, zs)))
    # corners order: (x0,y0,z0) (x0,y0,z1) (x0,y1,z0) (x0,y1,z1)
    #                (x1,y0,z0) (x1,y0,z1) (x1,y1,z0) (x1,y1,z1)
    faces = [
        [corners[0], corners[1], corners[3], corners[2]],
        [corners[4], corners[5], corners[7], corners[6]],
        [corners[0], corners[1], corners[5], corners[4]],
        [corners[2], corners[3], corners[7], corners[6]],
        [corners[0], corners[2], corners[6], corners[4]],
        [corners[1], corners[3], corners[7], corners[5]],
    ]
    box = Poly3DCollection(faces, facecolor=facecolor, edgecolor=edgecolor,
                           linewidths=0.7, alpha=0.95, zorder=4)
    ax.add_collection3d(box)


def draw_axes_triad(ax, center_mm, length_mm):
    for axis_index, (name, color) in enumerate(zip(AXES, COLORS)):
        direction = np.eye(3)[axis_index]
        tip = center_mm + direction * length_mm
        ax.quiver(*center_mm, *direction, length=length_mm, color=color,
                  linewidth=2.0, arrow_length_ratio=0.18, zorder=6)
        ax.text(*(tip * 1.0 + direction * 1.2), name.lower(), color=color,
                fontsize=12, fontweight="bold", ha="center", va="center", zorder=7)


def main():
    geometry = magnet_geometry(IDEAL_MODEL)
    center_mm = geometry["center_mm"]
    sensor_mm = SENSOR_1_HOME_MM

    paths, mags = compute_field_lines(IDEAL_MODEL, geometry)
    norm = plt.Normalize(vmin=COLOR_VMIN_MT, vmax=COLOR_VMAX_MT, clip=True)
    cmap = plt.get_cmap(CMAP_NAME)

    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d")

    draw_field_lines(ax, paths, mags, cmap, norm)
    draw_disc(ax, geometry)
    draw_box(ax, sensor_mm, BOX_SIZE_MM)
    draw_axes_triad(ax, sensor_mm, AXIS_LENGTH_MM)

    label_pos = sensor_mm + np.array([0.0, 0.0, BOX_SIZE_MM[2] / 2.0 + 5.0])
    ax.text(*label_pos, "Sensor 1", color="0.15", fontsize=12, fontweight="bold",
            ha="center", va="center", zorder=7)

    plot_center = center_mm
    span = BOUND_HALF_RANGE_MM
    ax.set_xlim(plot_center[0] - span, plot_center[0] + span)
    ax.set_ylim(plot_center[1] - span, plot_center[1] + span)
    ax.set_zlim(plot_center[2] - span, plot_center[2] + span)
    ax.set_box_aspect((1, 1, 1))
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    ax.view_init(**VIEW)
    ax.grid(False)
    ax.xaxis.pane.set_alpha(0.0)
    ax.yaxis.pane.set_alpha(0.0)
    ax.zaxis.pane.set_alpha(0.0)

    ax.text2D(
        0.03,
        0.96,
        "Ideal configuration",
        transform=ax.transAxes,
        fontsize=13,
        fontweight="bold",
        color="0.15",
        va="top",
    )
    ax.text2D(
        0.03,
        0.91,
        "Magnet centered on joint   Sensor 1 at same height",
        transform=ax.transAxes,
        fontsize=10,
        color="0.35",
        va="top",
    )

    fig.tight_layout()
    fig.savefig(OUTPUT_PATH, dpi=200, bbox_inches="tight")
    print(f"saved {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
