#!/usr/bin/env python3
"""Create a 16:9 presentation flowchart of the estimation pipeline."""

import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/magnetic_orientation_matplotlib")

import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Rectangle


OUTPUT_DIR = Path(__file__).resolve().parent
PNG_PATH = OUTPUT_DIR / "system_pipeline.png"
SVG_PATH = OUTPUT_DIR / "system_pipeline.svg"

BACKGROUND = "#f7f8fa"
TEXT = "#20252b"
MUTED = "#5d6670"
OFFLINE = "#168a7a"
OFFLINE_LIGHT = "#edf7f5"
RUNTIME = "#2864a0"
RUNTIME_LIGHT = "#eef4fa"
LOOKUP = "#c58b18"
LOOKUP_LIGHT = "#fff8e8"
ARROW = "#77818b"


def add_node(ax, x, y, width, height, title, lines, accent, fill="white"):
    box = FancyBboxPatch(
        (x, y),
        width,
        height,
        boxstyle="round,pad=0.006,rounding_size=0.008",
        linewidth=1.5,
        edgecolor=accent,
        facecolor=fill,
        zorder=3,
    )
    ax.add_patch(box)
    ax.add_patch(Rectangle((x, y), 0.008, height, color=accent, zorder=4))
    ax.text(
        x + 0.016,
        y + height - 0.030,
        title,
        fontsize=11.5,
        fontweight="bold",
        color=TEXT,
        ha="left",
        va="top",
        zorder=5,
    )
    for index, line in enumerate(lines):
        ax.text(
            x + 0.016,
            y + height - 0.070 - index * 0.030,
            line,
            fontsize=8.8,
            color=MUTED,
            ha="left",
            va="top",
            zorder=5,
        )
    return (x, y, width, height)


def add_arrow(ax, left_box, right_box, color=ARROW):
    lx, ly, lw, lh = left_box
    rx, ry, _rw, rh = right_box
    arrow = FancyArrowPatch(
        (lx + lw + 0.006, ly + lh / 2.0),
        (rx - 0.006, ry + rh / 2.0),
        arrowstyle="-|>",
        mutation_scale=13,
        linewidth=1.8,
        color=color,
        zorder=6,
    )
    ax.add_patch(arrow)


def add_lane_label(ax, y, color, title, subtitle):
    ax.text(0.053, y + 0.026, title, fontsize=13, fontweight="bold",
            color=color, ha="left", va="center")
    if subtitle:
        ax.text(0.053, y - 0.010, subtitle, fontsize=8.5, fontweight="bold",
                color=MUTED, ha="left", va="center")


def main():
    fig = plt.figure(figsize=(16, 9), facecolor=BACKGROUND)
    ax = fig.add_axes((0, 0, 1, 1))
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")

    ax.add_patch(Rectangle((0.035, 0.620), 0.93, 0.280,
                           facecolor=OFFLINE_LIGHT, edgecolor="none", zorder=0))
    ax.add_patch(Rectangle((0.035, 0.160), 0.93, 0.280,
                           facecolor=RUNTIME_LIGHT, edgecolor="none", zorder=0))
    ax.add_patch(Rectangle((0.035, 0.620), 0.008, 0.280,
                           facecolor=OFFLINE, edgecolor="none", zorder=1))
    ax.add_patch(Rectangle((0.035, 0.160), 0.008, 0.280,
                           facecolor=RUNTIME, edgecolor="none", zorder=1))

    add_lane_label(ax, 0.772, OFFLINE, "OFFLINE", "CALIBRATE + FIT")
    add_lane_label(ax, 0.312, RUNTIME, "REAL TIME", "")

    width = 0.135
    height = 0.155
    xs = (0.165, 0.325, 0.485, 0.645, 0.805)
    offline_y = 0.682
    runtime_y = 0.222

    offline_nodes = [
        add_node(ax, xs[0], offline_y, width, height, "Measure setup",
                 ("sensor + magnet positions", "magnet size + material"),
                 OFFLINE),
        add_node(ax, xs[1], offline_y, width, height, "Baseline model",
                 ("MagPy finite cylinder", "initial field prediction"),
                 OFFLINE),
        add_node(ax, xs[2], offline_y, width, height, "Record sweeps",
                 ("6 magnetic channels", "IMU angles = ground truth"),
                 OFFLINE),
        add_node(ax, xs[3], offline_y, width, height, "Optimize + fit",
                 (r"minimize $|B_{sim}-B_{meas}|$", "update simulated geometry"),
                 OFFLINE),
        add_node(ax, xs[4], offline_y, width, height, "Dense lookup table",
                 ("fitted model over pose grid", "6-D field <-> 3 angles"),
                 LOOKUP, LOOKUP_LIGHT),
    ]
    for left, right in zip(offline_nodes[:-1], offline_nodes[1:]):
        add_arrow(ax, left, right, OFFLINE)

    runtime_width = 0.155
    runtime_xs = (0.180, 0.390, 0.600, 0.810)
    runtime_nodes = [
        add_node(ax, runtime_xs[0], runtime_y, runtime_width, height, "Read sensors",
                 (r"$B_x, B_y, B_z$ from S1 + S2", "one 6-D measurement"),
                 RUNTIME),
        add_node(ax, runtime_xs[1], runtime_y, runtime_width, height, "Correct + filter",
                 ("subtract sensor offsets", r"EMA noise filter, $\alpha=0.2$"),
                 RUNTIME),
        add_node(ax, runtime_xs[2], runtime_y, runtime_width, height, "KD-tree search",
                 ("closest 6-D lookup point", "nearest-neighbor query"),
                 RUNTIME),
        add_node(ax, runtime_xs[3], runtime_y, runtime_width, height, "Return angles",
                 ("yaw / pitch / roll",),
                 RUNTIME),
    ]
    for left, right in zip(runtime_nodes[:-1], runtime_nodes[1:]):
        add_arrow(ax, left, right, RUNTIME)

    lookup = offline_nodes[-1]
    query = runtime_nodes[-2]
    lx, ly, lw, _lh = lookup
    qx, qy, qw, qh = query
    connector = FancyArrowPatch(
        (lx + lw / 2.0, ly - 0.006),
        (qx + qw / 2.0, qy + qh + 0.006),
        connectionstyle="arc3,rad=0.22",
        arrowstyle="-|>",
        mutation_scale=13,
        linewidth=2.0,
        color=LOOKUP,
        zorder=7,
    )
    ax.add_patch(connector)
    ax.text(0.775, 0.530, "load + index once", fontsize=9.2,
            fontweight="bold", color=LOOKUP, ha="center", va="center",
            bbox=dict(facecolor=BACKGROUND, edgecolor="none", pad=2.0), zorder=8)

    fig.savefig(PNG_PATH, dpi=180, facecolor=BACKGROUND)
    fig.savefig(SVG_PATH, facecolor=BACKGROUND)
    print(f"saved {PNG_PATH}")
    print(f"saved {SVG_PATH}")


if __name__ == "__main__":
    main()
