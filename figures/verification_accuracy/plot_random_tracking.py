#!/usr/bin/env python3
"""Plot random-recording magnetic estimates against the Xsens ground truth."""

import os
from pathlib import Path
import sys

os.environ.setdefault("MPLCONFIGDIR", "/tmp/magnetic_orientation_matplotlib")

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

from magnetic_pose.config import load_sensor_offsets
from magnetic_pose.lookup import PoseEstimator
from plot_verification_accuracy import process_recording, wrap180


OUTPUT_DIR = Path(__file__).resolve().parent
PNG_PATH = OUTPUT_DIR / "random_tracking.png"
SVG_PATH = OUTPUT_DIR / "random_tracking.svg"

AXES = ("Yaw", "Pitch", "Roll")
GROUND_TRUTH_COLOR = "#20252b"
ESTIMATE_COLOR = "#2c6aa6"


def main():
    estimator = PoseEstimator()
    offsets = load_sensor_offsets()
    times, truth, estimates = process_recording(
        ROOT / "verification" / "random.csv", estimator, offsets
    )

    errors = estimates - truth
    errors[:, 0] = wrap180(errors[:, 0])
    maes = np.mean(np.abs(errors), axis=0)

    fig, subplot_axes = plt.subplots(
        3,
        1,
        figsize=(13.333, 7.5),
        sharex=True,
        facecolor="#fafbfc",
    )
    for index, (axis, axis_name) in enumerate(zip(subplot_axes, AXES)):
        axis.set_facecolor("#fafbfc")
        axis.plot(
            times,
            truth[:, index],
            color=GROUND_TRUTH_COLOR,
            linewidth=2.0,
            label="IMU ground truth",
            zorder=2,
        )
        axis.plot(
            times,
            estimates[:, index],
            color=ESTIMATE_COLOR,
            linewidth=1.45,
            alpha=0.95,
            label="Magnetic estimate",
            zorder=3,
        )
        axis.set_ylabel(f"{axis_name} (deg)", fontsize=11, color="#303840")
        axis.grid(axis="both", color="#dce1e5", linewidth=0.8, zorder=0)
        axis.tick_params(axis="both", labelsize=9.5, colors="#59636d")
        axis.spines["top"].set_visible(False)
        axis.spines["right"].set_visible(False)
        axis.spines["left"].set_color("#aab2ba")
        axis.spines["bottom"].set_color("#aab2ba")
        axis.text(
            0.985,
            0.84,
            f"MAE {maes[index]:.2f} deg",
            transform=axis.transAxes,
            ha="right",
            va="top",
            fontsize=9.5,
            fontweight="bold",
            color=ESTIMATE_COLOR,
            bbox=dict(
                boxstyle="round,pad=0.28,rounding_size=0.15",
                facecolor="white",
                edgecolor=ESTIMATE_COLOR,
                linewidth=1.0,
            ),
            zorder=5,
        )

    subplot_axes[-1].set_xlabel("Time (s)", fontsize=11, color="#303840")
    subplot_axes[-1].set_xlim(times[0], times[-1])

    fig.text(
        0.070,
        0.945,
        "Random motion: magnetic estimate vs IMU ground truth",
        fontsize=23,
        fontweight="bold",
        color="#20252b",
        ha="left",
    )
    legend_handles = [
        Line2D([0], [0], color=GROUND_TRUTH_COLOR, linewidth=2.0,
               label="IMU ground truth"),
        Line2D([0], [0], color=ESTIMATE_COLOR, linewidth=1.8,
               label="Magnetic estimate"),
    ]
    fig.legend(
        handles=legend_handles,
        loc="upper right",
        bbox_to_anchor=(0.955, 0.952),
        ncol=2,
        frameon=False,
        fontsize=10.5,
        handlelength=2.3,
        columnspacing=2.0,
    )
    fig.subplots_adjust(left=0.090, right=0.970, bottom=0.095,
                        top=0.875, hspace=0.18)

    fig.savefig(PNG_PATH, dpi=200, facecolor=fig.get_facecolor())
    fig.savefig(SVG_PATH, facecolor=fig.get_facecolor())
    print(f"saved {PNG_PATH}")
    print(f"saved {SVG_PATH}")


if __name__ == "__main__":
    main()
