#!/usr/bin/env python3
"""Plot MAE and error variability for the four verification recordings."""

import csv
import json
import os
from pathlib import Path
import sys

os.environ.setdefault("MPLCONFIGDIR", "/tmp/magnetic_orientation_matplotlib")

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import matplotlib.pyplot as plt
import numpy as np

from magnetic_pose.config import load_sensor_offsets
from magnetic_pose.filtering import DEFAULT_EMA_ALPHA, ExponentialMovingAverage
from magnetic_pose.lookup import PoseEstimator


OUTPUT_DIR = Path(__file__).resolve().parent
PNG_PATH = OUTPUT_DIR / "verification_mae_std.png"
SVG_PATH = OUTPUT_DIR / "verification_mae_std.svg"
METRICS_PATH = OUTPUT_DIR / "verification_mae_std.json"

SCENARIOS = (
    ("yaw", "Yaw sweep"),
    ("pitch", "Pitch sweep"),
    ("roll", "Roll sweep"),
    ("random", "Random motion"),
)
AXES = ("Yaw", "Pitch", "Roll")
COLORS = ("#2c6aa6", "#2f8f5b", "#c84c3a")
FIELD_COLUMNS = (
    "S1_Bx_raw_mT", "S1_By_raw_mT", "S1_Bz_raw_mT",
    "S2_Bx_raw_mT", "S2_By_raw_mT", "S2_Bz_raw_mT",
)


def wrap180(values):
    return (values + 180.0) % 360.0 - 180.0


def estimate_recording(path, estimator, offsets):
    _times, truth, estimates = process_recording(path, estimator, offsets)
    errors = estimates - truth
    errors[:, 0] = wrap180(errors[:, 0])
    return errors


def process_recording(path, estimator, offsets):
    with Path(path).open(newline="") as source:
        rows = list(csv.DictReader(source))
    times = np.array([float(row["t_s"]) for row in rows])
    times -= times[0]
    truth = np.array([
        [float(row[f"imu_{axis.lower()}_deg"]) for axis in AXES]
        for row in rows
    ])
    raw_fields = np.array([
        [float(row[column]) for column in FIELD_COLUMNS]
        for row in rows
    ])

    field_filter = ExponentialMovingAverage(DEFAULT_EMA_ALPHA)
    estimates = np.array([
        estimator.estimate(field_filter.update(field - offsets))["angles_deg"]
        for field in raw_fields
    ])
    return times, truth, estimates


def summarize(errors):
    absolute = np.abs(errors)
    return {
        "samples": int(len(errors)),
        "mae_deg": np.mean(absolute, axis=0).tolist(),
        "std_absolute_error_deg": np.std(absolute, axis=0, ddof=0).tolist(),
        "std_signed_error_deg": np.std(errors, axis=0, ddof=0).tolist(),
    }


def main():
    estimator = PoseEstimator()
    offsets = load_sensor_offsets()

    scenario_errors = []
    summaries = {}
    for file_stem, label in SCENARIOS:
        errors = estimate_recording(
            ROOT / "verification" / f"{file_stem}.csv", estimator, offsets
        )
        scenario_errors.append(errors)
        summaries[file_stem] = summarize(errors)

    pooled = np.vstack(scenario_errors)
    summaries["all_recordings"] = summarize(pooled)

    labels = [label for _file_stem, label in SCENARIOS] + ["All recordings"]
    maes = np.array([
        summaries[file_stem]["mae_deg"] for file_stem, _label in SCENARIOS
    ] + [summaries["all_recordings"]["mae_deg"]])
    stds = np.array([
        summaries[file_stem]["std_absolute_error_deg"]
        for file_stem, _label in SCENARIOS
    ] + [summaries["all_recordings"]["std_absolute_error_deg"]])

    fig, ax = plt.subplots(figsize=(13.333, 7.5), facecolor="#fafbfc")
    ax.set_facecolor("#fafbfc")
    x = np.arange(len(labels), dtype=float)
    width = 0.23

    ax.axvspan(3.55, 4.45, color="#eef1f4", zorder=0)
    for axis_index, (axis_name, color) in enumerate(zip(AXES, COLORS)):
        positions = x + (axis_index - 1) * width
        values = maes[:, axis_index]
        error_std = stds[:, axis_index]
        bars = ax.bar(
            positions,
            values,
            width=width * 0.88,
            color=color,
            edgecolor="white",
            linewidth=0.8,
            label=axis_name,
            zorder=3,
        )
        ax.errorbar(
            positions,
            values,
            yerr=error_std,
            fmt="none",
            ecolor="#303840",
            elinewidth=1.5,
            capsize=5,
            capthick=1.5,
            zorder=5,
        )
        for bar, value in zip(bars, values):
            ax.text(
                bar.get_x() + bar.get_width() / 2.0,
                max(0.10, value - 0.10),
                f"{value:.2f}",
                ha="center",
                va="top",
                fontsize=9.2,
                fontweight="bold",
                color="white",
                zorder=6,
            )

    ax.set_xlim(-0.55, len(labels) - 0.45)
    ax.set_ylim(0.0, 2.75)
    ax.set_xticks(x, labels, fontsize=11)
    ax.set_ylabel("Absolute angular error (deg)", fontsize=12, color="#303840")
    ax.tick_params(axis="y", colors="#59636d", labelsize=10)
    ax.tick_params(axis="x", length=0, pad=10)
    ax.grid(axis="y", color="#d9dee3", linewidth=0.9, zorder=0)
    for spine in ("top", "right", "left"):
        ax.spines[spine].set_visible(False)
    ax.spines["bottom"].set_color("#aab2ba")

    fig.text(
        0.065,
        0.935,
        "Error and standard deviation: magnetic estimate vs IMU ground truth",
        fontsize=22,
        fontweight="bold",
        color="#20252b",
        ha="left",
    )
    legend = ax.legend(
        loc="upper center",
        bbox_to_anchor=(0.5, 1.08),
        ncol=3,
        frameon=False,
        fontsize=11,
        handlelength=1.3,
        columnspacing=2.2,
    )
    for handle in legend.legend_handles:
        handle.set_linewidth(0)

    fig.subplots_adjust(left=0.085, right=0.975, bottom=0.11, top=0.84)

    payload = {
        "method": {
            "reference": "Xsens IMU, direct same-sample comparison",
            "offset_correction": True,
            "ema_alpha": DEFAULT_EMA_ALPHA,
            "estimator": "nearest-neighbor 6-D KD-tree lookup",
            "offline_alignment": False,
            "std_ddof": 0,
        },
        "axis_order": [axis.lower() for axis in AXES],
        "scenarios": summaries,
    }
    METRICS_PATH.write_text(json.dumps(payload, indent=2) + "\n")
    fig.savefig(PNG_PATH, dpi=200, facecolor=fig.get_facecolor())
    fig.savefig(SVG_PATH, facecolor=fig.get_facecolor())
    print(f"saved {PNG_PATH}")
    print(f"saved {SVG_PATH}")
    print(f"saved {METRICS_PATH}")


if __name__ == "__main__":
    main()
