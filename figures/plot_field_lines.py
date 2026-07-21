#!/usr/bin/env python3
"""3D magnetic field line plot of the fixed rig magnet, for slides.

Traces field lines leaving the magnet's North face and arcing back into its
South face, colored by local field strength (blue = weak, red = strong), and
draws the magnet as a labeled disc (N/S) at its fitted position and tilt.
"""

import os
import sys
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/magnetic_orientation_matplotlib")

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection
from scipy.integrate import solve_ivp

from magnetic_pose.model import build_magnet_source, load_model, magnet_orientation

OUTPUT_PATH = Path(__file__).resolve().parent / "magnetic_field_lines.png"

# Field-line seeding, in fractions of magnet radius / mm.
RADIUS_FRACS = (0.25, 0.45, 0.65, 0.85)
NUM_AZIMUTH = 10
SEED_OFFSET_MM = 0.6
MAX_ARC_LENGTH_MM = 55.0
MAX_STEPS = 2000
BBOX_HALF_RANGE_MM = 32.0

COLOR_VMIN_MT = 20.0
COLOR_VMAX_MT = 100.0
CMAP_NAME = "jet"


def magnet_geometry(model):
    magnet = model["magnet"]
    orientation = magnet_orientation(magnet)
    center_mm = np.asarray(magnet["center_mm"], dtype=float)
    radius_mm = magnet["dimension_mm"][0] / 2.0
    height_mm = magnet["dimension_mm"][1]
    n_hat = orientation.apply((0.0, 0.0, -1.0))
    n_hat /= np.linalg.norm(n_hat)
    return {
        "orientation": orientation,
        "center_mm": center_mm,
        "radius_mm": radius_mm,
        "height_mm": height_mm,
        "n_hat": n_hat,
    }


def seed_points_mm(geometry):
    orientation = geometry["orientation"]
    radius_mm = geometry["radius_mm"]
    height_mm = geometry["height_mm"]
    phis = np.linspace(0.0, 2 * np.pi, NUM_AZIMUTH, endpoint=False)
    seeds = []
    for frac in RADIUS_FRACS:
        for phi in phis:
            local = np.array((
                frac * radius_mm * np.cos(phi),
                frac * radius_mm * np.sin(phi),
                -height_mm / 2.0 - SEED_OFFSET_MM,
            ))
            seeds.append(orientation.apply(local))
    return np.asarray(seeds) + geometry["center_mm"]


def trace_field_line(source, start_mm, geometry):
    south_face_mm = geometry["center_mm"] - geometry["n_hat"] * geometry["height_mm"] / 2.0
    stop_radius_mm = geometry["radius_mm"] * 1.15

    def unit_field(_s, y_mm):
        point_m = np.asarray(y_mm).reshape(1, 3) / 1000.0
        b_t = np.asarray(source.getB(point_m)).reshape(3)
        norm = np.linalg.norm(b_t)
        if norm < 1e-15:
            return np.zeros(3)
        return b_t / norm

    def near_south(_s, y_mm):
        return np.linalg.norm(np.asarray(y_mm) - south_face_mm) - stop_radius_mm

    near_south.terminal = True
    near_south.direction = -1

    def leaves_bbox(_s, y_mm):
        return BBOX_HALF_RANGE_MM - np.max(np.abs(np.asarray(y_mm) - geometry["center_mm"]))

    leaves_bbox.terminal = True
    leaves_bbox.direction = -1

    solution = solve_ivp(
        unit_field,
        (0.0, MAX_ARC_LENGTH_MM),
        start_mm,
        max_step=MAX_ARC_LENGTH_MM / MAX_STEPS,
        events=(near_south, leaves_bbox),
        dense_output=False,
    )
    path_mm = solution.y.T
    points_m = path_mm / 1000.0
    fields_t = np.asarray(source.getB(points_m))
    magnitudes_mt = np.linalg.norm(fields_t, axis=1) * 1000.0
    return path_mm, magnitudes_mt


def draw_disc(ax, geometry):
    orientation = geometry["orientation"]
    center_mm = geometry["center_mm"]
    radius_mm = geometry["radius_mm"]
    height_mm = geometry["height_mm"]

    theta = np.linspace(0.0, 2 * np.pi, 48)
    circle_local = np.stack(
        (radius_mm * np.cos(theta), radius_mm * np.sin(theta)), axis=-1
    )

    def to_world(local_xy, local_z):
        local = np.column_stack((local_xy, np.full(len(local_xy), local_z)))
        return orientation.apply(local) + center_mm

    north_ring = to_world(circle_local, -height_mm / 2.0)
    south_ring = to_world(circle_local, height_mm / 2.0)

    side = Poly3DCollection(
        [
            [north_ring[i], north_ring[i + 1], south_ring[i + 1], south_ring[i]]
            for i in range(len(theta) - 1)
        ],
        facecolor="0.75", edgecolor="none", alpha=0.9, zorder=2,
    )
    ax.add_collection3d(side)

    north_cap = Poly3DCollection([north_ring], facecolor="#c0392b", edgecolor="0.3",
                                  linewidth=0.5, alpha=0.98, zorder=3)
    south_cap = Poly3DCollection([south_ring], facecolor="#2c5f9e", edgecolor="0.3",
                                  linewidth=0.5, alpha=0.98, zorder=3)
    ax.add_collection3d(north_cap)
    ax.add_collection3d(south_cap)


def compute_field_lines(model, geometry):
    source = build_magnet_source(model)
    paths, mags = [], []
    for seed in seed_points_mm(geometry):
        path_mm, magnitude_mt = trace_field_line(source, seed, geometry)
        if len(path_mm) >= 2:
            paths.append(path_mm)
            mags.append(magnitude_mt)
    return paths, mags


def draw_field_lines(ax, paths, mags, cmap, norm):
    for path_mm, magnitude_mt in zip(paths, mags):
        segments = np.stack((path_mm[:-1], path_mm[1:]), axis=1)
        seg_colors = cmap(norm((magnitude_mt[:-1] + magnitude_mt[1:]) / 2.0))
        line_collection = Line3DCollection(segments, colors=seg_colors, linewidths=1.3)
        ax.add_collection3d(line_collection)

        n_points = len(path_mm)
        for frac in (0.55,):
            idx = int(frac * (n_points - 2))
            idx = max(0, min(idx, n_points - 2))
            start = path_mm[idx]
            direction = path_mm[idx + 1] - path_mm[idx]
            direction_norm = np.linalg.norm(direction)
            if direction_norm < 1e-9:
                continue
            direction = direction / direction_norm
            color = cmap(norm(magnitude_mt[idx]))
            ax.quiver(*start, *direction, length=2.4, color=color,
                      linewidth=1.5, arrow_length_ratio=1.0, zorder=5)


def main():
    model = load_model()
    geometry = magnet_geometry(model)
    paths, mags = compute_field_lines(model, geometry)

    all_mags = np.concatenate(mags)
    norm = plt.Normalize(vmin=COLOR_VMIN_MT, vmax=COLOR_VMAX_MT, clip=True)
    cmap = plt.get_cmap(CMAP_NAME)

    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d")

    draw_field_lines(ax, paths, mags, cmap, norm)
    draw_disc(ax, geometry)

    center_mm = geometry["center_mm"]
    span = BBOX_HALF_RANGE_MM
    ax.set_xlim(center_mm[0] - span, center_mm[0] + span)
    ax.set_ylim(center_mm[1] - span, center_mm[1] + span)
    ax.set_zlim(center_mm[2] - span, center_mm[2] + span)
    ax.set_box_aspect((1, 1, 1))
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    ax.view_init(elev=45, azim=205)
    ax.grid(False)
    ax.xaxis.pane.set_alpha(0.0)
    ax.yaxis.pane.set_alpha(0.0)
    ax.zaxis.pane.set_alpha(0.0)

    mappable = plt.cm.ScalarMappable(norm=norm, cmap=cmap)
    mappable.set_array(all_mags)
    cbar = fig.colorbar(mappable, ax=ax, shrink=0.6, pad=0.08)
    cbar.set_label("|B| (mT)")

    fig.tight_layout()
    fig.savefig(OUTPUT_PATH, dpi=200, bbox_inches="tight")
    print(f"saved {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
