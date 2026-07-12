#!/usr/bin/env python
"""Generate the README figures (static 2D previews of the 3D vector views).

Each panel shows the reading of ONE sensor in its own frame while ONE angle
sweeps and the other two stay at zero: every arrow is the (Bx,By,Bz) vector
the chip reports at one sweep step, colored from dark (sweep start) to bright
(sweep end). The title states how much the reading really changed:
the direction change between the first and last arrow, and the |B| range.

Writes PNG files into figures/.   Run:  .venv/bin/python make_figures.py
"""
import numpy as np
import magpylib as magpy
import matplotlib
matplotlib.use("Agg")                      # render to files, no window
import matplotlib.pyplot as plt
from matplotlib import cm
from scipy.spatial.transform import Rotation


def make_magnet(offset):
    m = magpy.magnet.Cylinder(polarization=(0, 0, 1.2), dimension=(10, 5))
    m.rotate_from_angax(90, "y", anchor=(0, 0, 0))    # N faces +x
    m.position = offset
    return m


def reading(magnet, sensor_home, yaw, pitch, roll):
    """The 3 numbers the chip reports at this pose (same model as the scripts)."""
    rotation = Rotation.from_euler("zyx", [yaw, pitch, roll], degrees=True)
    position = rotation.apply(sensor_home)
    field_world = magnet.getB(position)
    return rotation.inv().apply(field_world)


def draw_sweep(ax, magnet, sensor_home, sweep_name, amplitude, n_steps=13):
    """One 3D panel: the reading vector while one angle sweeps -amp..+amp."""
    angles = np.linspace(-amplitude, amplitude, n_steps)
    vectors = []
    for a in angles:
        pose = {"yaw": (a, 0, 0), "pitch": (0, a, 0), "roll": (0, 0, a)}[sweep_name]
        vectors.append(reading(magnet, sensor_home, *pose))
    vectors = np.array(vectors)

    # scale arrows so the typical one has length 1
    scale = np.linalg.norm(vectors, axis=1).mean()
    tips = vectors / scale

    for a, tip in zip(angles, tips):
        color = cm.viridis((a + amplitude) / (2 * amplitude))
        ax.plot([0, tip[0]], [0, tip[1]], [0, tip[2]], color=color, lw=2)
        ax.scatter(*tip, color=color, s=18)
    ax.text(*tips[0] * 1.15, f"{angles[0]:+.0f}°", fontsize=8, ha="center")
    ax.text(*tips[-1] * 1.15, f"{angles[-1]:+.0f}°", fontsize=8, ha="center")

    # how much did the reading really change over the sweep?
    # (largest difference between any two readings, in mT; noise is 0.1 mT)
    total_change = 0.0
    for v in vectors:
        for w in vectors:
            total_change = max(total_change, np.linalg.norm(v - w) * 1e3)
    strengths = np.linalg.norm(vectors, axis=1) * 1e3
    ax.set_title(f"{sweep_name} {angles[0]:+.0f}°..{angles[-1]:+.0f}°\n"
                 f"reading changes by {total_change:.1f} mT,  "
                 f"|B| {strengths.min():.0f}..{strengths.max():.0f} mT",
                 fontsize=9)

    lim = 1.35
    ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-lim, lim)
    ax.set_box_aspect([1, 1, 1])
    ax.set_xlabel("x_s", fontsize=8); ax.set_ylabel("y_s", fontsize=8)
    ax.set_zlabel("z_s", fontsize=8)
    ax.tick_params(labelsize=6)


def make_figure(panels, filename, suptitle):
    fig = plt.figure(figsize=(4.2 * len(panels), 4.4))
    for i, (magnet, sensor, sweep, amplitude) in enumerate(panels, start=1):
        ax = fig.add_subplot(1, len(panels), i, projection="3d")
        draw_sweep(ax, magnet, sensor, sweep, amplitude)
    fig.suptitle(suptitle, fontsize=11)
    fig.tight_layout()
    fig.savefig(f"figures/{filename}", dpi=150)
    plt.close(fig)
    print("wrote figures/" + filename)


if __name__ == "__main__":
    import os
    os.makedirs("figures", exist_ok=True)

    centered = make_magnet((0, 0, 0))
    offset = make_magnet((0, 3.0, 0))
    side_sensor = np.array([0.0, -13.4, -6.7])         # the Q1/Q2 sensor (15 mm)
    pole_sensor = np.array([0.0, 0.0, -15.0])          # a final-design sensor

    # Figure 1 (Q1+Q2): centered magnet -- yaw & pitch fan out, roll is frozen
    make_figure([(centered, side_sensor, "yaw", 120),
                 (centered, side_sensor, "pitch", 25),
                 (centered, side_sensor, "roll", 25)],
                "sweeps_centered_magnet.png",
                "Centered magnet, one sensor at (0,-13.4,-6.7): "
                "yaw and pitch change the reading, roll does not")

    # Figure 2 (Q3): the off-center trick makes roll visible.
    # Drawn as 2D component curves, where "frozen vs changing" is unmistakable.
    fig, axes = plt.subplots(1, 2, figsize=(9, 3.6), sharey=True)
    rolls = np.linspace(-25, 25, 51)
    for ax, magnet, label in [(axes[0], centered, "magnet centered"),
                              (axes[1], offset, "magnet 3 mm off-center")]:
        curves = np.array([reading(magnet, side_sensor, 0, 0, r) for r in rolls]) * 1e3
        for i, name in enumerate(["Bx", "By", "Bz"]):
            ax.plot(rolls, curves[:, i], lw=2, label=name)
        change = np.linalg.norm(curves.max(axis=0) - curves.min(axis=0))
        ax.set_title(f"{label}\nreading changes by {change:.1f} mT", fontsize=10)
        ax.set_xlabel("roll (°)")
        ax.grid(alpha=0.3)
    axes[0].set_ylabel("reading (mT)")
    axes[0].legend(fontsize=8)
    fig.suptitle("The same roll sweep, one sensor at (0,-13.4,-6.7): "
                 "centered = frozen, off-center = visible", fontsize=11)
    fig.tight_layout()
    fig.savefig("figures/roll_centered_vs_offset.png", dpi=150)
    plt.close(fig)
    print("wrote figures/roll_centered_vs_offset.png")

    # Figure 3 (Q4): final design -- every rotation changes the reading
    make_figure([(offset, pole_sensor, "yaw", 120),
                 (offset, pole_sensor, "pitch", 25),
                 (offset, pole_sensor, "roll", 25)],
                "sweeps_final_design.png",
                "Final design (off-center magnet, sensor at the -z pole): "
                "all three rotations are visible")
