"""Shared 3D orientation plotting helpers."""

import numpy as np
from scipy.spatial.transform import Rotation


COLORS = ("#d62728", "#2ca02c", "#1f77b4")
AXES = ("X", "Y", "Z")


def configure_panel(axis, title):
    axis.set_title(title, fontsize=12, pad=12)
    axis.set(xlim=(-1.15, 1.15), ylim=(-1.15, 1.15), zlim=(-1.15, 1.15))
    axis.set_box_aspect((1, 1, 1))
    axis.set_xlabel("world X")
    axis.set_ylabel("world Y")
    axis.set_zlabel("world Z")
    axis.view_init(elev=25, azim=-55)
    axis.grid(True, alpha=0.25)

    for index, name in enumerate(AXES):
        endpoint = np.eye(3)[index]
        axis.plot((0, endpoint[0]), (0, endpoint[1]), (0, endpoint[2]),
                  color="#999999", linestyle="--", linewidth=1, alpha=0.7)
        axis.text(*(endpoint * 1.08), f"{name}w", color="#777777", fontsize=9)

    lines, labels = [], []
    for name, color in zip(AXES, COLORS):
        line, = axis.plot((0, 0), (0, 0), (0, 0), color=color, linewidth=3,
                          marker="o", markevery=[1], markersize=5,
                          label=f"body {name}")
        lines.append(line)
        labels.append(axis.text(0, 0, 0, name, color=color, fontsize=11,
                                fontweight="bold"))
    axis.legend(loc="upper left", fontsize=8, framealpha=0.9)
    return lines, labels


def set_orientation(artists, angles):
    lines, labels = artists
    endpoints = Rotation.from_euler("ZYX", angles, degrees=True).apply(np.eye(3))
    for line, label, endpoint in zip(lines, labels, endpoints):
        line.set_data_3d((0, endpoint[0]), (0, endpoint[1]), (0, endpoint[2]))
        label.set_position_3d(endpoint * 1.08)


def angle_title(name, angles):
    return (f"{name}\nYaw {angles[0]:7.2f}°   Pitch {angles[1]:7.2f}°   "
            f"Roll {angles[2]:7.2f}°")
