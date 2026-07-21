"""Shared presentation styling for the off-center configuration sweeps."""

import numpy as np
from scipy.spatial.transform import Rotation


AXIS_COLORS = ("#d62728", "#2ca02c", "#1f77b4")
VIEW = dict(elev=45, azim=205)


def style_field_axes(ax, span):
    ax.set_xlim(-span, span)
    ax.set_ylim(-span, span)
    ax.set_zlim(-span, span)
    ax.set_box_aspect((1, 1, 1))
    ax.set_xlabel(r"$B_{x_s}$ (mT)", color=AXIS_COLORS[0], labelpad=8)
    ax.set_ylabel(r"$B_{y_s}$ (mT)", color=AXIS_COLORS[1], labelpad=8)
    ax.set_zlabel(r"$B_{z_s}$ (mT)", color=AXIS_COLORS[2], labelpad=7)
    ax.view_init(**VIEW)
    ax.grid(False)
    ax.xaxis.pane.set_alpha(0.0)
    ax.yaxis.pane.set_alpha(0.0)
    ax.zaxis.pane.set_alpha(0.0)
    ax.xaxis.line.set_color(AXIS_COLORS[0])
    ax.yaxis.line.set_color(AXIS_COLORS[1])
    ax.zaxis.line.set_color(AXIS_COLORS[2])
    ax.tick_params(axis="x", colors=AXIS_COLORS[0], labelsize=9, pad=1)
    ax.tick_params(axis="y", colors=AXIS_COLORS[1], labelsize=9, pad=1)
    ax.tick_params(axis="z", colors=AXIS_COLORS[2], labelsize=9, pad=1)


def add_sweep_header(ax, motion_name, axis_name, angle_symbol, result_text):
    """Add a concise plot label; the slides supply the explanatory context."""
    ax.text2D(
        0.03,
        0.96,
        f"{motion_name} sweep",
        transform=ax.transAxes,
        fontsize=13,
        fontweight="bold",
        color="0.15",
        va="top",
    )


def add_rotation_triad(fig, axis_name, angle_symbol):
    """Add a magnet-frame triad and positive rotation arrow."""
    inset = fig.add_axes((0.74, 0.72, 0.22, 0.22), projection="3d")
    basis = np.eye(3)
    for index, (name, color) in enumerate(zip("xyz", AXIS_COLORS)):
        direction = basis[index]
        inset.quiver(
            0.0,
            0.0,
            0.0,
            *direction,
            length=1.0,
            color=color,
            linewidth=2.0,
            arrow_length_ratio=0.18,
        )
        label_position = direction * 1.13
        inset.text(
            *label_position,
            name,
            color=color,
            fontsize=10,
            fontweight="bold",
            ha="center",
            va="center",
        )

    axis_index = "xyz".index(axis_name.lower())
    rotation_axis = basis[axis_index]
    start = basis[(axis_index + 1) % 3] * 0.62
    angles = np.radians(np.linspace(20.0, 310.0, 90))
    arc = Rotation.from_rotvec(angles[:, None] * rotation_axis).apply(start)
    inset.plot(*arc.T, color="0.2", linewidth=2.0, zorder=5)

    endpoint = arc[-1]
    tangent = np.cross(rotation_axis, endpoint)
    tangent /= np.linalg.norm(tangent)
    inset.quiver(
        *endpoint,
        *tangent,
        length=0.24,
        color="0.2",
        linewidth=2.0,
        arrow_length_ratio=0.65,
        zorder=6,
    )
    symbol_position = endpoint + tangent * 0.32
    inset.text(
        *symbol_position,
        rf"+${angle_symbol}$",
        color="0.2",
        fontsize=9,
        fontweight="bold",
        ha="center",
        va="center",
    )

    inset.set_xlim(-0.9, 1.25)
    inset.set_ylim(-0.9, 1.25)
    inset.set_zlim(-0.9, 1.25)
    inset.set_box_aspect((1, 1, 1))
    inset.view_init(**VIEW)
    inset.set_axis_off()
