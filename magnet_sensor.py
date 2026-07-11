#!/usr/bin/env python
"""B in the sensor's own frame as the shell YAWS about z (centered magnet).

The sensor starts at (0,-2,-1) and rides the shell through a full yaw turn:
its position AND its axes rotate together, so it reads
    B_sensor = Rz^-1 . B_world(Rz . p0).
Yawing moves the sensor to places with different field, so both the direction
and the strength of the reading change -- yaw is measurable (see README Q2).

Opens an interactive 3D view of the reading vector, colored by |B|.
"""
import numpy as np
import magpylib as magpy
import plotly.graph_objects as go
from scipy.spatial.transform import Rotation as Rot

magnet = magpy.magnet.Cylinder(polarization=(0, 0, 1.2), dimension=(3, 2))
magnet.rotate_from_angax(90, "y", anchor=(0, 0, 0))

p0 = np.array([0.0, -2.0, -1.0])                     # sensor start (mm)
angles = np.linspace(0, 360, 73)                     # full turn, 5 deg steps
Rz = Rot.from_euler("z", angles, degrees=True)

pos = Rz.apply(p0)
B = np.einsum("nij,nj->ni", Rz.inv().as_matrix(), magnet.getB(pos))   # sensor frame
mag = np.linalg.norm(B, axis=1)
cmin, cmax = mag.min(), mag.max()

# B arrows from the sensor (origin), colored by |B|; every 2nd sample.
L = 3.0
tip = B / cmax * L
sub = slice(None, None, 2)

fig = go.Figure()
shaft = np.stack([np.zeros_like(tip[sub]), tip[sub], np.full_like(tip[sub], np.nan)],
                 axis=1).reshape(-1, 3)
fig.add_trace(go.Scatter3d(x=shaft[:, 0], y=shaft[:, 1], z=shaft[:, 2], mode="lines",
                           line=dict(color="lightgray", width=2), hoverinfo="skip"))
fig.add_trace(go.Cone(
    x=tip[sub, 0], y=tip[sub, 1], z=tip[sub, 2],
    u=B[sub, 0], v=B[sub, 1], w=B[sub, 2],
    anchor="tip", sizemode="absolute", sizeref=0.9,
    colorscale="Viridis", cmin=cmin, cmax=cmax,
    customdata=np.stack([angles[sub], mag[sub]], axis=1),
    hovertemplate=("angle = %{customdata[0]:.0f}°<br>"
                   "B = (%{u:.3g}, %{v:.3g}, %{w:.3g}) T<br>"
                   "<b>|B| = %{customdata[1]:.4g} T</b><extra></extra>"),
    colorbar=dict(title="|B| (T)", len=0.6)))
fig.add_trace(go.Scatter3d(x=tip[:, 0], y=tip[:, 1], z=tip[:, 2], mode="lines",
                           line=dict(color=mag, colorscale="Viridis",
                                     cmin=cmin, cmax=cmax, width=5), hoverinfo="skip"))
fig.add_trace(go.Scatter3d(x=[0], y=[0], z=[0], mode="markers+text",
                           marker=dict(size=5, color="black"), text=["sensor"],
                           textposition="bottom center", hoverinfo="skip"))
A = L * 1.25
for vec, lab in ([A, 0, 0], "x_s"), ([0, A, 0], "y_s"), ([0, 0, A], "z_s"):
    fig.add_trace(go.Scatter3d(x=[0, vec[0]], y=[0, vec[1]], z=[0, vec[2]],
                               mode="lines+text", line=dict(color="dimgray", width=2),
                               text=["", lab], textfont=dict(color="dimgray"),
                               hoverinfo="skip"))
lim = A * 1.05
fig.update_layout(
    title="Sensor orbiting about z (start 0,-2,-1) — B in the sensor frame",
    showlegend=False,
    scene=dict(xaxis=dict(title="x_s (mm)", range=[-lim, lim]),
               yaxis=dict(title="y_s (mm)", range=[-lim, lim]),
               zaxis=dict(title="z_s (mm)", range=[-lim, lim]),
               aspectmode="cube", camera=dict(eye=dict(x=0.9, y=-0.9, z=1.5))),
    margin=dict(l=0, r=0, t=40, b=0))
fig.show()
