#!/usr/bin/env python
"""B read in a 3-axis sensor's frame as the sensor is pitched in place.

Magnet 3x2 mm at the origin, N -> +x. Sensor is FIXED at (0,-2,-1) and only
changes orientation, rotating about y from -30 to +30 deg. Position is fixed, so
the true field is constant; the chip's axes turn, so it reads
    B_sensor(t) = Ry(t)^-1 . B_world(p0).
=> |B| stays constant, only the direction rotates (here within the x-z plane, so
By ~ 0). Opens an interactive 3D view of B in the sensor frame.
"""
import numpy as np
import magpylib as magpy
import plotly.graph_objects as go
from scipy.spatial.transform import Rotation as Rot

magnet = magpy.magnet.Cylinder(polarization=(0, 0, 1.2), dimension=(3, 2))
magnet.rotate_from_angax(90, "y", anchor=(0, 0, 0))

p0 = np.array([0.0, -2.0, -1.0])                     # sensor position (mm), fixed
angles = np.linspace(-30, 30, 61)                    # -30..+30 deg, 1 deg steps
Ry = Rot.from_euler("y", angles, degrees=True)

Bw = magnet.getB(p0)                                 # constant world field at p0
B = np.einsum("nij,j->ni", Ry.inv().as_matrix(), Bw) # sensor frame
mag = np.linalg.norm(B, axis=1)

# B arrows from the sensor (origin). |B| is constant, so color encodes pitch
# angle (the informative variable); every 4th sample.
L = 3.0
tip = B / mag.mean() * L
sub = slice(None, None, 4)
b_s, t_s, a_s = B[sub], tip[sub], angles[sub]

fig = go.Figure()
shaft = np.stack([np.zeros_like(t_s), t_s, np.full_like(t_s, np.nan)], axis=1).reshape(-1, 3)
fig.add_trace(go.Scatter3d(x=shaft[:, 0], y=shaft[:, 1], z=shaft[:, 2], mode="lines",
                           line=dict(color="lightgray", width=2), hoverinfo="skip"))
fig.add_trace(go.Cone(
    x=t_s[:, 0], y=t_s[:, 1], z=t_s[:, 2], u=b_s[:, 0], v=b_s[:, 1], w=b_s[:, 2],
    anchor="tip", sizemode="absolute", sizeref=0.9,
    colorscale="Viridis", cmin=angles.min(), cmax=angles.max(),
    customdata=np.stack([a_s, mag[sub] * 1e3], axis=1),
    hovertemplate=("pitch = %{customdata[0]:.0f}°<br>"
                   "B = (%{u:.3g}, %{v:.3g}, %{w:.3g}) T<br>"
                   "<b>|B| = %{customdata[1]:.2f} mT</b><extra></extra>"),
    colorbar=dict(title="pitch (°)", len=0.6)))
fig.add_trace(go.Scatter3d(x=tip[:, 0], y=tip[:, 1], z=tip[:, 2], mode="lines",
                           line=dict(color=angles, colorscale="Viridis",
                                     cmin=angles.min(), cmax=angles.max(), width=5),
                           hoverinfo="skip"))
fig.add_trace(go.Scatter3d(
    x=tip[[0, -1], 0], y=tip[[0, -1], 1], z=tip[[0, -1], 2],
    mode="markers+text", marker=dict(size=4, color=["#440154", "#fde725"]),
    text=["-30°", "+30°"], textposition="top center", textfont=dict(size=11),
    hoverinfo="skip"))
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
    title="Sensor pitched in place about y (fixed at 0,-2,-1, -30°→+30°) — B in the sensor frame",
    showlegend=False,
    scene=dict(xaxis=dict(title="x_s (mm)", range=[-lim, lim]),
               yaxis=dict(title="y_s (mm)", range=[-lim, lim]),
               zaxis=dict(title="z_s (mm)", range=[-lim, lim]),
               aspectmode="cube", camera=dict(eye=dict(x=0.4, y=-1.8, z=0.6))),
    margin=dict(l=0, r=0, t=40, b=0))
fig.show()
