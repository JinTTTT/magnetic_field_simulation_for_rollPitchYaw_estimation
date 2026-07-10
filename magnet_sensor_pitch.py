#!/usr/bin/env python
"""
A 3-axis magnetic sensor (TLV493D-style) held at a FIXED position while it is
PITCHED (rotated about its y-axis), read out in the SENSOR's own frame.

  - Disc magnet 3 mm x 2 mm at the origin, N face -> +x, S face -> -x.
  - Sensor sits at (0, -2, -1) and does NOT move. It only changes orientation,
    rotating about the y-axis from -30 deg to +30 deg.
  - Because the sensor never moves, the true field in space (magnet/world frame)
    is CONSTANT in both magnitude and direction. But the chip's Hall plates turn
    with the sensor, so the reported vector rotates:
        B_sensor(t) = Ry(t)^-1 . B_world(p0),     B_world(p0) fixed.
    => |B| stays constant; only the direction changes with pitch.
    Here B_world ~ (-124, 0, 0) mT (points -x, into the S pole), so tilting about
    y spins the reading within the sensor's x-z plane (By stays ~0).

Outputs:
  - magnet_sensor_pitch_readings.csv : the sensor-frame vector at every pitch.
  - magnet_sensor_pitch.html         : interactive 3D view of B in the sensor
                                       frame. Arrows share a common origin (the
                                       sensor); tips trace an arc of constant
                                       radius (= constant |B|). Hover to read.

Run:  .venv/bin/python magnet_sensor_pitch.py
"""
import csv
import numpy as np
import magpylib as magpy
import plotly.graph_objects as go
from scipy.spatial.transform import Rotation as Rot

# 1. Magnet at origin, N -> +x.
magnet = magpy.magnet.Cylinder(polarization=(0, 0, 1.2), dimension=(3, 2))
magnet.rotate_from_angax(90, "y", anchor=(0, 0, 0))

# 2. Fixed sensor position; only its ORIENTATION pitches about y.
p0 = np.array([0.0, -2.0, -1.0])                     # sensor position (mm), fixed
angles = np.linspace(-30, 30, 61)                    # -30..+30 deg, 1 deg steps
Ry = Rot.from_euler("y", angles, degrees=True)

Bw = magnet.getB(p0)                                 # world field at p0 (constant)
B = np.einsum("nij,j->ni", Ry.inv().as_matrix(), Bw) # field in the sensor frame
mag = np.linalg.norm(B, axis=1)                      # |B| (T) -- essentially constant

# 3. Save the readings.
with open("magnet_sensor_pitch_readings.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["pitch_deg", "pos_x", "pos_y", "pos_z", "Bx", "By", "Bz", "absB_T"])
    for a, b, m in zip(angles, B, mag):
        w.writerow([f"{a:.1f}", *np.round(p0, 4), *b, m])
print(f"saved magnet_sensor_pitch_readings.csv  ({len(angles)} rows)")
print(f"|B| constant at {mag.mean()*1e3:.2f} mT (spread {np.ptp(mag)*1e3:.2e} mT)")

# 4. Draw B in the sensor frame: arrows from the sensor (origin). Magnitude is
#    constant, so color encodes PITCH ANGLE (the informative variable) instead of
#    |B|. Every 4th sample so arrows stay legible.
L = 3.0                                              # arrow length (display mm)
tip = B / mag.mean() * L
sub = slice(None, None, 4)
b_s, t_s, a_s = B[sub], tip[sub], angles[sub]

fig = go.Figure()

# arrow shafts: origin -> tip -> NaN gap
shaft = np.stack([np.zeros_like(t_s), t_s, np.full_like(t_s, np.nan)], axis=1).reshape(-1, 3)
fig.add_trace(go.Scatter3d(x=shaft[:, 0], y=shaft[:, 1], z=shaft[:, 2], mode="lines",
                           line=dict(color="lightgray", width=2), hoverinfo="skip"))

# arrow heads, pointing along B, colored by pitch angle
fig.add_trace(go.Cone(
    x=t_s[:, 0], y=t_s[:, 1], z=t_s[:, 2], u=b_s[:, 0], v=b_s[:, 1], w=b_s[:, 2],
    anchor="tip", sizemode="absolute", sizeref=0.9,
    colorscale="Viridis", cmin=angles.min(), cmax=angles.max(),
    customdata=np.stack([a_s, mag[sub] * 1e3], axis=1),
    hovertemplate=("pitch = %{customdata[0]:.0f}°<br>"
                   "B = (%{u:.3g}, %{v:.3g}, %{w:.3g}) T<br>"
                   "<b>|B| = %{customdata[1]:.2f} mT</b><extra></extra>"),
    colorbar=dict(title="pitch (°)", len=0.6)))

# arc traced by the arrow tips (constant radius = constant |B|)
fig.add_trace(go.Scatter3d(
    x=tip[:, 0], y=tip[:, 1], z=tip[:, 2], mode="lines",
    line=dict(color=angles, colorscale="Viridis",
              cmin=angles.min(), cmax=angles.max(), width=5),
    hoverinfo="skip"))

# mark the sweep endpoints so travel direction is clear
fig.add_trace(go.Scatter3d(
    x=tip[[0, -1], 0], y=tip[[0, -1], 1], z=tip[[0, -1], 2],
    mode="markers+text", marker=dict(size=4, color=["#440154", "#fde725"]),
    text=["-30°", "+30°"], textposition="top center",
    textfont=dict(size=11), hoverinfo="skip"))

# sensor at the origin + its reference axes
fig.add_trace(go.Scatter3d(x=[0], y=[0], z=[0], mode="markers+text",
                           marker=dict(size=5, color="black"),
                           text=["sensor"], textposition="bottom center",
                           hoverinfo="skip"))
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
    scene=dict(
        xaxis=dict(title="x_s (mm)", range=[-lim, lim]),
        yaxis=dict(title="y_s (mm)", range=[-lim, lim]),
        zaxis=dict(title="z_s (mm)", range=[-lim, lim]),
        aspectmode="cube",
        camera=dict(eye=dict(x=0.4, y=-1.8, z=0.6))),
    margin=dict(l=0, r=0, t=40, b=0))

fig.write_html("magnet_sensor_pitch.html", include_plotlyjs="cdn")
fig.show()
