#!/usr/bin/env python
"""
A 3-axis magnetic sensor orbiting a disc magnet, read out in the SENSOR's frame.

  - Disc magnet 3 mm x 2 mm at the origin, N face -> +x, S face -> -x.
  - Sensor starts at (0, -2, -1) and rotates rigidly about the z-axis a full
    turn (0 -> 360 deg), orbiting on a radius-2 circle at z = -1, back to start.
    Being off the z=0 plane, the reading is fully 3D (Bz != 0), which removes
    the yaw ambiguity that exists for an in-plane (z=0) orbit.
  - Because the sensor rotates, its local axes turn with it, so at angle t the
    field it reads is  B_sensor(t) = Rz(t)^-1 . B_world( Rz(t) . p0 ).

Outputs:
  - magnet_sensor_readings.csv : the recorded vectors at every angle.
  - magnet_sensor.html         : interactive 3D view of B in the sensor frame.
                                 Arrows show direction; color/length show |B|.
                                 Hover an arrow to read the values.

Run:  .venv/bin/python magnet_sensor.py
"""
import csv
import numpy as np
import magpylib as magpy
import plotly.graph_objects as go
from scipy.spatial.transform import Rotation as Rot

# 1. Magnet at origin, N -> +x.
magnet = magpy.magnet.Cylinder(polarization=(0, 0, 1.2), dimension=(3, 2))
magnet.rotate_from_angax(90, "y", anchor=(0, 0, 0))

# 2. Sensor orbit: rotate the start point about z, full turn, back to start.
p0 = np.array([0.0, -2.0, -1.0])                     # sensor start position (mm)
angles = np.linspace(0, 360, 73)                     # 0..360 deg, 5 deg steps
Rz = Rot.from_euler("z", angles, degrees=True)

pos = Rz.apply(p0)                                   # sensor positions, world
B = np.einsum("nij,nj->ni", Rz.inv().as_matrix(),    # field in the sensor frame
              magnet.getB(pos))
mag = np.linalg.norm(B, axis=1)                      # |B| (T)
cmin, cmax = mag.min(), mag.max()

# 3. Save the readings.
with open("magnet_sensor_readings.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["angle_deg", "pos_x", "pos_y", "pos_z", "Bx", "By", "Bz", "absB_T"])
    for a, p, b, m in zip(angles, pos, B, mag):
        w.writerow([f"{a:.1f}", *np.round(p, 4), *b, m])
print(f"saved magnet_sensor_readings.csv  ({len(angles)} rows)")

# 4. Draw B in the sensor frame: arrows from the sensor (origin), length and
#    color both proportional to |B|. Draw every 2nd sample so arrows stay legible.
L = 3.0                                              # longest arrow (display mm)
tip = B / cmax * L
sub = slice(None, None, 2)

fig = go.Figure()

# arrow shafts: origin -> tip -> NaN gap, stacked into one polyline
shaft = np.stack([np.zeros_like(tip[sub]), tip[sub], np.full_like(tip[sub], np.nan)],
                 axis=1).reshape(-1, 3)
fig.add_trace(go.Scatter3d(x=shaft[:, 0], y=shaft[:, 1], z=shaft[:, 2], mode="lines",
                           line=dict(color="lightgray", width=2), hoverinfo="skip"))

# arrow heads, pointing along B, colored by |B|
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

# closed path traced by the arrow tips over one revolution
fig.add_trace(go.Scatter3d(
    x=tip[:, 0], y=tip[:, 1], z=tip[:, 2], mode="lines",
    line=dict(color=mag, colorscale="Viridis", cmin=cmin, cmax=cmax, width=5),
    hoverinfo="skip"))

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
    title="Sensor rotating about z (start 0,-2,-1) — B in the sensor frame",
    showlegend=False,
    scene=dict(
        xaxis=dict(title="x_s (mm)", range=[-lim, lim]),
        yaxis=dict(title="y_s (mm)", range=[-lim, lim]),
        zaxis=dict(title="z_s (mm)", range=[-lim, lim]),
        aspectmode="cube",
        camera=dict(eye=dict(x=0.9, y=-0.9, z=1.5))),
    margin=dict(l=0, r=0, t=40, b=0))

fig.write_html("magnet_sensor.html", include_plotlyjs="cdn")
fig.show()
