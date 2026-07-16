#!/usr/bin/env python
"""The physical setup and the forward field model (the "simulation").

Hardware (matches the real rig, measured 2026-07-16):
  - Pivot: the ball joint center, at the origin. All rotations are about it.
  - Sensors: two 3-axis TLV493D on a ring of radius 24 mm in the z = +20 mm
    plane (the ball joint sits 20 mm below the sensor plane). Sensor 1 on the
    +x axis at (24, 0, 20); sensor 2 is 120 deg clockwise from it (top view),
    at azimuth -120 deg = (-12, -20.8, 20).
  - Magnet: ONE NdFeB disc (10 mm dia x 5 mm). Its N-S line points along +x
    (the roll axis), N facing sensor 1 (verified by measurement 2026-07-16).
    Its center sits 15 mm above the sensor plane, i.e. at (0, 0, 35) --
    off the pivot center along z, which is what makes roll observable.

The magnet is fixed to the base; the two sensors ride the shell, so a shell
rotation (yaw, pitch, roll) about the ball joint carries the sensors to new
positions and turns their axes with them.

This module has NO estimation code -- see estimation.py for the inverse solve.
Run it directly to open a 3D view of the setup (magnet, field, sensors):

    .venv/bin/python simulation.py
"""
import json
import os

import numpy as np
import magpylib as magpy
from scipy.spatial.transform import Rotation

# ---------------- measured rig geometry (pivot = ball joint = origin) ---------
SENSOR_PLANE_Z = 20.0           # mm, sensor x-y plane sits 20 mm above the pivot
MAGNET_Z = SENSOR_PLANE_Z + 15.0   # mm, magnet center 15 mm above the sensor plane

# ---------------- the magnet --------------------------------------------------
# dimension=(diameter, height): a single 10 mm dia x 5 mm disc.
magnet = magpy.magnet.Cylinder(polarization=(0, 0, 1.2), dimension=(10, 5))
magnet.rotate_from_angax(90, "y", anchor=(0, 0, 0))   # N-S line now along +x
magnet.position = (0, 0, MAGNET_Z)                    # fixed to the base

# ---------------- the two sensors (home positions, at zero angles) ------------
SENSOR_RADIUS = 24.0            # mm, ring radius in the z = SENSOR_PLANE_Z plane
SENSOR_1_AZIMUTH = 0.0          # deg from +x (CCW): sensor 1 sits on the +x axis
SENSOR_SEPARATION = 120.0       # deg, sensor 2 is this far CLOCKWISE (top view)


def _ring_point(azimuth_deg):
    a = np.radians(azimuth_deg)
    return np.array([SENSOR_RADIUS * np.cos(a),
                     SENSOR_RADIUS * np.sin(a),
                     SENSOR_PLANE_Z])


SENSOR_1_HOME = _ring_point(SENSOR_1_AZIMUTH)
SENSOR_2_HOME = _ring_point(SENSOR_1_AZIMUTH - SENSOR_SEPARATION)
SENSOR_HOMES = (SENSOR_1_HOME, SENSOR_2_HOME)

SENSOR_NOISE = 0.1e-3           # 0.1 mT of noise per axis

# ---------------- calibrated geometry (written by calibrate.py) ---------------
# If calibrated_geometry.json exists, the fitted magnet replaces the nominal
# one and each chip's fitted orientation + the IMU mount error are applied in
# predict_readings. Without the file everything runs on the nominal drawing.
_CHIP_ROTATIONS = (None, None)
_IMU_MOUNT_ERR = None
_YAW_ALIGNMENT_DEG = 0.0
_SENSOR_GAINS = (np.ones(3), np.ones(3))
_SENSOR_BIASES = (np.zeros(3), np.zeros(3))
_AMBIENT_FIELD = np.zeros(3)
_MODEL_SENSOR_HOMES = SENSOR_HOMES
_POSE_EULER_SEQUENCE = "zyx"
_CAL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "calibrated_geometry.json")
if os.path.exists(_CAL_PATH):
    with open(_CAL_PATH) as _f:
        _cal = json.load(_f)
    magnet = magpy.magnet.Cylinder(
        polarization=(0, 0, _cal["polarization"]), dimension=(10, 5))
    magnet.rotate_from_angax(90, "y", anchor=(0, 0, 0))
    magnet.rotate_from_angax(_cal["tilt_y"], "y", anchor=(0, 0, 0))
    magnet.rotate_from_angax(_cal["tilt_z"], "z", anchor=(0, 0, 0))
    magnet.position = _cal["magnet_position"]
    _CHIP_ROTATIONS = (Rotation.from_rotvec(_cal["chip_rotvec_S1"]),
                       Rotation.from_rotvec(_cal["chip_rotvec_S2"]))
    _IMU_MOUNT_ERR = Rotation.from_rotvec(_cal["imu_mount_rotvec"])
    _YAW_ALIGNMENT_DEG = _cal.get("yaw_alignment_deg", 0.0)
    _SENSOR_GAINS = (
        np.asarray(_cal.get("sensor_gain_S1", [1.0, 1.0, 1.0])),
        np.asarray(_cal.get("sensor_gain_S2", [1.0, 1.0, 1.0])),
    )
    _SENSOR_BIASES = (
        np.asarray(_cal.get("sensor_bias_mT_S1", [0.0, 0.0, 0.0])) * 1e-3,
        np.asarray(_cal.get("sensor_bias_mT_S2", [0.0, 0.0, 0.0])) * 1e-3,
    )
    _AMBIENT_FIELD = np.asarray(
        _cal.get("ambient_field_mT", [0.0, 0.0, 0.0])) * 1e-3
    _MODEL_SENSOR_HOMES = (
        np.asarray(_cal.get("sensor_home_S1", SENSOR_1_HOME)),
        np.asarray(_cal.get("sensor_home_S2", SENSOR_2_HOME)),
    )
    _POSE_EULER_SEQUENCE = _cal.get("pose_euler_sequence", "zyx")

# workspace of the device (pitch/roll narrowed to +-10 deg, 2026-07-16)
YAW_RANGE = (-120, 120)
PITCH_RANGE = (-10, 10)
ROLL_RANGE = (-10, 10)


# ---------------- forward direction: angles -> readings -----------------------
def predict_readings(yaw, pitch, roll):
    """The 6 numbers the two chips report when the shell stands at these angles.

    The magnet is fixed; the shell (with the sensors) rotates, so each sensor is
    carried to rotation.apply(home) and reports the field in its own turned frame.
    """
    rotation = Rotation.from_euler(
        _POSE_EULER_SEQUENCE,
        [yaw + _YAW_ALIGNMENT_DEG, pitch, roll], degrees=True)
    if _IMU_MOUNT_ERR is not None:
        rotation = rotation * _IMU_MOUNT_ERR        # angles are the IMU's
    readings = []
    for home, chip_rot, gain, bias in zip(
            _MODEL_SENSOR_HOMES, _CHIP_ROTATIONS, _SENSOR_GAINS, _SENSOR_BIASES):
        position = rotation.apply(home)             # shell carries the sensor
        field_world = magnet.getB(position) + _AMBIENT_FIELD
        field_chip = rotation.inv().apply(field_world)   # into the chip's frame
        if chip_rot is not None:
            field_chip = chip_rot.apply(field_chip)      # fitted die orientation
        field_chip = gain * field_chip + bias
        readings.extend(field_chip)
    return np.array(readings)                        # [B1x B1y B1z B2x B2y B2z]


def simulate(yaw, pitch, roll, noise=0.0):
    """predict_readings plus optional sensor noise (a fake measurement)."""
    readings = predict_readings(yaw, pitch, roll)
    if noise > 0:
        readings = readings + np.random.default_rng().normal(0, noise, 6)
    return readings


# ---------------- 3D visualization of the setup -------------------------------
MAGNET_CENTER = np.array([0.0, 0.0, MAGNET_Z])
MAGNET_RADIUS = 5.0                 # mm (10 mm diameter)
MAGNET_HALF_LEN = 2.5               # mm (one 5 mm disc, N-S along x)


def _cylinder_mesh(a, b, radius, color, n=48):
    """A plotly Mesh3d cylinder (with end caps) spanning point a to point b."""
    import plotly.graph_objects as go
    a, b = np.asarray(a, float), np.asarray(b, float)
    axis = b - a
    ahat = axis / np.linalg.norm(axis)
    # two vectors perpendicular to the axis
    ref = np.array([1.0, 0, 0]) if abs(ahat[0]) < 0.9 else np.array([0, 1.0, 0])
    u = np.cross(ahat, ref); u /= np.linalg.norm(u)
    v = np.cross(ahat, u)
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = np.outer(np.cos(t), u) + np.outer(np.sin(t), v)   # (n,3) unit ring
    r0 = a + radius * ring
    r1 = b + radius * ring
    verts = np.vstack([r0, r1, a, b])            # 2n ring verts + 2 cap centers
    c0, c1 = 2 * n, 2 * n + 1
    i, j, k = [], [], []
    for s in range(n):
        s2 = (s + 1) % n
        i += [s, s];        j += [s2, n + s2];  k += [n + s2, n + s]   # side quad
        i += [c0];          j += [s];           k += [s2]              # cap a
        i += [c1];          j += [n + s2];       k += [n + s]           # cap b
    return go.Mesh3d(x=verts[:, 0], y=verts[:, 1], z=verts[:, 2],
                     i=i, j=j, k=k, color=color, flatshading=True,
                     hoverinfo="skip", showscale=False)


def _field_line(seed, sign=1.0, step=0.5, max_steps=1200):
    """Trace one 3D magnetic field line by RK4 integration along B/|B|."""
    def deriv(p):
        B = magnet.getB(p)
        nb = np.linalg.norm(B)
        return None if nb < 1e-12 else sign * B / nb

    def inside(p):   # inside the magnet body?
        d = p - MAGNET_CENTER
        return abs(d[0]) < MAGNET_HALF_LEN and np.hypot(d[1], d[2]) < MAGNET_RADIUS

    pts = [np.asarray(seed, float)]
    p = pts[0].copy()
    for _ in range(max_steps):
        k1 = deriv(p)
        if k1 is None: break
        k2 = deriv(p + 0.5 * step * k1)
        if k2 is None: break
        k3 = deriv(p + 0.5 * step * k2)
        if k3 is None: break
        k4 = deriv(p + step * k3)
        if k4 is None: break
        p = p + step / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        pts.append(p.copy())
        if inside(p) or np.abs(p - MAGNET_CENTER).max() > 40 or p[2] < -20:
            break
    return np.array(pts)


def build_figure():
    """A clean 3D plotly figure: red/blue magnet, field lines, two sensors, origin."""
    import plotly.graph_objects as go
    fig = go.Figure()

    # ---- magnetic field lines (grey curves) looping N -> S, seeded on rings
    #      just off the N pole face at several radii and azimuths ----
    seeds = []
    for rho in (3.2, 4.0, 4.7):
        for th in np.linspace(0, 2 * np.pi, 12, endpoint=False):
            offset = np.array([0.0, rho * np.cos(th), rho * np.sin(th)])
            seeds.append(MAGNET_CENTER + np.array([MAGNET_HALF_LEN + 0.5, 0, 0]) + offset)
    for seed in seeds:
        line = _field_line(seed, sign=1.0, step=0.4, max_steps=1600)
        if len(line) > 3:
            fig.add_trace(go.Scatter3d(
                x=line[:, 0], y=line[:, 1], z=line[:, 2], mode="lines",
                line=dict(color="rgba(70,70,70,0.8)", width=2.5),
                hoverinfo="skip", showlegend=False))

    # ---- the magnet: real-size cylinder, S half red (-x), N half blue (+x) ----
    mid = MAGNET_CENTER
    fig.add_trace(_cylinder_mesh(mid + [-MAGNET_HALF_LEN, 0, 0], mid,
                                 MAGNET_RADIUS, "crimson"))       # S pole
    fig.add_trace(_cylinder_mesh(mid, mid + [MAGNET_HALF_LEN, 0, 0],
                                 MAGNET_RADIUS, "royalblue"))     # N pole
    fig.add_trace(go.Scatter3d(
        x=[mid[0] + MAGNET_HALF_LEN + 2.5, mid[0] - MAGNET_HALF_LEN - 2.5],
        y=[0, 0], z=[mid[2], mid[2]], mode="text",
        text=["<b>N</b>", "<b>S</b>"],
        textfont=dict(size=18, color=["royalblue", "crimson"]),
        hoverinfo="skip", showlegend=False))

    # ---- the two sensors: small dark cylinders (flat, like the chip) ----
    for home, label in zip(SENSOR_HOMES, ("sensor 1", "sensor 2")):
        fig.add_trace(_cylinder_mesh(home + [0, 0, -1.0], home + [0, 0, 1.0],
                                     3.5, "#2b2b2b"))
        fig.add_trace(go.Scatter3d(
            x=[home[0]], y=[home[1]], z=[home[2] + 3],
            mode="text", text=[f"<b>{label}</b><br>({home[0]:.0f},{home[1]:.0f},{home[2]:.0f})"],
            textfont=dict(size=12, color="black"), hoverinfo="skip", showlegend=False))

    # ---- pivot: the ball joint center at the origin ----
    fig.add_trace(go.Scatter3d(
        x=[0], y=[0], z=[0], mode="markers+text",
        marker=dict(size=4, color="black"),
        text=["pivot / ball joint (0,0,0)"], textposition="bottom center",
        hoverinfo="skip", showlegend=False))

    fig.update_layout(
        title=(f"Magnet (S=red, N=blue) at (0,0,{MAGNET_Z:.0f}), N-S along x — "
               f"2 sensors 120° apart, r={SENSOR_RADIUS:.0f} mm at z={SENSOR_PLANE_Z:.0f}"),
        showlegend=False,
        scene=dict(xaxis_title="x (mm)", yaxis_title="y (mm)", zaxis_title="z (mm)",
                   aspectmode="data",
                   camera=dict(eye=dict(x=1.5, y=1.5, z=1.1))),
        margin=dict(l=0, r=0, t=40, b=0))
    return fig


if __name__ == "__main__":
    fig = build_figure()
    fig.write_html("figures/setup_3d.html", include_plotlyjs="inline")
    print("wrote figures/setup_3d.html")
    try:
        fig.write_image("figures/setup_3d.png", width=1100, height=850, scale=2)
        print("wrote figures/setup_3d.png")
    except Exception as e:
        print("PNG export skipped:", e)
    print("\nsensor 1 home:", np.round(SENSOR_1_HOME, 2))
    print("sensor 2 home:", np.round(SENSOR_2_HOME, 2))
