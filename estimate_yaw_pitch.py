#!/usr/bin/env python
"""Estimate a sensor's yaw & pitch from its raw (Bx, By, Bz) reading.

Roll is about the magnet's x-axis (the field direction), so it has no effect and
does not need to be known. Two steps:
  1) simulate: given known yaw/pitch/roll, produce the raw reading the chip sees.
  2) estimate: from ONLY that raw reading, recover yaw and pitch.
"""
import numpy as np
import magpylib as magpy
from scipy.spatial.transform import Rotation as Rot

# --- the magnet and the fixed sensor position ------------------------------
magnet = magpy.magnet.Cylinder(polarization=(0, 0, 1.2), dimension=(3, 2))
magnet.rotate_from_angax(90, "y", anchor=(0, 0, 0))
SENSOR_POS = [0.0, -2.0, -1.0]                 # on the x=0 plane: field points along x
B_world = magnet.getB(SENSOR_POS)              # field at the sensor, in the magnet frame


def simulate(yaw, pitch, roll):
    """Known angles -> raw (Bx, By, Bz) the sensor would report, in its own frame."""
    # rotations about the magnet's fixed axes: yaw(z), pitch(y), roll(x),
    # with roll outermost (about the field axis) so it has no effect.
    R = Rot.from_euler("zyx", [yaw, pitch, roll], degrees=True)
    return R.inv().apply(B_world)              # world field seen in the sensor frame


def estimate(B):
    """Raw (Bx, By, Bz) -> estimated (yaw, pitch) in degrees. Roll not needed."""
    Bx, By, Bz = B
    yaw   = np.degrees(np.arctan2(By, -Bx))
    pitch = np.degrees(np.arctan2(-Bz, np.hypot(Bx, By)))
    return yaw, pitch


# --- try it: known angles -> simulate -> estimate -> compare ----------------
print(f"{'true yaw':>8} {'true pitch':>10} {'roll':>6} | "
      f"{'Bx':>8} {'By':>8} {'Bz':>8} | {'est yaw':>8} {'est pitch':>9}")
for yaw, pitch, roll in [(15, 20, 0), (15, 20, 45), (-25, 10, 80),
                         (30, -15, -60), (0, 28, 130)]:
    B = simulate(yaw, pitch, roll)
    est_yaw, est_pitch = estimate(B)
    print(f"{yaw:8d} {pitch:10d} {roll:6d} | "
          f"{B[0]*1e3:8.2f} {B[1]*1e3:8.2f} {B[2]*1e3:8.2f} | "
          f"{est_yaw:8.2f} {est_pitch:9.2f}")
