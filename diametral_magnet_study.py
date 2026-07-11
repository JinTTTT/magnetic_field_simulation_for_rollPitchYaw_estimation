#!/usr/bin/env python
"""Diametrally magnetized disc magnet for 3-angle estimation.

Rotary encoders use a diametral disc (magnetized ACROSS the diameter) spinning
about its geometric axis: for a sensor on that axis the field direction turns
1:1 with the shaft -> angle = atan2 of two components. Here we mount the same
idea in the ball joint: disc geometric axis along the roll axis (x), N-S along
y, magnet FIXED and centered; sensors ride the shell. Because the rotation
axis is perpendicular to the magnetization, the field has no rotational
symmetry about the roll axis -- roll is visible WITHOUT any off-center trick.

Configs compared (worst-case sigma_min of the Jacobian over the workspace,
yaw,pitch +-40 deg, roll +-180 deg, sensors on the shell radius sqrt(5)):
  A. axial magnet + 0.5 mm offset, baseline sensor pair   (current best)
  B. diametral centered, baseline sensor pair
  C. diametral centered, sensor1 at the roll-axis pole (sqrt5,0,0) + (0,-2,-1)
  D. same as C but magnet also offset 0.5 mm along +y
  E. diametral centered, ONLY the pole sensor (3 readings, 3 angles)
Plus: encoder check -- at neutral yaw/pitch, is atan2(-Bz,-By) == roll exactly?

Run:  .venv/bin/python diametral_magnet_study.py
"""
import numpy as np
import magpylib as magpy
from scipy.spatial.transform import Rotation as Rot

R_SHELL = np.sqrt(5.0)
NOISE = 0.1                                       # mT per axis

def axial_magnet(offset=(0, 0, 0)):
    m = magpy.magnet.Cylinder(polarization=(0, 0, 1.2), dimension=(3, 2))
    m.rotate_from_angax(90, "y", anchor=(0, 0, 0))          # N -> +x
    m.position = offset
    return m

def diametral_magnet(offset=(0, 0, 0)):
    # polarization _|_ geometric axis (diametral), then axis z -> x;
    # result: disc axis along x (the roll axis), N-S along y.
    m = magpy.magnet.Cylinder(polarization=(0, 1.2, 0), dimension=(3, 2))
    m.rotate_from_angax(90, "y", anchor=(0, 0, 0))
    m.position = offset
    return m

def make_pose_set(n_yp=11, n_roll=72):
    g = np.meshgrid(np.linspace(-40, 40, n_yp), np.linspace(-40, 40, n_yp),
                    np.arange(-180, 180, 360 / n_roll), indexing="ij")
    poses = np.stack(g, -1).reshape(-1, 3)
    h = 0.1
    pert = np.concatenate([poses + s * h * e for e in np.eye(3) for s in (1, -1)])
    R = Rot.from_euler("zyx", pert, degrees=True)
    return dict(n=len(poses), h=h, Rmat=R.as_matrix(), Rinv=R.inv().as_matrix())

def sigmas(source, sensors, S):
    reads = []
    for p in np.atleast_2d(sensors):
        pos = S["Rmat"] @ p
        reads.append(np.einsum("nij,nj->ni", S["Rinv"], source.getB(pos)))
    Ball = np.concatenate(reads, axis=1).reshape(3, 2, S["n"], -1)
    J = np.moveaxis((Ball[:, 0] - Ball[:, 1]) / (2 * S["h"]), 0, -1) * 1e3
    return np.linalg.svd(J, compute_uv=False)[:, -1]

POLE = np.array([R_SHELL, 0.0, 0.0])              # sensor ON the roll axis
BASE = np.array([[0.0, -2.0, -1.0], [0.0, -1.0, 2.0]])

if __name__ == "__main__":
    S = make_pose_set()
    configs = [
        ("A axial+offset, base pair ", axial_magnet((0, 0.5, 0)), BASE),
        ("B diametral, base pair    ", diametral_magnet(), BASE),
        ("C diametral, pole+side    ", diametral_magnet(), np.array([POLE, BASE[0]])),
        # NB: offsetting along y slides the magnet along its own dipole
        # symmetry line (m || y) -- no help. Offset must be _|_ m:
        ("D  diametral+offset y (bad)", diametral_magnet((0, 0.5, 0)), np.array([POLE, BASE[0]])),
        ("D2 diametral+offset x     ", diametral_magnet((0.5, 0, 0)), np.array([POLE, BASE[0]])),
        ("D3 diametral+offset z     ", diametral_magnet((0, 0, 0.5)), np.array([POLE, BASE[0]])),
        ("E diametral, pole ONLY    ", diametral_magnet(), POLE),
    ]
    print(f"worst-direction sensitivity over the workspace ({NOISE} mT noise):")
    print(f"{'config':>28} | {'min':>6} {'median':>7} {'max':>7} (mT/deg) | worst err")
    for name, mag, sens in configs:
        s = sigmas(mag, sens, S)
        print(f"{name:>28} | {s.min():6.3f} {np.median(s):7.3f} {s.max():7.3f}"
              f"          | {NOISE/s.min():6.3f} deg")

    # encoder check: neutral yaw/pitch, sweep roll, pole sensor, atan2 formula
    print("\nencoder check at the pole sensor (yaw=pitch=0):")
    m = diametral_magnet()
    print(f"{'roll':>6} | {'Bx':>7} {'By':>8} {'Bz':>8} (mT) | atan2(-Bz,-By)")
    for roll in (-120, -45, 0, 30, 90, 165):
        R = Rot.from_euler("zyx", [0, 0, roll], degrees=True)
        b = R.inv().apply(m.getB(R.apply(POLE))) * 1e3
        est = np.degrees(np.arctan2(-b[2], -b[1]))
        print(f"{roll:+6d} | {b[0]:7.2f} {b[1]:8.2f} {b[2]:8.2f}      | {est:+8.2f}")
