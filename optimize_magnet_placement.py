#!/usr/bin/env python
"""Optimize the MAGNET (size, offset, tilt) for yaw/pitch/roll separability.

Sensors are fixed at the baseline pair (0,-2,-1) and (0,-1,2) on the shell
(radius sqrt(5) mm, ball-joint kinematics as in optimize_sensor_placement.py).
The magnet is a disc, polarization 1.2 T, free parameters:
    D    - diameter (mm)          H   - height along N-S (mm)
    off  - center offset from the pivot, perpendicular to the roll axis (mm)
    ang  - direction of that offset in the y-z plane (deg)
    tilt - N-S axis tilted away from the roll axis, about z (deg)
Constraint: the whole magnet body stays >= 0.15 mm clear of the sensor shell.
(NB the current design, D=3 H=2 off=0.5, touches the shell exactly.)

Metric (same as the sensor study): worst-case sigma_min of the 6x3 Jacobian
d(readings)/d(angles) over the workspace yaw,pitch +-40 deg, roll +-180 deg
-> the signal available in the hardest-to-see angle direction, in mT/deg.

Run:  .venv/bin/python optimize_magnet_placement.py
"""
import numpy as np
import magpylib as magpy
from scipy.spatial.transform import Rotation as Rot
from scipy.optimize import minimize

R_SHELL = np.sqrt(5.0)
CLEARANCE = 0.15
NOISE = 0.1                                     # mT per axis
SENSORS = np.array([[0.0, -2.0, -1.0], [0.0, -1.0, 2.0]])

# parameter vector x = [D, H, off, ang, tilt]
X0     = np.array([3.0, 2.0, 0.5,   0.0,  0.0])   # current design (touches shell)
LO     = np.array([1.0, 0.5, 0.0, -180.0,  0.0])
HI     = np.array([3.0, 2.0, 1.5,  180.0, 90.0])

def build(x):
    D, H, off, ang, tilt = x
    m = magpy.magnet.Cylinder(polarization=(0, 0, 1.2), dimension=(D, H))
    m.rotate_from_angax(90, "y", anchor=(0, 0, 0))          # N -> +x
    m.rotate_from_angax(tilt, "z", anchor=(0, 0, 0))        # tilt off roll axis
    m.position = off * np.array([0, np.cos(np.radians(ang)), np.sin(np.radians(ang))])
    return m

def fits(x):
    """Farthest magnet-body point must stay inside the shell minus clearance."""
    D, H, off, ang, tilt = x
    u = np.array([np.cos(np.radians(tilt)), np.sin(np.radians(tilt)), 0])
    c = off * np.array([0, np.cos(np.radians(ang)), np.sin(np.radians(ang))])
    c_par = abs(c @ u)
    c_perp = np.linalg.norm(c - (c @ u) * u)
    return np.hypot(c_par + H / 2, c_perp + D / 2) <= R_SHELL - CLEARANCE

def make_pose_set(n_yp, n_roll):
    g = np.meshgrid(np.linspace(-40, 40, n_yp), np.linspace(-40, 40, n_yp),
                    np.arange(-180, 180, 360 / n_roll), indexing="ij")
    poses = np.stack(g, -1).reshape(-1, 3)
    h = 0.1
    pert = np.concatenate([poses + s * h * e for e in np.eye(3) for s in (1, -1)])
    R = Rot.from_euler("zyx", pert, degrees=True)
    return dict(n=len(poses), h=h, Rmat=R.as_matrix(), Rinv=R.inv().as_matrix())

def worst_sigma(x, S, check=True):
    if check and not (fits(x) and np.all(x >= LO) and np.all(x <= HI)):
        return -1.0
    magnet = build(x)
    reads = []
    for p in SENSORS:
        pos = S["Rmat"] @ p
        reads.append(np.einsum("nij,nj->ni", S["Rinv"], magnet.getB(pos)))
    Ball = np.concatenate(reads, axis=1).reshape(3, 2, S["n"], -1)
    J = np.moveaxis((Ball[:, 0] - Ball[:, 1]) / (2 * S["h"]), 0, -1) * 1e3
    return np.linalg.svd(J, compute_uv=False)[:, -1].min()

if __name__ == "__main__":
    S = make_pose_set(9, 36)
    print(f"current design (D=3, H=2, off=0.5, tilt=0): "
          f"{worst_sigma(X0, S, check=False):.3f} mT/deg"
          "   [touches the shell -> reference only]")

    rng = np.random.default_rng(0)
    cand = rng.uniform(LO, HI, size=(600, 5))
    scores = np.array([worst_sigma(x, S) for x in cand])
    order = np.argsort(scores)[::-1]
    print(f"random search (600): best {scores[order[0]]:.3f} mT/deg")

    best_x, best_s = None, -np.inf
    for x0 in [X0, *cand[order[:4]]]:
        r = minimize(lambda x: -worst_sigma(x, S), x0, method="Nelder-Mead",
                     options=dict(xatol=1e-2, fatol=1e-3, maxiter=600))
        if -r.fun > best_s:
            best_x, best_s = r.x, -r.fun
    D, H, off, ang, tilt = best_x
    print(f"after refinement: {best_s:.3f} mT/deg")
    print(f"\noptimal magnet: D={D:.2f} mm, H={H:.2f} mm, offset={off:.2f} mm "
          f"at {ang:.0f} deg in y-z, tilt={tilt:.1f} deg")

    S_val = make_pose_set(11, 72)
    s_ref = worst_sigma(X0, S_val, check=False)
    s_opt = worst_sigma(best_x, S_val)
    print(f"\nvalidation (finer grid): worst-case sensitivity, {NOISE} mT noise")
    print(f"  current magnet : {s_ref:.3f} mT/deg -> worst error {NOISE/s_ref:.3f} deg")
    print(f"  optimal magnet : {s_opt:.3f} mT/deg -> worst error {NOISE/s_opt:.3f} deg")
    print(f"  improvement    : {s_opt/s_ref:.2f}x")
