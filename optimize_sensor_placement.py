#!/usr/bin/env python
"""Optimize the placement of two 3-axis sensors for yaw/pitch/roll estimation.

Setup: magnet 3x2 mm at (0, 0.5, 0) (0.5 mm off the pivot, N -> +x), pivot at
the origin, sensors ride the shell at radius |(0,-2,-1)| = sqrt(5) mm and
rotate with it: reading_i(y,p,r) = R^-1 . B(R . p_i),  R = Rx(r) Ry(p) Rz(y).

Metric: at each pose, the 6x3 Jacobian J = d(readings)/d(angles) maps angle
changes to reading changes. Its smallest singular value sigma_min (mT/deg) is
the signal in the WORST angle direction; expected angle error ~ noise/sigma_min.
We maximize the minimum sigma_min over the workspace (yaw,pitch +-40, roll
+-180): make the flattest spot of the lookup table as steep as possible.

Search: random placements on the shell sphere (with clearance constraints),
then Nelder-Mead refinement of the best ones, then validation on a fresh pose
set + a full estimator recovery test with sensor noise.

Run:  .venv/bin/python optimize_sensor_placement.py [--html out.html]
"""
import sys
import numpy as np
import magpylib as magpy
from scipy.spatial.transform import Rotation as Rot
from scipy.optimize import minimize, least_squares

# --- geometry ----------------------------------------------------------------
MAGNET_OFFSET = np.array([0, 0.5, 0])
R_SHELL = np.sqrt(5.0)                 # sensor orbit radius (mm), as in the repo
MIN_SENSOR_GAP = 1.2                   # mm between the two chips
MIN_MAGNET_CLEARANCE = 0.25            # mm from the magnet body
NOISE = 0.1                            # sensor noise, mT per axis

magnet = magpy.magnet.Cylinder(polarization=(0, 0, 1.2), dimension=(3, 2))
magnet.rotate_from_angax(90, "y", anchor=(0, 0, 0))
magnet.position = MAGNET_OFFSET

BASELINE = np.array([[0.0, -2.0, -1.0], [0.0, -1.0, 2.0]])   # hand-picked pair

def sph_to_xyz(th_phi):
    """(theta, phi) in deg -> point on the shell sphere."""
    t, p = np.radians(th_phi[..., 0]), np.radians(th_phi[..., 1])
    return R_SHELL * np.stack([np.sin(t) * np.cos(p), np.sin(t) * np.sin(p),
                               np.cos(t)], axis=-1)

def magnet_clearance(p):
    """Distance from a point to the magnet body (cylinder, axis || x)."""
    dx = abs(p[0]) - 1.0
    dr = np.hypot(p[1] - MAGNET_OFFSET[1], p[2] - MAGNET_OFFSET[2]) - 1.5
    if dx <= 0 and dr <= 0:
        return -1.0                                       # inside the magnet
    return np.hypot(max(dx, 0), max(dr, 0))

def valid(PS):
    return (np.linalg.norm(PS[0] - PS[1]) >= MIN_SENSOR_GAP
            and all(magnet_clearance(p) >= MIN_MAGNET_CLEARANCE for p in PS))

# --- worst-case sensitivity over a pose set (vectorized) ----------------------
def make_pose_set(n_yp, n_roll):
    """Deterministic pose grid: yaw,pitch in +-40, roll in +-180."""
    g = np.meshgrid(np.linspace(-40, 40, n_yp), np.linspace(-40, 40, n_yp),
                    np.arange(-180, 180, 360 / n_roll), indexing="ij")
    poses = np.stack(g, -1).reshape(-1, 3)
    h = 0.1                            # central-difference step (deg)
    pert = np.concatenate([poses + s * h * e for e in np.eye(3) for s in (1, -1)])
    R = Rot.from_euler("zyx", pert, degrees=True)
    return dict(n=len(poses), h=h, Rmat=R.as_matrix(), Rinv=R.inv().as_matrix())

def sigma_mins(PS, S):
    """sigma_min (mT/deg) of the stacked Jacobian at every pose in the set."""
    reads = []
    for p in PS:
        pos = S["Rmat"] @ p
        reads.append(np.einsum("nij,nj->ni", S["Rinv"], magnet.getB(pos)))
    Ball = np.concatenate(reads, axis=1).reshape(3, 2, S["n"], -1)
    J = np.moveaxis((Ball[:, 0] - Ball[:, 1]) / (2 * S["h"]), 0, -1) * 1e3
    return np.linalg.svd(J, compute_uv=False)[:, -1]

# --- search -------------------------------------------------------------------
def xyz_to_sph(p):
    return np.array([np.degrees(np.arccos(p[2] / R_SHELL)),
                     np.degrees(np.arctan2(p[1], p[0]))])

def optimize(n_random=1000, seed=0):
    S = make_pose_set(n_yp=9, n_roll=36)          # dense search grid
    rng = np.random.default_rng(seed)

    def score(x):                       # x = (th1, phi1, th2, phi2), maximize
        PS = sph_to_xyz(x.reshape(2, 2))
        return sigma_mins(PS, S).min() if valid(PS) else -1.0

    base_x = np.concatenate([xyz_to_sph(p) for p in BASELINE])
    print(f"random search: {n_random} placements ... (baseline scores {score(base_x):.3f})")
    cand = np.column_stack([np.degrees(np.arccos(rng.uniform(-1, 1, 2 * n_random))),
                            rng.uniform(-180, 180, 2 * n_random)]).reshape(-1, 4)
    scores = np.array([score(x) for x in cand])
    order = np.argsort(scores)[::-1]
    print(f"  best random score: {scores[order[0]]:.3f} mT/deg")

    best_x, best_s = None, -np.inf
    for x0 in [base_x, *cand[order[:5]]]:         # baseline is a seed too
        r = minimize(lambda x: -score(x), x0, method="Nelder-Mead",
                     options=dict(xatol=0.5, fatol=1e-3, maxiter=400))
        if -r.fun > best_s:
            best_x, best_s = r.x, -r.fun
    print(f"  after refinement:  {best_s:.3f} mT/deg")
    return sph_to_xyz(best_x.reshape(2, 2))

# --- full recovery test (does the estimator actually work?) -------------------
def read(ypr, PS):
    R = Rot.from_euler("zyx", ypr, degrees=True)
    return R.inv().apply(magnet.getB(R.apply(PS))).ravel()

def recovery_test(PS, n=20, seed=3):
    yg, pg = np.arange(-40, 41, 10), np.arange(-40, 41, 10)
    rg = np.arange(-180, 180, 5)
    G = np.stack(np.meshgrid(yg, pg, rg, indexing="ij"), -1).reshape(-1, 3)
    Rg = Rot.from_euler("zyx", G, degrees=True)
    Bg = np.stack([np.einsum("nij,nj->ni", Rg.inv().as_matrix(), magnet.getB(Rg.apply(p)))
                   for p in PS], 1).reshape(len(G), -1)

    def estimate(b):
        best, cost = None, np.inf
        for x0 in G[np.argsort(np.linalg.norm(Bg - b, axis=1))[:5]]:
            s = least_squares(lambda x: (read(x, PS) - b) * 1e3, x0,
                              bounds=([-45, -45, -185], [45, 45, 185]))
            if s.cost < cost:
                best, cost = s.x, s.cost
        return best

    rng = np.random.default_rng(seed)
    poses = np.column_stack([rng.uniform(-40, 40, n), rng.uniform(-40, 40, n),
                             rng.uniform(-180, 180, n)])
    errs = []
    for t in poses:
        e = estimate(read(t, PS) + rng.normal(0, NOISE * 1e-3, 3 * len(PS)))
        err = np.abs(e - t); err[2] = min(err[2], 360 - err[2])
        errs.append(err.max())
    return np.array(errs)

# --- run -----------------------------------------------------------------------
if __name__ == "__main__":
    PS_opt = optimize()
    S_val = make_pose_set(n_yp=11, n_roll=72)   # finer grid: no overfitting

    print("\nplacements (mm):")
    print(f"  baseline : s1 = {np.round(BASELINE[0], 2)},  s2 = {np.round(BASELINE[1], 2)}")
    print(f"  optimized: s1 = {np.round(PS_opt[0], 2)},  s2 = {np.round(PS_opt[1], 2)}")
    print(f"  sensor separation: baseline {np.linalg.norm(BASELINE[0]-BASELINE[1]):.2f} mm, "
          f"optimized {np.linalg.norm(PS_opt[0]-PS_opt[1]):.2f} mm")

    print(f"\nworst-direction sensitivity on the fine validation grid (noise {NOISE} mT):")
    print(f"{'config':>10} | {'min':>6} {'median':>7} (mT/deg) | worst-pose error")
    for name, PS in [("baseline", BASELINE), ("optimized", PS_opt)]:
        s = sigma_mins(PS, S_val)
        print(f"{name:>10} | {s.min():6.3f} {np.median(s):7.3f}          | "
              f"{NOISE/s.min():6.3f} deg (median {NOISE/np.median(s):5.3f})")

    for name, PS in [("baseline", BASELINE), ("optimized", PS_opt)]:
        e = recovery_test(PS)
        print(f"recovery test, {name:9s}: 20 noisy poses, worst {e.max():6.3f} deg, "
              f"median {np.median(e):5.3f} deg")

    # optional 3D view of the geometry
    if "--html" in sys.argv:
        import plotly.graph_objects as go
        fig = go.Figure()
        th = np.linspace(0, 2*np.pi, 40)
        X, TH = np.meshgrid([-1, 1], th)
        fig.add_trace(go.Surface(x=X, y=1.5*np.cos(TH)+0.5, z=1.5*np.sin(TH),
                                 surfacecolor=np.zeros_like(X), opacity=0.35,
                                 colorscale=[[0, "gray"], [1, "gray"]],
                                 showscale=False, hoverinfo="skip"))
        u, v = np.mgrid[0:np.pi:15j, 0:2*np.pi:30j]        # shell wireframe
        fig.add_trace(go.Surface(x=R_SHELL*np.sin(u)*np.cos(v),
                                 y=R_SHELL*np.sin(u)*np.sin(v), z=R_SHELL*np.cos(u),
                                 opacity=0.08, surfacecolor=np.zeros_like(u),
                                 colorscale=[[0, "steelblue"], [1, "steelblue"]],
                                 showscale=False, hoverinfo="skip"))
        for PS, col, lab in [(BASELINE, "gray", "baseline"),
                             (PS_opt, "crimson", "optimized")]:
            fig.add_trace(go.Scatter3d(
                x=PS[:, 0], y=PS[:, 1], z=PS[:, 2], mode="markers+text",
                marker=dict(size=7, color=col), text=[f"{lab} s1", f"{lab} s2"],
                textfont=dict(color=col), textposition="top center", name=lab))
        fig.add_trace(go.Scatter3d(x=[0], y=[0], z=[0], mode="markers+text",
                                   marker=dict(size=4, color="black"),
                                   text=["pivot"], showlegend=False))
        lim = R_SHELL * 1.15
        fig.update_layout(title="Sensor placement: baseline (gray) vs optimized (red)",
                          scene=dict(xaxis=dict(range=[-lim, lim], title="x (mm)"),
                                     yaxis=dict(range=[-lim, lim], title="y (mm)"),
                                     zaxis=dict(range=[-lim, lim], title="z (mm)"),
                                     aspectmode="cube"),
                          margin=dict(l=0, r=0, t=40, b=0))
        out = sys.argv[sys.argv.index("--html") + 1]
        fig.write_html(out, include_plotlyjs=True)
        print("wrote", out)
