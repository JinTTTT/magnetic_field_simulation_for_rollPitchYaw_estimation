#!/usr/bin/env python
"""
3D magnetic field of a disc magnet, drawn as traced field lines.
Magnet: 3 mm diameter x 2 mm tall, sitting at the origin.
Rotated +90 deg about the y-axis, so the N face points +x (S points -x).

Run:  .venv/bin/python magnet_3d.py
Opens an interactive WebGL view in the browser (also writes magnet_3d.html).
Drag to rotate, scroll to zoom — GPU rendering, so it stays smooth.
Hover any field line to read the field strength |B| at that position.
"""
import numpy as np
import magpylib as magpy
import plotly.graph_objects as go

# 1. The magnet, at the origin. In its local frame polarization = +z -> top face N.
#    Then rotate +90 deg about y so N faces +x, S faces -x.
R_MAG, H_MAG = 1.5, 1.0          # radius, half-height (mm)
magnet = magpy.magnet.Cylinder(polarization=(0, 0, 1.2), dimension=(2 * R_MAG, 2 * H_MAG))
magnet.rotate_from_angax(90, "y", anchor=(0, 0, 0))
ori = magnet.orientation          # scipy Rotation: local frame -> world frame

def rot(x, y, z):
    """Apply the magnet's orientation to array-shaped coordinates."""
    pts = np.stack([x, y, z], axis=-1)
    w = ori.apply(pts.reshape(-1, 3)).reshape(pts.shape)
    return w[..., 0], w[..., 1], w[..., 2]

# 2. Trace a field line by RK4-walking along the B direction. Stop when the line
#    escapes the view sphere or returns to the S face, so nothing trails off ugly.
#    Re-entry is tested in the magnet's LOCAL frame so it works after rotation.
R_DOMAIN = 6.5
def field_line(seed, step=0.12, n_steps=600):
    d = lambda p: (b := magnet.getB(p)) / np.linalg.norm(b)
    p, path = np.array(seed, float), [np.array(seed, float)]
    for _ in range(n_steps):
        k1 = d(p); k2 = d(p + .5*step*k1); k3 = d(p + .5*step*k2); k4 = d(p + step*k3)
        p = p + step/6 * (k1 + 2*k2 + 2*k3 + k4)
        path.append(p.copy())
        pl = ori.inv().apply(p)                          # position in local frame
        if np.linalg.norm(p) > R_DOMAIN:                 # left the view box
            break
        if pl[2] < -H_MAG and np.hypot(pl[0], pl[1]) < R_MAG:   # re-entered S face
            break
    return np.array(path)

# 3. Seed dense rings just off the N face (local +z), all the way around, then
#    rotate the seeds into world coordinates -> a full, symmetric "flower".
local_seeds = np.array([(r*np.cos(a), r*np.sin(a), H_MAG + 0.05)
                        for r in (0.5, 0.8, 1.1, 1.35)
                        for a in np.linspace(0, 2*np.pi, 12, endpoint=False)])
seeds = ori.apply(local_seeds)

# 4. Build one Scatter3d line per seed, colored by |B| (log scale), shared colorbar.
#    Each vertex carries its real |B| (Tesla) so hovering reads out the strength
#    at that position — hover anywhere on a line to probe the field.
lines = [field_line(s) for s in seeds]
absB = [np.linalg.norm(magnet.getB(L), axis=1) for L in lines]   # Tesla, per vertex
logB = [np.log10(b) for b in absB]
cmin = min(v.min() for v in logB)
cmax = max(v.max() for v in logB)

hover = ("x=%{x:.2f}, y=%{y:.2f}, z=%{z:.2f} mm<br>"
         "<b>|B| = %{customdata:.4g} T</b><extra></extra>")
fig = go.Figure()
for L, c, b in zip(lines, logB, absB):
    fig.add_trace(go.Scatter3d(
        x=L[:, 0], y=L[:, 1], z=L[:, 2], mode="lines",
        line=dict(color=c, coloraxis="coloraxis", width=4),
        customdata=b, hovertemplate=hover, showlegend=False))

# 5. Magnet body: a semi-transparent gray cylinder (side + caps), rotated to match.
th = np.linspace(0, 2*np.pi, 48)
TH, ZC = np.meshgrid(th, np.array([-H_MAG, H_MAG]))
xs, ys, zs = rot(R_MAG*np.cos(TH), R_MAG*np.sin(TH), ZC)
fig.add_trace(go.Surface(
    x=xs, y=ys, z=zs, surfacecolor=np.zeros_like(TH),
    colorscale=[[0, "gray"], [1, "gray"]], opacity=0.35, showscale=False, hoverinfo="skip"))
for zcap in (-H_MAG, H_MAG):                             # top/bottom disc caps
    rr = np.linspace(0, R_MAG, 2)
    xc, yc, zc = rot(np.outer(rr, np.cos(th)), np.outer(rr, np.sin(th)),
                     np.full((2, th.size), zcap))
    fig.add_trace(go.Surface(
        x=xc, y=yc, z=zc, surfacecolor=np.zeros((2, th.size)),
        colorscale=[[0, "gray"], [1, "gray"]], opacity=0.35, showscale=False, hoverinfo="skip"))

# 6. N / S pole labels, placed on the local poles then rotated (N ends up at +x).
lx, ly, lz = rot(np.array([0, 0.]), np.array([0, 0.]), np.array([H_MAG + 0.5, -H_MAG - 0.5]))
fig.add_trace(go.Scatter3d(
    x=lx, y=ly, z=lz, mode="text",
    text=["<b>N</b>", "<b>S</b>"], textfont=dict(size=18, color="black"),
    hoverinfo="skip", showlegend=False))

# 7. Log colorbar with ticks labelled in real Tesla values.
tickvals = np.arange(np.floor(cmin), np.ceil(cmax) + 1)
fig.update_layout(
    title="Disc magnet 3 mm × 2 mm at origin — N faces +x — 3D field lines",
    coloraxis=dict(colorscale="Viridis", cmin=cmin, cmax=cmax,
                   colorbar=dict(title="|B| (T)", tickvals=tickvals,
                                 ticktext=[f"10<sup>{int(t)}</sup>" for t in tickvals],
                                 len=0.6)),
    scene=dict(
        xaxis=dict(title="x (mm)", range=[-6, 6]),
        yaxis=dict(title="y (mm)", range=[-6, 6]),
        zaxis=dict(title="z (mm)", range=[-6, 6]),
        aspectmode="cube"),
    margin=dict(l=0, r=0, t=40, b=0))

fig.write_html("magnet_3d.html", include_plotlyjs="cdn")
fig.show()
