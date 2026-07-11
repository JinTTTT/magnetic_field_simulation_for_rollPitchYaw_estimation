# Magnet field & rotary-sensor simulation

Magnetic field of a small disc magnet and a 3-axis sensor (TLV493D-style) near
it, using [magpylib](https://magpylib.readthedocs.io) for the physics and
[Plotly](https://plotly.com/python/) for interactive 3D views.

Magnet: 3 mm × 2 mm disc at the origin, N face → +x.

## Scripts

| Script | What it shows |
|---|---|
| `magnet_3d.py` | 3D field lines, colored by \|B\|. |
| `magnet_sensor.py` | Sensor orbits about z (full turn); B in the sensor frame — decodes yaw. |
| `magnet_sensor_pitch.py` | Sensor fixed at `(0,-2,-1)`, pitched about y (±30°); B in the sensor frame — decodes pitch. |
| `estimate_yaw_pitch.py` | Simulate a raw `(Bx,By,Bz)` from known yaw/pitch/roll, then estimate yaw & pitch back from it. |

The plotting scripts open an interactive browser view (drag/scroll/hover);
nothing is saved to disk. `estimate_yaw_pitch.py` just prints.

## Setup & run

```bash
python -m venv .venv
.venv/bin/pip install numpy scipy magpylib plotly
.venv/bin/python magnet_3d.py        # or magnet_sensor.py, etc.
```

## Yaw / pitch / roll

Each is a rotation about one of the magnet's fixed axes (right-hand rule):

| Angle | Axis | Motion |
|---|---|---|
| **Yaw** | z | turn left/right (shake head "no") |
| **Pitch** | y | tip up/down (nod "yes") |
| **Roll** | x = N–S line | spin about the N–S line (tilt head to shoulder) |

Orientation used in code: `R = Rx(roll)·Ry(pitch)·Rz(yaw)`.

## Measuring the angle

At `(0,-2,-1)` the field points along the magnet's x-axis. The sensor reads that
fixed field's direction in its own frame, `(Bx,By,Bz)`:

```
yaw   = atan2(By, -Bx)               # bearing in the sensor's x-y plane
pitch = atan2(-Bz, hypot(Bx, By))    # tilt out of that plane
```

- **Roll spins around the field itself → invisible**, so it drops out and yaw/pitch
  are recovered for any roll.
- The formulas use component ratios, so they are independent of `|B|` (magnet
  strength, temperature).
- Holds while the sensor stays on the `x = 0` plane. Off it, the field tilts away
  from the roll axis and roll starts to matter.

## The invisible rotation (experimental finding)

From simulation experiments we found that **both magnet types have one invisible
rotation, and in both cases it is the rotation about the magnet's own N–S line**
(the magnetization direction **m**):

| Magnet | N–S line points | Invisible rotation |
|---|---|---|
| Axially magnetized cylinder (N on a flat face) | along the geometric axis (x here) | about x — exactly our roll |
| Diametrally magnetized disc (N on one side of the rim) | across the diameter (y here) | about y — the blindness doesn't vanish, it moves |

The reason is one symmetry law: *a magnet's field is rotationally symmetric
("round") about the line through its center along m*. Rotating the sensor
assembly about that line sweeps every sensor through identical field — the
readings never change, no matter how many sensors are used. Verified
numerically: the reading change under that rotation is exactly 0 (machine
precision) for both magnet types.

**The cure is the same for both: mount the magnet off-center**, shifted
perpendicular to m, so its symmetry line no longer passes through the pivot.
Then no rotation about any pivot axis leaves the field unchanged, and all three
angles become measurable. (Shifting *along* m does nothing — the symmetry line
must miss the pivot.) Two consequences worth remembering:

- Tilting a centered magnet does **not** help: the field's symmetry line tilts
  with it and still passes through the pivot; the blind axis just tilts too.
- Two crossed magnets at the same spot don't help either: two dipoles at one
  point sum to a single tilted dipole — still round about its own axis. The
  symmetry-breaking always comes from *displacement*, not orientation.

Design studies quantifying this (worst-case sensitivity over the workspace,
two sensors, 0.1 mT noise): `optimize_sensor_placement.py`,
`optimize_magnet_placement.py`,
`diametral_magnet_study.py`. Best found: diametral disc offset 0.45 mm along
the roll axis, sensors at the shell's ±y poles → worst-case error ≈ 0.1°.

## Potential future improvement: diametrally magnetized disc

Switch from the axially magnetized magnet to a **diametral disc** (N–S across
the diameter, as in commercial rotary encoders), still mounted off-center
(shift ⊥ m, e.g. along the roll axis). Simulated gains over the axial design:

- **Stronger worst-case signal** (0.97 vs 0.74 mT/°, worst-case error 0.10° vs
  0.14°): the field's two strong spots (along m, 2× the sideways strength) land
  *perpendicular* to the roll axis, where sensors can actually use them —
  for the axial magnet those spots lie on the roll axis, i.e. on the blind line.
- **Roll becomes a first-order signal**: the field direction rotates 1:1 with
  roll (compass-needle/encoder effect, ~2–4 mT/° over the full ±180°), instead
  of the faint offset-induced strength change (~0.6–0.8 mT/°). The offset is
  then only needed for pitch (rotation about m).
- **More even field over the workspace** (peak 433 vs 1346 mT): fits a real
  sensor's measuring range with much less headroom wasted — up to ~4× better
  worst-case error if constrained to a ±130 mT part.

Cost: the disc must be rotationally aligned (clocked) at assembly, since its
N–S direction matters; small errors are absorbed by calibration. Details and
numbers: `diametral_magnet_study.py`.
