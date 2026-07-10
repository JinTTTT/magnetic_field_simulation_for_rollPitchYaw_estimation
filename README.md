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
