# Magnet field & rotary-sensor simulation

Simulates the magnetic field of a small disc magnet and a 3-axis sensor
(TLV493D-style) near it, using [magpylib](https://magpylib.readthedocs.io) for
the physics and [Plotly](https://plotly.com/python/) for interactive 3D views.

Magnet: 3 mm diameter × 2 mm tall disc, at the origin, N face → +x.

## Scripts

| Script | What it shows |
|---|---|
| `magnet_3d.py` | Traced 3D field lines of the magnet, colored by \|B\|. |
| `magnet_sensor.py` | Sensor orbits the magnet about z (full turn); B in the sensor frame. Both \|B\| and direction vary — decodes yaw. |
| `magnet_sensor_pitch.py` | Sensor fixed at `(0,-2,-1)`, pitched in place about y (−30°→+30°); B in the sensor frame. \|B\| constant, only the direction rotates — decodes pitch. |

Each script opens an interactive WebGL view in the browser. Drag to rotate,
scroll to zoom, hover a line/arrow to read the field values. Nothing is written
to disk — just run the script when you want to see it.

## Setup

```bash
python -m venv .venv
.venv/bin/pip install numpy scipy magpylib plotly
```

## Run

```bash
.venv/bin/python magnet_3d.py
.venv/bin/python magnet_sensor.py
.venv/bin/python magnet_sensor_pitch.py
```

## Notes

- The sensor reads the field on its own (rotating) axes:
  `B_sensor(θ) = R(θ)⁻¹ · B_world(pos(θ))`.
- **Orbit (yaw)** moves the sensor to different field points, so magnitude and
  direction both change. **Pitch in place** keeps the position fixed, so `|B|` is
  constant and only the direction rotates — cleanly decodable via the component
  ratio (amplitude-independent).
- x is the magnet's symmetry axis, so rotating the sensor *about that axis*
  (roll around the N–S line) is a symmetry: the reading never changes and roll is
  unobservable. Pitch and yaw are the two measurable rotational DOF.
