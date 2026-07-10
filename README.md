# Magnet field & rotary-sensor simulation

Simulates the magnetic field of a small disc magnet and a 3-axis sensor orbiting
it, using [magpylib](https://magpylib.readthedocs.io) for the physics and
[Plotly](https://plotly.com/python/) for interactive 3D views.

Magnet: 3 mm diameter × 2 mm tall disc, at the origin, N face → +x.

## Scripts

| Script | What it does | Output |
|---|---|---|
| `magnet_3d.py` | Traced 3D field lines of the magnet, colored by \|B\|. | `magnet_3d.html` |
| `magnet_sensor.py` | A sensor at `(0, -2, -1)` rotates a full turn about the z-axis; records the B vector it reads in its own frame and plots it in 3D. | `magnet_sensor.html`, `magnet_sensor_readings.csv` |

Each field line / arrow is hoverable to read the field strength at that position.

## Setup

```bash
python -m venv .venv
.venv/bin/pip install numpy scipy magpylib plotly
```

## Run

```bash
.venv/bin/python magnet_3d.py
.venv/bin/python magnet_sensor.py
```

Both open an interactive WebGL view in the browser and also write a self-contained
`.html` file. Drag to rotate, scroll to zoom, hover to read values.

## Notes

- Rotating the sensor rigidly about z means its local axes turn with it, so the
  reading is `B_sensor(θ) = Rz(θ)⁻¹ · B_world(Rz(θ)·p₀)`.
- The sensor is placed off the z = 0 plane (`z = -1`) on purpose: an in-plane
  orbit is planar (`Bz = 0`) and has a yaw ambiguity near the pole crossing;
  going off-plane makes the reading fully 3D and uniquely decodable over a full turn.
