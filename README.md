# Magnetic orientation estimation

This branch contains the clean rebuild of a system that estimates ball-joint
yaw, pitch, and roll from one fixed magnet and two 3-axis TLV493D sensors.

The earlier implementation proved that the approach works in practice. Its
code, fitted models, lookup table, and recorded datasets remain available on the
`master` branch. They are intentionally not used as calibration inputs here.

## Current status

The clean workflow is defined, and the directly measured rig geometry has been
separated from old fitted parameters. No new sensor calibration, model fit, or
lookup table exists yet on this branch.

The carried-over geometry in `geometry_priors.json` has been checked by the
user. The next task is the magnet-out sensor calibration.

## Files

- `REAL_WORLD_PLAN.md` — the ordered calibration and validation workflow
- `geometry_priors.json` — measured or nominal physical values used as fitting
  priors; it contains no fitted parameters
- `measure_sensor_offsets.py` — records the six static magnet-out offsets at
  mechanical home and preserves the raw samples
- `tools/read_tlv493d.py` — basic two-sensor hardware check
- `tools/xsens_mti630_reader.py` — low-level Xsens orientation reader

New scripts and data formats will be added one step at a time after the previous
step passes its gate.

## Pose convention

- Angle order: yaw, pitch, roll
- Units: degrees
- Rotation sequence: intrinsic `ZYX`
- Mechanical home pose: `(0, 0, 0)`
- IMU pitch and roll are used directly
- At the beginning of each session, the IMU yaw at mechanical home is averaged
  as `yaw0`; session yaw is `wrap180(yaw_raw - yaw0)`
- `yaw0` is not changed during a session

See `REAL_WORLD_PLAN.md` for the complete process.

## Measure static sensor offsets

Remove the main magnet, keep it far from the rig, and place the mechanism at
mechanical home. Then run:

```bash
env/bin/python measure_sensor_offsets.py
```

The default acquisition takes three batches of 128 samples. It writes every
sample to `sensor_offset_samples.csv` and writes the six means and stability
statistics to `sensor_offsets.json`. Existing outputs are not overwritten
unless `--force` is supplied.
