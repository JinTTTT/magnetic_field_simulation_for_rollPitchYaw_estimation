# Magnetic pose estimation

Estimates ball-joint yaw, pitch, and roll from two TLV493D magnetic sensors and
one fixed magnet. The final model reports yaw directly in the mechanical dial
frame; no runtime yaw correction is applied. Pose estimation uses a fast
precomputed KD-tree lookup.

## Estimator mental model

Each pose produces six magnetic values: Bx, By, and Bz from each of the two
sensors. The lookup table contains the simulated six-channel field for 106,281
poses covering the calibrated workspace. Its resolution is 0.5 degrees in yaw
and 1 degree in pitch and roll.

At startup, the stored field vectors are indexed by a six-dimensional KD-tree.
For every measurement, the estimator finds the table entry with the nearest
six-channel field and directly returns that entry's yaw, pitch, and roll. No
iterative pose optimization runs during estimation. The output is therefore
quantized to the table resolution.

The reported model RMS is the root-mean-square difference between the measured
field and the selected table entry. It should be monitored as the fit/confidence
indicator. The physical forward model is only needed when rebuilding the table
after the rig model changes.

## Run

Magnetic estimate only:

```bash
env/bin/python live_estimation.py
```

Magnetic estimate beside the Xsens reference:

```bash
env/bin/python live_estimation_vs_imu.py
```

Keep the rig at mechanical home during the one-second Xsens startup zeroing.
The Xsens comparison is intended for short sessions because its yaw can drift
near the magnet.

Reproduce the saved verification result:

```bash
env/bin/python verify_model.py
```

## Final performance

| Measurement | Result |
|---|---:|
| Calibration field RMSE | 0.122 mT |
| Verification yaw MAE | 0.709° |
| Verification pitch MAE | 0.382° |
| Verification roll MAE | 0.701° |
| Mechanical dial yaw MAE | 0.468° |
| Runtime yaw offset | 0° |

Verification yaw removes smooth Xsens heading drift against the magnetic model,
so it measures repeatability rather than an independent absolute-yaw holdout.
The six mechanical dial poses provide the absolute-yaw check.

## Layout

```text
config/
  model.json             fitted model, geometry, pose convention, workspace
  pose_lookup.npz        model-fingerprinted 0.5°/1° field lookup table
  sensor_offsets.json    six magnet-out sensor offsets
data/
  calibration.csv        original calibration recording
  verification.csv       original verification recording
magnetic_pose/            shared model, lookup, hardware, IMU, and plotting code
results/
  calibration_report.json
  dial_frame_check.json
  verification_report.json
tools/
  build_lookup_table.py
  calibrate_sensor_offsets.py
  read_sensors.py
live_estimation.py
live_estimation_vs_imu.py
verify_model.py
```

All fitted geometry and operating bounds are together in `config/model.json`.
The pose convention is intrinsic `ZYX` in degrees, ordered yaw, pitch, roll.
The workspace is yaw ±60°, pitch ±10°, and roll ±10°.

## Maintenance tools

Read both magnetic sensors:

```bash
env/bin/python tools/read_sensors.py
```

Recalibrate offsets after removing the main magnet:

```bash
env/bin/python tools/calibrate_sensor_offsets.py --force
```

Rebuild the lookup table after refitting or changing the physical model:

```bash
env/bin/python tools/build_lookup_table.py --force
```

The lookup table stores a SHA-256 fingerprint of `model.json`; the runtime
rejects it if the model has changed and the table has not been rebuilt.

The Raspberry Pi warning that the I²C frequency is not settable from Python is
expected and does not stop acquisition.
