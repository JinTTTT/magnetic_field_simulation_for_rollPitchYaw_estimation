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
Each new six-channel sensor measurement is offset-corrected and passed through
an exponential moving average with alpha 0.2. The estimator immediately finds
the table entry with the nearest filtered field and returns that entry's yaw,
pitch, and roll. No block-average acquisition or iterative pose optimization
runs during estimation. The output is quantized to the table resolution.

The reported model RMS is the root-mean-square difference between the measured
field and the selected table entry. It should be monitored as the fit/confidence
indicator. The physical forward model is only needed when rebuilding the table
after the rig model changes.

## Run

Magnetic estimate only:

```bash
env/bin/python live_estimation.py
```

The EMA can be tuned at runtime. A larger alpha responds faster but passes more
sensor noise; a smaller alpha is smoother but adds more lag:

```bash
env/bin/python live_estimation.py --ema-alpha 0.2
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

Record four raw comparison sessions separately. Keep the rig at mechanical home
during the one-second yaw zero at the beginning of every command, then perform
the named movement. Each command opens the same live magnetic/Xsens comparison
figure, so it can be captured with a screen recorder. Close the figure to finish
and save the CSV:

```bash
env/bin/python record_raw_session.py recordings/yaw.csv
env/bin/python record_raw_session.py recordings/roll.csv
env/bin/python record_raw_session.py recordings/pitch.csv
env/bin/python record_raw_session.py recordings/random.csv
```

Every row uses the magnetic read time and contains only `t_s`, zeroed Xsens yaw,
Xsens pitch and roll, and the six raw magnetic channels in mT. Use `--duration`
for a fixed recording length or `--force` to replace an existing file. A fixed
duration stops CSV recording while leaving the comparison figure open.

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
magnetic_pose/            shared model, lookup, filtering, hardware, IMU, and plotting code
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
record_raw_session.py
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
