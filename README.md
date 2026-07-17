# Magnetic pose estimation

Estimates ball-joint yaw, pitch, and roll from two TLV493D magnetic sensors and
one fixed magnet. The final model reports yaw directly in the mechanical dial
frame; no runtime yaw correction is applied.

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
| Verification yaw MAE | 0.320° |
| Verification pitch MAE | 0.342° |
| Verification roll MAE | 0.602° |
| Mechanical dial yaw MAE | 0.468° |
| Runtime yaw offset | 0° |

Verification yaw removes smooth Xsens heading drift against the magnetic model,
so it measures repeatability rather than an independent absolute-yaw holdout.
The six mechanical dial poses provide the absolute-yaw check.

## Layout

```text
config/
  model.json             fitted model, geometry, pose convention, workspace
  sensor_offsets.json    six magnet-out sensor offsets
data/
  calibration.csv        original calibration recording
  verification.csv       original verification recording
magnetic_pose/            shared model, estimator, hardware, IMU, and plotting code
results/
  calibration_report.json
  dial_frame_check.json
  verification_report.json
tools/
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

The Raspberry Pi warning that the I²C frequency is not settable from Python is
expected and does not stop acquisition.
