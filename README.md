# Magnetic orientation estimation

Estimates ball-joint yaw, pitch, and roll from one fixed magnet and two 3-axis
TLV493D sensors, using a fitted finite-cylinder physical model. An Xsens
MTi-630 provides pitch/roll labels during calibration; live estimation needs
the magnetic sensors only.

## Current status

The accepted model (`physical_model.json`) was fitted on 332 calibration poses
whose IMU yaw labels were first cleaned of heading wander (see below). Field
residual: 0.122 mT RMSE against 0.108 mT sensor noise. Accuracy on the 60-pose
verification set (heading-compensated labels): yaw MAE 0.32°, pitch 0.34°,
roll 0.60°; worst single error 2.3°. A 6-pose check against the mechanical
dial confirms absolute yaw within ±0.6° (the +60° edge pose is the weak spot
at ~1°).

**Key lesson:** the MTi-630 heading is magnetometer-aided and wanders several
degrees near the rig magnet (≈7.5° swing during the calibration hour). IMU
yaw is therefore not usable as an absolute reference over time. The pipeline
handles this with two artifacts:

- `yaw_zero_correction.json` — the constant rotation (+5.38°) between the
  model's calibration frame and the mechanical dial frame, measured from poses
  at known dial angles (`yaw_bias_diagnostic.csv`). `PhysicalModelEstimator`
  auto-loads it (SHA-locked to the model) and reports dial-frame yaw.
- `calibration_heading_wander.json` — the smooth-in-time heading error removed
  from the calibration labels before fitting (three compensate/refit
  iterations, converged).

The next calibration cycle should take yaw labels from the mechanical dial
directly, which makes both artifacts unnecessary. Pitch and roll from the IMU
are gravity-referenced and remain trustworthy.

The pre-refit model, correction, and manifest are kept as `*_v1_backup.*`;
copying them over the defaults rolls everything back.

## Pose convention

- Angle order yaw, pitch, roll; degrees; intrinsic `ZYX`.
- Mechanical home `(0, 0, 0)`; workspace yaw ±60°, pitch ±10°, roll ±10°
  (simultaneous pitch −10° and roll −10° is mechanically unreachable).
- Estimates are reported in the mechanical dial frame.

## Files

Pipeline scripts, in workflow order:

- `measure_sensor_offsets.py` — magnet-out static offsets at home
- `test_sensor_stability.py` — coherent-reader noise gate at two poses
- `measure_imu_yaw_reference.py` — home-pose `yaw0` for IMU display/labels
- `test_magnet_mount.py` — magnet reinstallation repeatability gate
- `record_calibration_data.py` — synchronized IMU + magnetic pose recorder
- `freeze_datasets.py` — locks dataset hashes/roles into `dataset_manifest.json`
- `fit_physical_model.py` — fits the model from the locked calibration data
- `compensate_calibration_yaw.py` — removes IMU heading wander from calibration
  yaw labels (iterate with refitting until converged)
- `fit_yaw_zero_correction.py` — measures the model-frame→dial-frame offset
  from poses at known dial angles
- `compensate_verification_yaw.py` — same wander removal for verification data
- `evaluate_physical_model.py` — evaluator; locked-holdout mode by default,
  `--data`/`--yaw-column` for other labeled sets
- `verify_model.py` — one command: compensate + evaluate the verification data
- `live_estimation.py` — live 3D pose from the magnetic sensors only
- `live_estimation_vs_imu.py` — live magnetic estimate next to the Xsens
  reference

Support: `physical_model.py` (forward model), `physical_estimator.py`
(inversion: coarse grid + bounded refinement + tracking),
`physical_model_fit_config.json` (bounds/priors/gate),
`geometry_priors.json`, `tools/` (TLV493D coherent reader, Xsens reader).

Data and evidence: `calibration_data.csv` (raw, original IMU labels),
`calibration_data_heading_compensated_iter3.csv` (fitting labels; manifest
entry), `verification_data.csv` (+ `_yaw_compensated`), `yaw_bias_diagnostic.csv`
(dial poses), sensor/mount/IMU raw sample files, and the fit/verification
reports named `physical_model_*`.

## Workflows

Hardware preparation (each writes raw samples + a JSON report, refuses to
overwrite without `--force`):

```bash
env/bin/python measure_sensor_offsets.py    # magnet removed, rig at home
env/bin/python test_sensor_stability.py     # magnet installed
env/bin/python measure_imu_yaw_reference.py # rig still at home
env/bin/python test_magnet_mount.py         # 5 reinstallation trials
```

Record poses (ENTER records one averaged pose, `q` quits; appends to
`calibration_data.csv`):

```bash
env/bin/python record_calibration_data.py
```

Fit and calibrate the frame:

```bash
env/bin/python freeze_datasets.py           # lock dataset hashes
env/bin/python fit_physical_model.py        # fit from locked calibration only
env/bin/python compensate_calibration_yaw.py  # then refit; repeat until stable
env/bin/python fit_yaw_zero_correction.py   # needs poses at known dial angles
```

Report performance on the verification data (compensation + evaluation):

```bash
env/bin/python verify_model.py
```

(`evaluate_physical_model.py` without `--data` is reserved for consuming a
future locked untouched holdout.)

Run live (magnetic-only, dial-frame output):

```bash
env/bin/python live_estimation.py
```

The status line shows the six-channel model RMS (~0.03 mT is healthy; a jump
means a wrong tracking minimum or a physical change — retry with
`--cold-start`). `live_estimation_vs_imu.py` additionally shows the Xsens
panel; its yaw comparison mixes frames (IMU heading wanders), so treat yaw
"error" there as an IMU-drift display, not model error.

All magnetic acquisition uses the shared coherent reader
(`tools/tlv493d_coherent.py`): a TLV493D frame is accepted only with `CHANNEL`
status 0, and one pre-trigger frame is discarded after every pose change.

See `transfer_to_real_plan.md` for the full process and next steps.
