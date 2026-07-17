# Transfer-to-real plan

Reproducible pipeline for estimating yaw/pitch/roll from two TLV493D sensors
and one fixed magnet on the real rig. Replaces the original REAL_WORLD_PLAN;
simplified after the first full cycle and updated with what that cycle taught.

## Fixed conventions

- Angles `yaw, pitch, roll` in degrees, intrinsic `ZYX`; fields in mT.
- Mechanical home `(0, 0, 0)`; workspace yaw ±60°, pitch ±10°, roll ±10°
  (pitch −10° together with roll −10° is unreachable and excluded).
- Estimates and truth labels live in the **mechanical dial frame**.
- IMU pitch/roll are gravity-referenced and used directly.
- **IMU yaw is not an absolute reference.** The MTi-630 heading is
  magnetometer-aided and wanders several degrees near the magnet (measured:
  ~7.5° swing over the 75-minute calibration session, ~3° within a 10-minute
  session). Prefer the dial for yaw labels; if IMU yaw labels are used they
  must be heading-compensated before fitting (step 6).
- Raw measurements are never modified; corrections happen in processing.
  Every TLV493D reading must pass the coherent-frame check (`CHANNEL = 0`).
- Calibration and verification datasets are frozen with SHA-256 manifests;
  fitting never opens verification data.

## Pipeline

Each step gates the next; each writes raw samples plus a JSON report.

1. **Geometry priors** — measured positions, dimensions, and uncertainties in
   `geometry_priors.json`. Initial guesses and prior widths, never results.
2. **Magnet-out sensor offsets** (`measure_sensor_offsets.py`) — six static
   channel means at home; later readings use `corrected = raw − offset`.
   Gate: consistent batch means, acceptable noise.
3. **Sensor stability** (`test_sensor_stability.py`) — per-channel stddev
   below limit at home and ~+90° yaw.
4. **IMU yaw reference** (`measure_imu_yaw_reference.py`) — home-pose `yaw0`
   so the IMU display reads ≈0 at home. Session-scoped: re-measure at the
   start of any session that compares against IMU yaw.
5. **Magnet mount repeatability** (`test_magnet_mount.py`) — reinstallation
   spread below the magnetic error budget.
6. **Record calibration** (`record_calibration_data.py`) — cover the
   workspace including edges and combined rotations; freeze with
   `freeze_datasets.py`; fit with `fit_physical_model.py`.
   Fit gate: converged, no parameter at a bound, priors respected, field RMSE
   near sensor noise.
   *If yaw labels came from the IMU:* run `compensate_calibration_yaw.py`
   and refit, repeating until the wander estimate stops changing (first cycle:
   3 iterations, final update 0.1° rms). *Next cycle: record the dial angle
   as the yaw label and skip this.*
7. **Dial-frame alignment** — a few poses at known dial angles let
   `fit_yaw_zero_correction.py` measure any constant model-frame→dial-frame
   offset. For the first-cycle IMU-labeled data,
   `align_calibration_yaw_to_dial.py` applies that offset to a derived dataset
   before the final refit. Run the dial check again afterward: the remaining
   mean offset should be near zero, with no runtime correction. This conversion
   is unnecessary when every calibration yaw is dial-labeled directly.
8. **Verification** (`verify_model.py`, wrapping
   `compensate_verification_yaw.py` + `evaluate_physical_model.py`) —
   separate session, never used in fitting. IMU-labeled sets need heading
   compensation first and then measure only pose-dependent + random error;
   absolute yaw needs dial-labeled poses.
9. **Live estimation** (`live_estimation.py`; `live_estimation_vs_imu.py` to
   compare against the IMU) — magnetic sensors only, dial-frame output,
   model-RMS health readout, tracking with global reacquisition fallback.

## Status after the first cycle (2026-07-17)

- Model fitted on 332 heading-compensated, dial-aligned poses: 0.122 mT RMSE
  (noise 0.108 mT); no runtime yaw offset.
- Verification (60 poses, compensated labels): yaw MAE 0.32°, pitch 0.34°,
  roll 0.60°; worst single error 2.3°.
- Dial check (6 poses): mean offset 0.00027°, yaw MAE 0.47°, worst 1.40° at
  the +60° edge.
- Rotated-frame artifacts kept as `*_model_frame_backup.*`; older artifacts
  remain as `*_v1_backup.*`.

## Next steps, in order of value

1. **Dial-labeled recalibration** — record the dial angle as yaw truth at
   every pose (IMU only for pitch/roll). Removes the heading problem at the
   source; expected yaw accuracy at the pitch/roll level (~0.3–0.6°).
2. **Residual correction** — fit a smooth pose-dependent correction on
   calibration residuals only (cross-validated); roll is now the weakest
   axis and the yaw station curve is a known target.
3. **Fresh untouched holdout** — required for any final accuracy claim; both
   existing verification sets are consumed development evidence.
4. Optional: versioned lookup table for faster startup or lookup-based
   estimation; not needed at current grid-build times (~seconds).
