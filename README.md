# Magnetic orientation estimation

This branch contains the clean rebuild of a system that estimates ball-joint
yaw, pitch, and roll from one fixed magnet and two 3-axis TLV493D sensors.

The earlier implementation proved that the approach works in practice. Its
code, fitted models, lookup table, and recorded datasets remain available on the
`master` branch. They are intentionally not used as calibration inputs here.

## Current status

The coherent TLV493D reader and stability test, magnet-out offsets, fixed IMU
yaw reference, and magnet-mount repeatability test have passed. The locked
datasets contain 332 calibration poses and 60 verification poses.
The unreachable simultaneous pitch `-10` and roll `-10` corner is documented
rather than treated as missing data.

The calibration-only finite-cylinder physical fit has passed its gate at
`0.126585 mT` RMSE, compared with `0.108457 mT` median measured channel sample
noise. The optimizer converged, no parameter reached a bound, all prior
displacements remained below four stated prior standard deviations, and the
fitter did not load verification measurements.

The physical model has now been evaluated once on the 60-pose verification
set, without residual correction. Its mean absolute angle errors are
`2.037 deg` yaw, `0.315 deg` pitch, and `0.602 deg` roll. The per-axis 95th
percentile absolute errors are `3.606 deg`, `0.904 deg`, and `1.451 deg`,
respectively. This verification set is now consumed as physical-baseline
development evidence. After residual correction is fixed, final evaluation
requires a newly recorded untouched holdout set.

## Files

- `REAL_WORLD_PLAN.md` — the ordered calibration and validation workflow
- `geometry_priors.json` — measured or nominal physical values used as fitting
  priors; it contains no fitted parameters
- `measure_sensor_offsets.py` — records the six static magnet-out offsets at
  mechanical home and preserves the raw samples
- `measure_imu_yaw_reference.py` — records stationary home-pose IMU samples and
  saves their circular mean yaw as `yaw0`
- `test_magnet_mount.py` — measures corrected home fields across repeated magnet
  installations and reports their per-channel spread
- `record_calibration_data.py` — interactively records synchronized IMU poses
  and raw/corrected magnetic measurements for physical-model fitting
- `freeze_datasets.py` — records checksums, row counts, and strict calibration
  versus holdout roles in `dataset_manifest.json`
- `physical_model_fit_config.json` — explicit optimization bounds, prior
  uncertainties, weighting, and acceptance thresholds
- `fit_physical_model.py` — fits the finite-cylinder model using only the locked
  calibration input
- `physical_model.py` — reusable forward prediction from the fitted model
- `physical_model.json` — accepted calibration-only physical-model parameters
- `physical_model_calibration_report.json` — convergence, prior, and residual
  evidence for the physical-model gate
- `physical_estimator.py` — inverts the fitted model with a coarse global
  search, bounded refinement, and live tracking
- `evaluate_physical_model.py` — reproducible physical-only verification
  evaluator
- `physical_model_verification_report.json` — saved 60-pose baseline accuracy
- `live_3d.py` — live side-by-side magnetic estimate and Xsens reference
- `tools/tlv493d_coherent.py` — shared frame-validated TLV493D-A1B6 reader
- `test_sensor_stability.py` — compares coherent magnetic noise at home and
  approximately +90° yaw before accepting new calibration measurements
- `tools/read_tlv493d.py` — basic two-sensor hardware check
- `tools/xsens_mti630_reader.py` — low-level Xsens orientation reader

All magnetic acquisition scripts use the shared coherent reader. A TLV493D
register snapshot is accepted only when its `CHANNEL` status is zero, indicating
that Bx, By, and Bz belong to one complete measurement frame. After every pose
change, one coherent pre-trigger frame is discarded and the code waits for a
fresh conversion before averaging begins.

## Pose convention

- Angle order: yaw, pitch, roll
- Units: degrees
- Rotation sequence: intrinsic `ZYX`
- Mechanical home pose: `(0, 0, 0)`
- Operating workspace: yaw `[-60, +60]`, pitch `[-10, +10]`, and roll
  `[-10, +10]` degrees
- The workspace is not a full Cartesian box: simultaneous pitch `-10` and roll
  `-10` degrees is mechanically unreachable and is excluded.
- IMU pitch and roll are used directly
- The accepted home measurement in `imu_yaw_reference.json` supplies `yaw0` to
  recording, evaluation, and live comparison
- Referenced yaw is `wrap180(yaw_raw - yaw0)`; `yaw0` is not recaptured during
  normal use of this model

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

## Validate coherent sensor stability

With the magnet installed, run:

```bash
env/bin/python test_sensor_stability.py
```

Follow the prompts for mechanical home and approximately +90° yaw. The test
preserves magnetic values, frame counters, retry counts, and raw register bytes.
Both poses must remain below the default 0.2 mT per-channel standard-deviation
limit before recording new offsets, mount results, or calibration poses.

## Measure the IMU yaw reference

Keep the rig stationary at mechanical home and run:

```bash
env/bin/python measure_imu_yaw_reference.py
```

The script records 200 fresh Xsens samples by default. It writes the raw
orientations to `imu_home_samples.csv` and the fixed home yaw reference to
`imu_yaw_reference.json`. Pitch and roll are reported but are not calibrated.

## Test the magnet mount

Run the test with the magnet initially removed:

```bash
env/bin/python test_magnet_mount.py
```

For each of five trials, install the magnet, place the rig at mechanical home,
and press ENTER. Remove the magnet again before the next trial. The script loads
`sensor_offsets.json`, preserves all readings in `magnet_mount_samples.csv`, and
writes the corrected-field repeatability report to `magnet_mount_test.json`.
The default acceptance limit is 0.1 mT on every channel.

## Record calibration poses

Leave the accepted magnet installation in place and run:

```bash
env/bin/python record_calibration_data.py
```

The recorder loads `sensor_offsets.json` and `imu_yaw_reference.json`, displays
the current yaw/pitch/roll continuously, and records one synchronized averaged
pose whenever ENTER is pressed. Type `q` then ENTER to stop. Data is appended to
`calibration_data.csv`; an existing file must have the exact current schema.

## Freeze datasets and fit the physical model

After calibration and untouched verification acquisition are complete, freeze
their exact bytes and supporting inputs:

```bash
env/bin/python freeze_datasets.py
```

Then fit only the locked calibration dataset:

```bash
env/bin/python fit_physical_model.py
```

The fitter verifies the calibration and geometry hashes. It reads the holdout
entry only to enforce its `untouched_final_evaluation_only` policy and never
opens `verification_data.csv`. Existing fit outputs are protected unless
`--force` is supplied. The accepted runtime parameters are written to
`physical_model.json`; diagnostics and calibration predictions remain separate.

## Run the current model live in 3D

Keep the accepted magnet installation in place and connect both TLV493D sensors
and the Xsens IMU. From the repository root, run:

```bash
env/bin/python live_3d.py
```

The left panel shows the physical-model magnetic estimate and the right panel
shows the Xsens reference. The bottom line reports instantaneous absolute yaw,
pitch, and roll errors plus the six-channel model RMS. The program loads the
locked sensor offsets and the saved `yaw0`; it does not silently recapture or
change either reference. The Xsens panel uses its latest raw orientation sample;
only the fixed `yaw0` subtraction is applied to Xsens yaw. Close the window or
press Ctrl-C to stop.

The default averages eight magnetic frames per update. For a faster but noisier
display, use `--samples 4`. If tracking ever appears stuck after moving the rig
abruptly, run once with `--cold-start` to force a global search on every frame.
