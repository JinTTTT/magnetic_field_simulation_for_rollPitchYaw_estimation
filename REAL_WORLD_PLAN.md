# Clean calibration and estimation plan

## Goal

Build a reproducible pipeline that estimates yaw, pitch, and roll from two
TLV493D magnetic sensors and one fixed magnet.

The existing implementation has already proved that the approach works. This
branch starts a new calibration cycle with new data and a clearly separated
workflow. Existing calibration files are treated as legacy results and are not
inputs to the new fit.

## Fixed conventions

- Angles are always stored and reported as `yaw, pitch, roll` in degrees.
- The pose convention is intrinsic `ZYX`.
- Magnetic fields are recorded in mT. The model may convert them to T internally.
- The mechanical home pose is `(yaw, pitch, roll) = (0, 0, 0)`.
- The operating workspace is yaw `[-60, +60]`, pitch `[-10, +10]`, and roll
  `[-10, +10]` degrees.
- The Xsens IMU continues to provide roll, pitch, and yaw.
- Roll and pitch are used directly from the IMU.
- Only yaw is referenced at startup: hold the rig at mechanical home, average
  the IMU yaw as `yaw0`, then use `wrap180(yaw_raw - yaw0)` for the whole session.
- `yaw0` is captured once at the beginning of a recording or live session and
  is never changed in the middle of that session.
- Calibration and verification data never share fitted parameters or residual
  correction training.
- Raw measurements are preserved. Corrections are applied during processing,
  not written back into raw data.
- Every TLV493D reading must pass the shared complete-frame check (`CHANNEL = 0`)
  before it can enter calibration, verification, fitting, or live estimation.

## Step-by-step workflow

### 1. Freeze the proof of concept

Keep the current working implementation and results as the feasibility
baseline. Develop the clean pipeline on this branch without changing the old
measurements.

**Output:** a known baseline that can always be reproduced and compared.

### 2. Record measured geometry as priors

Measure and record:

- Pivot and sensor positions
- Magnet position and dimensions
- Sensor mounting directions
- Magnet direction and approximate strength
- IMU mounting direction
- Measurement uncertainty for every dimension

These values are initial guesses and fitting priors, not fixed optimization
results. When a measured uncertainty is unavailable, define a conservative
fitting bound before optimizing that parameter.

**Output:** `geometry_priors.json`.

### 3. Measure the static magnetic offsets without the main magnet

Assume the magnet-out error is static and consistent in the six sensor
channels. Remove the main magnet, keep it far from the rig, and place the rig at
the mechanical `(0, 0, 0)` home pose.

1. Let both sensors settle.
2. Record several batches of raw readings without moving the rig.
3. Compute the mean and standard deviation of each of the six channels.
4. Save the six means as the static offsets.
5. For all later magnet-in data, use `corrected = raw - static_offset`.

Do not reuse the old offset file. The new raw samples, sample count, timestamp,
and per-channel standard deviations must be saved with the result.

**Output:** raw home-pose magnet-out samples and `sensor_offsets.json`.

**Gate:** repeated batches at home produce consistent means and acceptable
per-channel standard deviations.

### 4. Establish the IMU yaw reference

Place the rig at the mechanical `(0, 0, 0)` home pose and keep it still.

1. Average several IMU yaw samples to obtain `yaw0`.
2. Use `wrap180(yaw_raw - yaw0)` as yaw.
3. Use the IMU pitch and roll values directly.
4. Store `yaw0` and a session ID with every dataset.
5. Do not re-zero yaw during that session.

Repeat the startup procedure for each new recording or live session because the
physical home pose is the reference, not a permanent raw IMU heading value.

**Gate:** repeated starts at mechanical home report yaw close to zero and have
acceptable short-term drift.

### 5. Install the magnet and test mount repeatability

Install the magnet in its final keyed mount. At a fixed pose, remove and
reinstall it several times and compare all six sensor channels.

**Output:** a magnet-mount repeatability report.

**Gate:** reinsertion variation is below the chosen magnetic error budget.

### 6. Record a new calibration dataset

With the magnet installed:

1. Start at mechanical home and capture `yaw0` once.
2. Cover the full yaw, pitch, and roll workspace, including edges and combined
   rotations.
3. At each pose, record synchronized IMU angles and magnetic samples.
4. Save means, standard deviations, sample count, timestamps, temperature,
   session ID, and `yaw0`.

Only this new dataset is used to fit the physical model.

**Output:** a new raw calibration dataset.

### 7. Record untouched verification data

Record verification data in a separate acquisition session using the same
startup yaw procedure, units, and pose convention. Cover the full workspace and
include combined rotations.

Do not use these measurements for fitting, choosing priors, selecting residual
features, or tuning regularization.

**Output:** a new raw verification dataset.

### 8. Fit the physical model

For every calibration pose, compare the six measured magnetic channels with the
six channels predicted by the forward model. Optimize the physical parameters
to minimize that difference.

Fit appropriate parameters such as:

- Magnet position, direction, and effective strength
- Sensor positions and mounting rotations
- IMU-to-rig alignment terms if needed
- Sensor calibration and ambient-field terms not already fixed by Step 3

Measured geometry supplies the initial values and uncertainty-weighted prior
penalties. The calibration data determines the final fitted values.

**Output:** a fitted physical-model candidate and calibration residual report.

**Gate:** fitted values remain physically plausible and the field residual is
close to the measured noise level.

### 9. Fit the residual correction

After the physical model is fixed, fit a smooth pose-dependent correction to its
remaining six-channel error. Use only calibration data and select its
regularization with grouped cross-validation.

The correction must improve held-out calibration groups, not only training
error.

**Output:** a physical model plus residual-correction candidate.

### 10. Evaluate once on verification data

Lock the model and correction before opening the verification result. Report:

- Median, 95th percentile, and maximum error for yaw, pitch, and roll
- Median, 95th percentile, and maximum worst-axis error
- Magnetic-model residuals
- Results grouped by recording session and workspace region

If the model is changed after examining verification results, that verification
set becomes development data and a new untouched set must be recorded.

**Output:** the final verification report and an accepted or rejected model.

### 11. Build a versioned lookup table

Build the pose-to-field table only from the accepted model. Store model and
sensor-calibration identifiers with the table so stale combinations are
rejected automatically.

**Output:** a versioned `lookup_table.npz`.

### 12. Run live estimation

At startup:

1. Place the rig at mechanical home.
2. Capture `yaw0` once for the IMU comparison.
3. Load the accepted sensor calibration, physical model, residual correction,
   and matching lookup table.
4. Apply the same units, channel order, angle order, and rotation convention
   used during recording and fitting.
5. Estimate continuously, using the previous estimate as the tracking seed.

Display the magnetic estimate, IMU reference, per-axis error, and magnetic-model
residual.

## Work rule

Complete and review one step before implementing the next. Do not collect the
new magnet-in calibration dataset until the magnet-out sensor calibration and
IMU yaw convention have both passed their gates.
