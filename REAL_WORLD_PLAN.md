# From simulation to the real device

Fit the physical model to the real rig, then build the lookup table from that
fitted model. Ground truth comes from an **Xsens MTi-620 (VRU) on the shell**.

**Why no magnet-swap is needed:** MEMS accel/gyro are immune to magnetic fields,
so the onboard fusion never lets the magnet corrupt the pose:
- **roll & pitch** — from gravity, drift-free, read directly with the magnet in.
- **yaw** — not perturbed by the magnet. On a gyro-only (**VRU**) heading profile
  the magnetometer is ignored outright; on an **AHRS** profile the XKF3 filter
  treats the magnet as a magnetic **disturbance** and rejects it, trusting the
  gyro for heading. *Confirmed empirically on the MTi-630:* moving a magnet around
  the unit does not move the reported yaw. The cost either way is that yaw is
  effectively free-running gyro and slowly **drifts** — bound it by **re-zeroing
  at a mechanical home stop** every few poses and keeping sessions short (MTi gyro
  drift is small, <~1° over a few minutes).

> **Caveat — confirm with the magnet *mounted*, not just waved.** What's verified
> so far is rejection of a *moving* magnet (a transient). A permanently mounted
> magnet is a *sustained* field, which an AHRS profile could in principle slowly
> trust. Before relying on this: hold a fixed pose for **~60 s with the magnet in**
> and confirm yaw doesn't creep. If it does, switch the MTi to a **VRU/gyro-only**
> profile, which removes the magnetometer from the heading solution entirely.

Do the phases in order; don't advance until the **gate** passes.

| # | Phase | Gate |
|---|---|---|
| 0 | Hardware bring-up | readings stable after averaging |
| 1 | Test magnet mount | reinsertion spread < 0.1 mT |
| 2 | Calibrate sensors + IMU | magnet-out reading ≈ 0; yaw re-zero repeatable |
| 3 | Collect data (staged A→B→C) | Stage A sane; Stage B recovers angles |
| 4 | Fit the model | residual → sensor noise (~0.1–0.2 mT) |
| 5 | Build the table | \|B\| range matches measurement |
| 6 | Verify | median error within ~2× the sim's ~1° |

---

## Phase 0 — Hardware bring-up
- Both TLV493D on one I²C bus (two addresses); confirm Bx,By,Bz in mT.
- Mount the MTi-620 rigidly on the shell, its axes aligned to the joint frame.
  Stream orientation (roll/pitch/yaw) via MT Manager / the Xsens API.
- Set up a **home stop** at (0,0,0) — a repeatable mechanical detent used to
  re-zero the drifting yaw.
- Average **8–16 samples** per magnetic reading. Log **temperature**.

**Gate:** static magnetic readings stable to ≤ 0.05 mT; IMU roll/pitch to ≤ 0.2°.

## Phase 1 — Test the magnet mount (critical)
Everything assumes the magnet returns to the exact same pose each time.
- Build a **keyed/socketed** mount (not free-hand).
- Hold the shell fixed; record the 6 numbers, remove + reinsert the magnet,
  record again — **repeat 5–10×**.
- Compute each channel's spread (max − min).

**Gate:** spread < 0.1 mT (good) or < 0.3 mT (usable, ~1° extra error).
This number is the floor on your final accuracy.

## Phase 2 — Calibrate sensors + IMU
- **Sensor offset:** magnet out, record each TLV493D's reading as its offset `b0`;
  subtract from all later readings.
- **IMU:** no magnetometer calibration needed (the MTi-620 doesn't use it). Just
  verify the **yaw re-zero**: return to the home stop, reset yaw to 0, and check
  it reads ~0 each time you come back to the stop.

**Gate:** magnet-out reading ≈ 0 (≤ 0.1 mT); yaw reads ~0 at the home stop to
≤ 0.3° across repeats.

## Phase 3 — Collect calibration data
No magnet swapping — the magnet stays in the whole time (it doesn't affect the
MTi-620). Just manage yaw drift with the home stop.

**Per pose:**
1. **Re-zero yaw** at the home stop (do this every few poses to bound drift).
2. Move to a pose and **hold it fixed**.
3. Record `yaw,pitch,roll` from the MTi-620 **and** `B1x…B2z` from the two
   sensors (average 8–16) — at the same instant.
4. Every so often, return to the home stop and confirm yaw is still ~0; if it has
   drifted, re-zero and redo the poses since the last good re-zero.

**CSV** (`calibration_data.csv`):
```
pose_id, yaw_truth, pitch_truth, roll_truth, B1x,B1y,B1z, B2x,B2y,B2z, temp_C
```

**Collect in stages — don't record 50+ poses up front.** Prove the approach
cheaply, scale only if it works:

| stage | poses | fits | purpose |
|---|---|---|---|
| **A sanity** | 7 | nothing | catch gross bugs (sign/axis/units) |
| **B reduced** | ~19 | 9 params (offsets + magnet pos) | prove the pipeline end-to-end |
| **C full** | ~50 | all 24 params | production accuracy + trustworthy residual |

- **Stage A** — one axis at a time: `(0,0,0)`, `(±90,0,0)`, `(0,±25,0)`,
  `(0,0,±25)`. Compare the *nominal* model's prediction to the measurement.
  The `(0,0,±25)` roll pair is make-or-break: readings must actually change.
- **Stage B** — add range + combinations: yaw `±120,±60`; pitch/roll mids
  `±12`; corners `(±120,±25,±25)`, `(±60,∓25,±25)`. Keep 2–3 poses **held out**.
- **Stage C** — fill the interior and all 8 corners; keep a held-out set.

Rule of thumb: **~3–10× more readings (6/pose) than free parameters.**

**Gate:** Stage A predictions sane; Stage B `estimate()` roughly recovers angles.

## Phase 4 — Fit the model
Fit the geometry parameters so predictions match the data (same `least_squares`
engine as `estimate()`, but the unknowns are the geometry, not the angles):

| parameters | count | start (Stage) |
|---|---|---|
| sensor offsets (×2) | 6 | B |
| magnet position | 3 | B |
| magnet N–S tilt (2) + strength (1) | 3 | C |
| sensor position + orientation (×2) | 12 | C |

Minimize `Σ ‖predict_readings(p, pose_j) − measured_j‖²`, starting from the
nominal values in `simulation.py`. **Fix the gauge:** pivot at origin, home = zero;
trust magnet strength *or* sensor gain, not both. Note the true parameters need
not be recovered exactly — only a model that predicts the readings.

**Gate (health check):** RMS residual near sensor noise (~0.1–0.2 mT). Much
larger → something unmodeled (steel nearby, tilted magnet, bad offset). Fix it.

> Validated in simulation: a fake rig (magnet shifted 1 mm/tilted 2–3°, sensors
> mis-mounted with offsets) fit from 19 poses drops the residual 0.82 → 0.08 mT,
> and cuts estimation error from ~10° (nominal) to ~1.5° (fitted).

Output: `calibrated_geometry.json`.

## Phase 5 — Build the table
Load the fitted geometry, run `build_lookup_table.py` → `lookup_table.npz` from
the **calibrated** model. Nothing else in the estimator changes.

**Gate:** table builds; |B| range matches Phase 3 measurements.

## Phase 6 — Verify
On ~15–30 **fresh** poses (not used in the fit), run `estimate(measured)` and
compare to IMU truth. Report per-axis and worst-axis error (median, 95th, worst).

**Gate:** median worst-axis error within ~2× the simulated ~1°. Roll is the weak
axis; more averaging or a better mount helps it most.

## Deploy
Cold start uses the table; then track with `estimate(measured, seed=previous)`.
Average 4–8 samples per estimate; operate near the calibration temperature; keep
ferrous objects away.

---

## Scripts to build

| Script | Phase | Purpose |
|---|---|---|
| `mount_test.py` | 1 | log reinsertions, report per-channel spread |
| `log_calibration.py` | 3 | log MTi-620 pose + both sensors per pose → CSV |
| `predict_readings(params, pose)` | 4 | refactor `simulation.py` to make geometry fittable |
| `calibrate.py` | 4 | fit params, print residual, save `calibrated_geometry.json` |
| `verify.py` | 6 | estimate on held-out poses, report accuracy |

`simulation.py` / `build_lookup_table.py` / `estimation.py` stay the backbone;
calibration just swaps nominal geometry for fitted geometry.
