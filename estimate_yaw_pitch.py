#!/usr/bin/env python
"""One sensor + centered magnet: yaw and pitch work, roll is invisible.

(README questions Q1 and Q2.) The magnet sits exactly at the pivot center,
N facing +x. Its field is perfectly round about its own N-S line, so rolling
the shell about that line carries the sensor through identical field -- the
reading cannot change, no matter how many sensors ride the shell.
Part 1 of the demo shows exactly that: sweep roll, the reading is frozen.

Yaw and pitch DO move the sensor to places with different field, so part 2
recovers those two angles from the 3 numbers of a single sensor -- and it
works for ANY roll, because roll has no effect on the numbers.

Same structure as estimate_yaw_pitch_roll.py (forward model, lookup table,
least-squares fine-tuning), only with a centered magnet, one sensor, and
two unknowns instead of three.
"""
import numpy as np
import magpylib as magpy
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares

# ---------------- the hardware ------------------------------------------------
magnet = magpy.magnet.Cylinder(polarization=(0, 0, 1.2), dimension=(10, 5))
magnet.rotate_from_angax(90, "y", anchor=(0, 0, 0))   # turn it so N faces +x
# NOTE: the magnet stays centered at (0,0,0) -- that is WHY roll is invisible.

SENSOR_HOME = np.array([0.0, -13.4, -6.7])  # sensor position at zero angles (15 mm)
SENSOR_NOISE = 0.1e-3                        # 0.1 mT of noise per axis


# ---------------- forward direction: angles -> reading -------------------------
def predict_reading(yaw, pitch, roll=0.0):
    """If the shell stands at these angles, what 3 numbers does the chip report?

    roll is accepted only so the demo can prove it has no effect."""
    rotation = Rotation.from_euler("zyx", [yaw, pitch, roll], degrees=True)
    # 1) the shell carries the sensor to a new position in space
    position = rotation.apply(SENSOR_HOME)
    # 2) the true magnetic field at that position (in world axes)
    field_world = magnet.getB(position)
    # 3) the chip reports the field along its own rotated axes
    field_chip = rotation.inv().apply(field_world)
    return field_chip                          # 3 numbers: Bx, By, Bz


def simulate(yaw, pitch, roll, noise=0.0):
    """A fake measurement: predict_reading plus optional sensor noise."""
    reading = predict_reading(yaw, pitch, roll)
    if noise > 0:
        reading = reading + np.random.default_rng().normal(0, noise, 3)
    return reading


# ---------------- lookup table over (yaw, pitch), built once -------------------
yaw_values = np.arange(-120, 121, 10)     # -120, -110, ..., +120  (25 values)
pitch_values = np.arange(-25, 26, 5)      # -25, -20, ..., +25     (11 values)

GRID_POSES = []                            # row i: [yaw, pitch]
GRID_READINGS = []                         # row i: the 3 numbers at that pose
for yaw in yaw_values:
    for pitch in pitch_values:
        GRID_POSES.append([yaw, pitch])
        GRID_READINGS.append(predict_reading(yaw, pitch))
GRID_POSES = np.array(GRID_POSES)
GRID_READINGS = np.array(GRID_READINGS)


# ---------------- backward direction: reading -> (yaw, pitch) ------------------
def estimate(measured, seed=None):
    """3 measured numbers -> (yaw, pitch) in degrees. Roll cannot be recovered."""

    # --- stage 1: pick starting guesses (skipped if the caller has one) -------
    if seed is not None:
        starting_guesses = [np.array(seed, dtype=float)]
    else:
        distances = np.zeros(len(GRID_POSES))
        for i in range(len(GRID_POSES)):
            distances[i] = np.linalg.norm(GRID_READINGS[i] - measured)
        closest_first = np.argsort(distances)
        starting_guesses = [GRID_POSES[closest_first[0]],
                            GRID_POSES[closest_first[1]],
                            GRID_POSES[closest_first[2]]]

    # --- stage 2: fine-tune from each guess, keep the best fit ----------------
    def mismatch(angles):
        yaw, pitch = angles
        return (predict_reading(yaw, pitch) - measured) * 1e3   # in mT

    best_angles = None
    best_cost = np.inf
    for guess in starting_guesses:
        result = least_squares(mismatch, guess, bounds=([-125, -30], [125, 30]))
        if result.cost < best_cost:
            best_cost = result.cost
            best_angles = result.x
    return best_angles


# ---------------- demo ---------------------------------------------------------
if __name__ == "__main__":
    print("Part 1 -- roll is INVISIBLE: hold yaw=20, pitch=10, sweep roll")
    print(f"{'roll':>6} | reading (Bx, By, Bz) in mT")
    for roll in (0, 45, 90, 180, 270):
        b = predict_reading(20, 10, roll) * 1e3
        print(f"{roll:6d} | {b[0]:9.4f} {b[1]:9.4f} {b[2]:9.4f}")
    print("-> identical numbers. No algorithm can recover roll from this.\n")

    print(f"Part 2 -- yaw & pitch still work ({SENSOR_NOISE*1e3} mT noise), any roll:")
    rng = np.random.default_rng(7)
    print(f"{'true y/p (roll)':>22} | {'estimated y/p':>16} | err (deg)")
    worst = 0.0
    for _ in range(8):
        yaw = rng.uniform(-120, 120)
        pitch = rng.uniform(-25, 25)
        roll = rng.uniform(-25, 25)          # unknown to the estimator!
        measured = simulate(yaw, pitch, roll, noise=SENSOR_NOISE)
        est_yaw, est_pitch = estimate(measured)
        err = max(abs(est_yaw - yaw), abs(est_pitch - pitch))
        worst = max(worst, err)
        print(f"{yaw:8.2f} {pitch:6.2f} ({roll:6.2f}) | {est_yaw:8.2f} {est_pitch:7.2f}"
              f" | {err:7.3f}")
    print(f"\nworst error: {worst:.3f} deg")
