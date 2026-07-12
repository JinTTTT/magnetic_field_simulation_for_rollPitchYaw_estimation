#!/usr/bin/env python
"""Estimate yaw, pitch AND roll from two 3-axis magnetic sensor readings.

The idea (see README Q3):
  - The magnet is glued 3 mm OFF-CENTER. A centered magnet's field is
    perfectly round about its N-S line, so roll would be invisible.
  - TWO sensors are used. One sensor alone has poses where some angle
    combination barely changes its reading; the second sensor covers those.
    Their positions (the shell's +z and -z poles) come from
    optimize_sensor_placement.py.

The program has two halves:
  predict_readings(yaw, pitch, roll) -- the "forward" direction:
      given the angles, compute the 6 numbers the chips would report.
  estimate(measured)                 -- the "backward" direction:
      given 6 measured numbers, find the angles that explain them.
      It works in two stages:
        stage 1: compare the measurement against a precomputed lookup table
                 to find a rough starting guess (right neighborhood),
        stage 2: fine-tune with least_squares until predicted == measured.

Workspace of the device: yaw -120..+120 deg, pitch and roll -25..+25 deg.
"""
import numpy as np
import magpylib as magpy
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares

# ---------------- the hardware, same numbers as the README --------------------
magnet = magpy.magnet.Cylinder(polarization=(0, 0, 1.2), dimension=(10, 5))
magnet.rotate_from_angax(90, "y", anchor=(0, 0, 0))   # turn it so N faces +x
magnet.position = (0, 3.0, 0)                         # 3 mm off-center (the trick)

# sensor positions at zero angles: the shell's -z and +z poles (optimized
# placement for this workspace, found by optimize_sensor_placement.py).
# 15 mm from the pivot: the Adafruit TLV493D breakout is 25 mm wide, so the
# chip cannot ride much closer -- and at 15 mm the field stays well inside
# the sensor's +-130 mT range.
SENSOR_1_HOME = np.array([0.0, 0.0, -15.0])
SENSOR_2_HOME = np.array([0.0, 0.0,  15.0])

SENSOR_NOISE = 0.1e-3                          # 0.1 mT of noise per axis


# ---------------- forward direction: angles -> readings ------------------------
def predict_readings(yaw, pitch, roll):
    """If the shell stands at these angles, what 6 numbers do the chips report?"""
    # the rotation of the whole shell (sensors ride on it)
    rotation = Rotation.from_euler("zyx", [yaw, pitch, roll], degrees=True)

    readings = []
    for home in (SENSOR_1_HOME, SENSOR_2_HOME):
        # 1) the shell carries the sensor to a new position in space
        position = rotation.apply(home)
        # 2) the true magnetic field at that position (in world axes)
        field_world = magnet.getB(position)
        # 3) the chip's axes rotated with the shell, so it reports the field
        #    rotated back into its own frame
        field_chip = rotation.inv().apply(field_world)
        # collect this chip's three numbers (Bx, By, Bz)
        readings.extend(field_chip)

    return np.array(readings)          # 6 numbers: [B1x B1y B1z B2x B2y B2z]


def simulate(yaw, pitch, roll, noise=0.0):
    """Like predict_readings, but optionally adds sensor noise (a fake 'measurement')."""
    readings = predict_readings(yaw, pitch, roll)
    if noise > 0:
        readings = readings + np.random.default_rng().normal(0, noise, 6)
    return readings


# ---------------- the lookup table, built once at start ------------------------
# For a grid of sample poses covering the whole workspace, precompute what the
# sensors would read. Later, a measurement is compared against this table to
# find a rough starting guess.
yaw_values   = np.arange(-120, 121, 10)   # -120, -110, ..., +120  (25 values)
pitch_values = np.arange(-25, 26, 5)      # -25, -20, ..., +25     (11 values)
roll_values  = np.arange(-25, 26, 5)      #                        (11 values)

GRID_POSES = []                            # row i: [yaw, pitch, roll]
GRID_READINGS = []                         # row i: the 6 numbers at that pose
for yaw in yaw_values:
    for pitch in pitch_values:
        for roll in roll_values:
            GRID_POSES.append([yaw, pitch, roll])
            GRID_READINGS.append(predict_readings(yaw, pitch, roll))
GRID_POSES = np.array(GRID_POSES)          # 25*11*11 = 3025 rows
GRID_READINGS = np.array(GRID_READINGS)


# ---------------- backward direction: readings -> angles -----------------------
def estimate(measured, seed=None):
    """6 measured numbers -> (yaw, pitch, roll) in degrees.

    If you already have a good guess (e.g. the previous frame's answer in a
    real-time loop), pass it as seed and stage 1 is skipped."""

    # --- stage 1: pick starting guesses ---------------------------------------
    if seed is not None:
        starting_guesses = [np.array(seed, dtype=float)]
    else:
        # how far is the measurement from each stored reading in the table?
        distances = np.zeros(len(GRID_POSES))
        for i in range(len(GRID_POSES)):
            difference = GRID_READINGS[i] - measured
            distances[i] = np.linalg.norm(difference)
        # take the 3 closest table entries as starting guesses (3, not 1,
        # in case the single closest one is in a wrong "look-alike" region)
        closest_first = np.argsort(distances)
        starting_guesses = [GRID_POSES[closest_first[0]],
                            GRID_POSES[closest_first[1]],
                            GRID_POSES[closest_first[2]]]

    # --- stage 2: fine-tune from each guess, keep the best fit ----------------
    def mismatch(angles):
        """How wrong is this angle guess? (6 differences, in mT)"""
        yaw, pitch, roll = angles
        predicted = predict_readings(yaw, pitch, roll)
        return (predicted - measured) * 1e3

    best_angles = None
    best_cost = np.inf
    for guess in starting_guesses:
        # least_squares turns the three angle "knobs" step by step until the
        # mismatch is as small as possible (see README / notes)
        result = least_squares(mismatch, guess,
                               bounds=([-125, -30, -30], [125, 30, 30]))
        if result.cost < best_cost:      # result.cost = how much mismatch is left
            best_cost = result.cost
            best_angles = result.x
    return best_angles


# ---------------- demo: simulate with noise, estimate, compare -----------------
if __name__ == "__main__":
    rng = np.random.default_rng(7)
    print(f"sensor noise: {SENSOR_NOISE*1e3} mT per axis\n")
    print(f"{'true y/p/r':>22} | {'estimated y/p/r':>24} | err (deg)")

    worst = 0.0
    for _ in range(10):
        # a random true pose inside the workspace
        yaw = rng.uniform(-120, 120)
        pitch = rng.uniform(-25, 25)
        roll = rng.uniform(-25, 25)

        measured = simulate(yaw, pitch, roll, noise=SENSOR_NOISE)
        est_yaw, est_pitch, est_roll = estimate(measured)

        err = max(abs(est_yaw - yaw), abs(est_pitch - pitch), abs(est_roll - roll))
        worst = max(worst, err)
        print(f"{yaw:7.2f} {pitch:7.2f} {roll:7.2f} | "
              f"{est_yaw:8.2f} {est_pitch:8.2f} {est_roll:8.2f} | {err:7.3f}")

    print(f"\nworst error: {worst:.3f} deg")
