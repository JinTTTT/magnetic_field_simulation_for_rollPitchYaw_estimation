#!/usr/bin/env python
"""Estimate yaw, pitch and roll from the two sensors' 6 numbers.

The inverse of simulation.py's forward model. It works in two stages:

  stage 1 -- coarse guess from the lookup table:
      compare the measurement against every stored reading in lookup_table.npz
      and take the closest few poses. This lands us in the right neighbourhood
      (and dodges "look-alike" poses that a single closest match might hit).

  stage 2 -- fine-tune with least_squares:
      starting from each coarse guess, adjust (yaw, pitch, roll) until the
      forward model's predicted readings match the measurement. Keep the best fit.

The forward model and the hardware live in simulation.py; the table is built by
build_lookup_table.py. Run this file for a noise-in / accuracy-out demo:

    .venv/bin/python build_lookup_table.py   # once, if the table is missing
    .venv/bin/python estimation.py
"""
import os

import numpy as np
from scipy.optimize import least_squares

import simulation as sim
from build_lookup_table import TABLE_PATH, save_table

# ---- load the lookup table (build it if it is not there yet) ------------------
if not os.path.exists(TABLE_PATH):
    print(f"{TABLE_PATH} not found -- building it now...")
    save_table()
_table = np.load(TABLE_PATH)
GRID_POSES = _table["poses"]        # (N, 3) [yaw, pitch, roll]
GRID_READINGS = _table["readings"]  # (N, 6)

# search bounds for least_squares: the workspace plus a small margin
MARGIN = 5
LOWER = [sim.YAW_RANGE[0] - MARGIN, sim.PITCH_RANGE[0] - MARGIN, sim.ROLL_RANGE[0] - MARGIN]
UPPER = [sim.YAW_RANGE[1] + MARGIN, sim.PITCH_RANGE[1] + MARGIN, sim.ROLL_RANGE[1] + MARGIN]


def estimate(measured, seed=None, n_starts=3):
    """6 measured numbers -> (yaw, pitch, roll) in degrees.

    If a good guess is known (e.g. the previous frame in a tracking loop), pass
    it as seed to skip the table lookup.
    """
    measured = np.asarray(measured, float)

    # --- stage 1: starting guesses ---
    if seed is not None:
        starts = [np.asarray(seed, float)]
    else:
        distances = np.linalg.norm(GRID_READINGS - measured, axis=1)
        closest = np.argsort(distances)[:n_starts]
        starts = [GRID_POSES[i] for i in closest]

    # --- stage 2: fine-tune from each guess, keep the best fit ---
    def residual(angles):
        # 6 differences, in mT, between prediction and measurement
        return (sim.predict_readings(*angles) - measured) * 1e3

    best_angles, best_cost = None, np.inf
    for guess in starts:
        result = least_squares(residual, guess, bounds=(LOWER, UPPER))
        if result.cost < best_cost:
            best_cost = result.cost
            best_angles = result.x
    return best_angles


# ---- demo: simulate noisy measurements, estimate, compare --------------------
if __name__ == "__main__":
    rng = np.random.default_rng(7)
    print(f"geometry: magnet at ({', '.join(f'{v:g}' for v in sim.MAGNET_CENTER)}), "
          f"2 sensors r={sim.SENSOR_RADIUS:.0f} mm, 120 deg apart")
    print(f"sensor noise: {sim.SENSOR_NOISE*1e3:.1f} mT per axis\n")
    print(f"{'true y/p/r':>22} | {'estimated y/p/r':>24} | err (deg)")

    errors = []
    for _ in range(10):
        yaw = rng.uniform(*sim.YAW_RANGE)
        pitch = rng.uniform(*sim.PITCH_RANGE)
        roll = rng.uniform(*sim.ROLL_RANGE)

        measured = sim.simulate(yaw, pitch, roll, noise=sim.SENSOR_NOISE)
        est = estimate(measured)

        err = max(abs(est[0] - yaw), abs(est[1] - pitch), abs(est[2] - roll))
        errors.append(err)
        print(f"{yaw:7.2f} {pitch:7.2f} {roll:7.2f} | "
              f"{est[0]:8.2f} {est[1]:8.2f} {est[2]:8.2f} | {err:7.3f}")

    print(f"\nworst of these 10: {max(errors):.3f} deg")
