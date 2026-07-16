#!/usr/bin/env python
"""Build the lookup table: a grid of poses -> the 6 sensor readings.

This is the bridge between the simulation and the estimator. It sweeps a grid of
(yaw, pitch, roll) over the whole workspace, asks the forward model in
simulation.py what the two chips would read at each pose, and saves the result.

The estimator (estimation.py, next step) loads this table so that, given a real
measurement, it can find the nearest stored pose as a good starting guess before
fine-tuning with least_squares.

Run:
    .venv/bin/python build_lookup_table.py
writes lookup_table.npz  (poses, readings, and the grid axes).
"""
import time
import numpy as np

import simulation as sim

# ---- the grid over the device workspace (ranges come from simulation.py) -----
# coarse enough to build fast, fine enough that every real pose lands near a
# table entry (the estimator only needs a starting guess in the right basin).
YAW_VALUES   = np.arange(sim.YAW_RANGE[0], sim.YAW_RANGE[1] + 1, 10)
PITCH_VALUES = np.arange(sim.PITCH_RANGE[0], sim.PITCH_RANGE[1] + 1, 2)
ROLL_VALUES  = np.arange(sim.ROLL_RANGE[0], sim.ROLL_RANGE[1] + 1, 2)

TABLE_PATH = "lookup_table.npz"


def build_table(yaw_values=YAW_VALUES, pitch_values=PITCH_VALUES,
                roll_values=ROLL_VALUES):
    """Return (poses, readings): poses[i]=[yaw,pitch,roll], readings[i]=6 numbers."""
    poses, readings = [], []
    for yaw in yaw_values:
        for pitch in pitch_values:
            for roll in roll_values:
                poses.append((yaw, pitch, roll))
                readings.append(sim.predict_readings(yaw, pitch, roll))
    return np.array(poses, float), np.array(readings, float)


def save_table(path=TABLE_PATH):
    poses, readings = build_table()
    np.savez(path, poses=poses, readings=readings,
             yaw_values=YAW_VALUES, pitch_values=PITCH_VALUES,
             roll_values=ROLL_VALUES)
    return poses, readings


if __name__ == "__main__":
    t0 = time.time()
    poses, readings = save_table()
    dt = time.time() - t0

    # a quick sanity summary
    per_sensor_mag = np.linalg.norm(readings.reshape(-1, 2, 3), axis=2) * 1e3  # mT
    print(f"built {len(poses)} poses in {dt:.1f}s  ->  {TABLE_PATH}")
    print(f"  grid: yaw {YAW_VALUES[0]}..{YAW_VALUES[-1]} step "
          f"{YAW_VALUES[1]-YAW_VALUES[0]} ({len(YAW_VALUES)}),  "
          f"pitch/roll {PITCH_VALUES[0]}..{PITCH_VALUES[-1]} step "
          f"{PITCH_VALUES[1]-PITCH_VALUES[0]} ({len(PITCH_VALUES)}/{len(ROLL_VALUES)})")
    print(f"  readings array: {readings.shape}")
    verdict = ("inside the +-130 mT sensor range" if per_sensor_mag.max() < 130
               else "WARNING: exceeds the +-130 mT sensor range at some poses!")
    print(f"  per-sensor |B| range: {per_sensor_mag.min():.2f} .. "
          f"{per_sensor_mag.max():.2f} mT  ({verdict})")
