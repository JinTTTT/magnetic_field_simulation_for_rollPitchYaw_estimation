#!/usr/bin/env python
"""Phase 4: fit the physical model to the recorded calibration data.

Reads calibration_data.csv (IMU truth + sensor readings per pose, from
log_calibration.py) and sensor_offsets.json (magnet-out zero offsets), then
fits the geometry so the forward model predicts the measurements:

    parameters (14):
      field scale (1)      -- magnet strength, sign = polarity (can be flipped)
      magnet position (3)
      magnet tilt y,z (2)  -- N-S axis deviation from +x
      chip rotation  (3+3) -- how each TLV493D die is oriented on its bus
      IMU mount err  (2)   -- residual pitch/roll misalignment of the IMU
                              (no yaw term: the magnet axis IS the yaw reference)

    fixed (the gauge): sensor positions at their nominal ring spots, pivot at
    the origin. The fit does not need the true values -- only a model that
    predicts the readings.

Writes calibrated_geometry.json. simulation.py picks that file up
automatically, so afterwards rebuild the table and everything downstream
uses the calibrated model:

    env/bin/python calibrate.py
    env/bin/python build_lookup_table.py
"""
import json

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import magpylib as magpy

import simulation as sim

DATA_PATH = "calibration_data.csv"
OFFSETS_PATH = "sensor_offsets.json"
OUT_PATH = "calibrated_geometry.json"

NOMINAL_POLARIZATION = 1.2      # T, the datasheet value the scale multiplies


def load_data():
    rows = np.genfromtxt(DATA_PATH, delimiter=",", skip_header=1)[:, :10]
    with open(OFFSETS_PATH) as f:
        off = json.load(f)
    b0 = np.array(off["S1"] + off["S2"])
    poses = rows[:, 1:4]                    # yaw, pitch, roll (deg, IMU truth)
    meas = rows[:, 4:10] - b0               # mT, zero offsets removed
    return poses, meas


def build_magnet(scale, position, tilt_y, tilt_z):
    magnet = magpy.magnet.Cylinder(
        polarization=(0, 0, NOMINAL_POLARIZATION * scale), dimension=(10, 5))
    magnet.rotate_from_angax(90, "y", anchor=(0, 0, 0))     # N-S along +x
    magnet.rotate_from_angax(tilt_y, "y", anchor=(0, 0, 0))
    magnet.rotate_from_angax(tilt_z, "z", anchor=(0, 0, 0))
    magnet.position = position
    return magnet


def predict(x, pose_list):
    """Forward model with parameters x -> (n, 6) readings in mT."""
    magnet = build_magnet(x[0], x[1:4], x[4], x[5])
    chip_rots = (Rotation.from_rotvec(x[6:9]), Rotation.from_rotvec(x[9:12]))
    imu_err = Rotation.from_rotvec([x[12], x[13], 0.0])
    out = []
    for pose in pose_list:
        rot = Rotation.from_euler("zyx", pose, degrees=True) * imu_err
        row = []
        for home, chip_rot in zip(sim.SENSOR_HOMES, chip_rots):
            field_world = magnet.getB(rot.apply(home)) * 1e3      # mT
            row.extend(chip_rot.apply(rot.inv().apply(field_world)))
        out.append(row)
    return np.array(out)


def fit(poses, meas):
    """Multi-start least squares (chip orientation basins need seeding)."""
    def resid(x):
        return (predict(x, poses) - meas).ravel()

    best = None
    for scale0 in (1.0, -1.0):              # polarity unknown a priori
        for z1 in (-90, 0, 90, 180):
            for z2 in (-90, 0, 90, 180):
                x0 = np.r_[scale0, 0.0, 0.0, sim.MAGNET_Z, 0.0, 0.0,
                           Rotation.from_euler("zyx", (z1, -70, 0),
                                               degrees=True).as_rotvec(),
                           Rotation.from_euler("zyx", (z2, 30, -66),
                                               degrees=True).as_rotvec(),
                           0.0, 0.0]
                r = least_squares(resid, x0)
                if best is None or r.cost < best.cost:
                    best = r
    rms = np.sqrt(np.mean(resid(best.x) ** 2))
    worst = np.abs(predict(best.x, poses) - meas).max()
    return best.x, rms, worst


def main():
    poses, meas = load_data()
    print(f"fitting {len(poses)} poses ({meas.size} readings, 14 parameters)...")
    x, rms, worst = fit(poses, meas)

    print(f"\nRMS residual {rms:.3f} mT   worst channel {worst:.3f} mT")
    print(f"field scale {x[0]:+.3f}  "
          f"({'polarity FLIPPED, ' if x[0] < 0 else ''}"
          f"|P| = {abs(NOMINAL_POLARIZATION * x[0]):.2f} T)")
    print(f"magnet position ({x[1]:+.2f}, {x[2]:+.2f}, {x[3]:.2f}) mm")
    print(f"magnet tilt y/z {x[4]:+.1f} / {x[5]:+.1f} deg")
    for name, rv in (("S1", x[6:9]), ("S2", x[9:12])):
        eul = Rotation.from_rotvec(rv).as_euler("xyz", degrees=True)
        print(f"{name} chip rotation (xyz euler) {np.round(eul, 1)} deg")
    print(f"IMU mount err pitch/roll "
          f"{np.degrees(x[13]):+.2f} / {np.degrees(x[12]):+.2f} deg")

    out = {
        "comment": "fitted by calibrate.py; loaded by simulation.py",
        "polarization": NOMINAL_POLARIZATION * x[0],
        "magnet_position": list(x[1:4]),
        "tilt_y": x[4],
        "tilt_z": x[5],
        "chip_rotvec_S1": list(x[6:9]),
        "chip_rotvec_S2": list(x[9:12]),
        "imu_mount_rotvec": [x[12], x[13], 0.0],
        "rms_residual_mT": rms,
        "n_poses": len(poses),
    }
    with open(OUT_PATH, "w") as f:
        json.dump(out, f, indent=2)
    print(f"\nwrote {OUT_PATH} -- now rebuild the table:  "
          f"env/bin/python build_lookup_table.py")


if __name__ == "__main__":
    main()
