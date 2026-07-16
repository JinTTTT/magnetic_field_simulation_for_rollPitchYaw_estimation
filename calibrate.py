#!/usr/bin/env python3
"""Fit baseline and extended physical models to recorded calibration poses.

The baseline model preserves the original 14 parameters. The extended model
adds yaw alignment, residual channel biases, relative channel gains, and a
world-frame ambient magnetic field. Fits are bounded and robust by default.

The default command writes calibrated_geometry_candidate.json, leaving the
active calibrated_geometry.json untouched:

    env/bin/python calibrate.py
"""
import argparse
import json
import os

import magpylib as magpy
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

import simulation as sim
from field_correction import (
    DEFAULT_ALPHAS, fit_correction, predict_correction,
)

DATA_PATH = "calibration_data.csv"
OFFSETS_PATH = "sensor_offsets.json"
VERIFY_PATH = "verification_data.csv"
LEGACY_VERIFY_PATH = "verify_data.csv"
OUT_PATH = "calibrated_geometry_candidate.json"
ACTIVE_PATH = "calibrated_geometry.json"

NOMINAL_POLARIZATION = 1.2
BASELINE_SIZE = 14
EXTENDED_SIZE = 36
FIT_EULER_SEQUENCE = "ZYX"


def load_data(data_path=DATA_PATH, offsets_path=OFFSETS_PATH):
    rows = np.genfromtxt(data_path, delimiter=",", skip_header=1)
    rows = np.atleast_2d(rows)
    if rows.shape[1] < 10 or not np.isfinite(rows[:, :10]).all():
        raise ValueError(f"{data_path} must contain finite pose and field values")
    with open(offsets_path) as f:
        offsets = json.load(f)
    b0 = np.asarray(offsets["S1"] + offsets["S2"], dtype=float)
    poses = rows[:, 1:4]
    measurements = rows[:, 4:10] - b0
    return poses, measurements


def build_magnet(scale, position, tilt_y, tilt_z):
    magnet = magpy.magnet.Cylinder(
        polarization=(0, 0, NOMINAL_POLARIZATION * scale), dimension=(10, 5))
    magnet.rotate_from_angax(90, "y", anchor=(0, 0, 0))
    magnet.rotate_from_angax(tilt_y, "y", anchor=(0, 0, 0))
    magnet.rotate_from_angax(tilt_z, "z", anchor=(0, 0, 0))
    magnet.position = position
    return magnet


def unpack_extended(x, mode):
    if mode == "baseline":
        return (0.0, np.zeros(6), np.ones(6), np.zeros(3),
                np.asarray(sim.SENSOR_HOMES))
    homes = np.asarray(sim.SENSOR_HOMES) + x[30:36].reshape(2, 3)
    return x[14], x[15:21], x[21:27], x[27:30], homes


def predict(x, pose_list, mode="baseline", euler_sequence=FIT_EULER_SEQUENCE):
    """Predict offset-corrected fields in mT for an array of IMU poses."""
    magnet = build_magnet(x[0], x[1:4], x[4], x[5])
    chip_rots = (Rotation.from_rotvec(x[6:9]), Rotation.from_rotvec(x[9:12]))
    imu_err = Rotation.from_rotvec([x[12], x[13], 0.0])
    yaw_alignment, biases, gains, ambient, sensor_homes = unpack_extended(x, mode)
    poses = np.atleast_2d(np.asarray(pose_list, dtype=float)).copy()
    poses[:, 0] += yaw_alignment
    rotations = Rotation.from_euler(euler_sequence, poses, degrees=True) * imu_err
    output = np.empty((len(poses), 6), dtype=float)
    for sensor_index, (home, chip_rot) in enumerate(zip(sensor_homes, chip_rots)):
        positions = rotations.apply(home)
        field_world = magnet.getB(positions) * 1e3 + ambient
        field_chip = chip_rot.apply(rotations.inv().apply(field_world))
        channel = slice(3 * sensor_index, 3 * sensor_index + 3)
        output[:, channel] = gains[channel] * field_chip + biases[channel]
    return output


def seed_from_geometry(path=ACTIVE_PATH):
    if not os.path.exists(path):
        return np.r_[
            -1.0, 0.0, 0.0, sim.MAGNET_Z, 0.0, 0.0,
            Rotation.from_euler("zyx", (0, -70, 0), degrees=True).as_rotvec(),
            Rotation.from_euler("zyx", (180, 30, -66), degrees=True).as_rotvec(),
            0.0, 0.0,
        ]

    with open(path) as f:
        geometry = json.load(f)
    chip1 = Rotation.from_rotvec(geometry["chip_rotvec_S1"]).as_rotvec()
    chip2 = Rotation.from_rotvec(geometry["chip_rotvec_S2"]).as_rotvec()
    imu = geometry.get("imu_mount_rotvec", [0.0, 0.0, 0.0])
    return np.r_[
        geometry["polarization"] / NOMINAL_POLARIZATION,
        geometry["magnet_position"], geometry["tilt_y"], geometry["tilt_z"],
        chip1, chip2, imu[0], imu[1],
    ]


def extend_seed(baseline, geometry_path=ACTIVE_PATH):
    yaw = 0.0
    biases = np.zeros(6)
    gains = np.ones(6)
    ambient = np.zeros(3)
    sensor_deltas = np.zeros(6)
    if os.path.exists(geometry_path):
        with open(geometry_path) as f:
            geometry = json.load(f)
        yaw = geometry.get("yaw_alignment_deg", yaw)
        biases = np.asarray(
            geometry.get("sensor_bias_mT_S1", [0, 0, 0])
            + geometry.get("sensor_bias_mT_S2", [0, 0, 0]), dtype=float)
        gains = np.asarray(
            geometry.get("sensor_gain_S1", [1, 1, 1])
            + geometry.get("sensor_gain_S2", [1, 1, 1]), dtype=float)
        ambient = np.asarray(geometry.get("ambient_field_mT", ambient), dtype=float)
        fitted_homes = np.r_[
            geometry.get("sensor_home_S1", sim.SENSOR_1_HOME),
            geometry.get("sensor_home_S2", sim.SENSOR_2_HOME),
        ]
        sensor_deltas = fitted_homes - np.r_[sim.SENSOR_1_HOME, sim.SENSOR_2_HOME]
    return np.r_[baseline, yaw, biases, gains, ambient, sensor_deltas]


def parameter_bounds(mode, polarity):
    if polarity < 0:
        scale_low, scale_high = -2.0, -0.2
    else:
        scale_low, scale_high = 0.2, 2.0
    lower = np.r_[
        scale_low, -8.0, -8.0, 25.0, -25.0, -25.0,
        np.full(6, -np.pi), np.full(2, -np.radians(15.0)),
    ]
    upper = np.r_[
        scale_high, 8.0, 8.0, 45.0, 25.0, 25.0,
        np.full(6, np.pi), np.full(2, np.radians(15.0)),
    ]
    if mode == "extended":
        lower = np.r_[lower, -20.0, np.full(6, -2.0), np.full(6, 0.6),
                      np.full(3, -0.5), np.full(6, -5.0)]
        upper = np.r_[upper, 20.0, np.full(6, 2.0), np.full(6, 1.4),
                      np.full(3, 0.5), np.full(6, 5.0)]
    return lower, upper


def fit_model(poses, measurements, x0, mode="baseline", loss="soft_l1",
              f_scale=0.15, max_nfev=1000):
    lower, upper = parameter_bounds(mode, x0[0])
    x0 = np.clip(np.asarray(x0, dtype=float), lower + 1e-8, upper - 1e-8)

    def data_residual(x):
        return (predict(x, poses, mode=mode) - measurements).ravel()

    def objective(x):
        residual = data_residual(x)
        if mode == "extended":
            # Fix the field-strength/sensor-gain scale gauge while allowing
            # relative per-channel gain differences.
            residual = np.r_[
                residual,
                (np.mean(x[21:27]) - 1.0) * 10.0,
                x[30:36] * 0.05,
            ]
        return residual

    result = least_squares(
        objective, x0, bounds=(lower, upper), loss=loss, f_scale=f_scale,
        max_nfev=max_nfev,
    )
    residual = predict(result.x, poses, mode=mode) - measurements
    return result, residual


def residual_summary(residual):
    pose_rms = np.sqrt(np.mean(residual ** 2, axis=1))
    return {
        "rms_mT": float(np.sqrt(np.mean(residual ** 2))),
        "worst_channel_mT": float(np.abs(residual).max()),
        "channel_rms_mT": np.sqrt(np.mean(residual ** 2, axis=0)),
        "pose_median_mT": float(np.median(pose_rms)),
        "pose_p95_mT": float(np.percentile(pose_rms, 95)),
        "pose_max_mT": float(pose_rms.max()),
    }


def print_fit(name, result, residual):
    summary = residual_summary(residual)
    print(f"\n{name}: success={result.success}, evaluations={result.nfev}")
    print(f"  field RMS {summary['rms_mT']:.3f} mT; "
          f"worst channel {summary['worst_channel_mT']:.3f} mT")
    print("  channel RMS: " + " ".join(
        f"{value:.3f}" for value in summary["channel_rms_mT"]))
    print(f"  pose RMS median/p95/max: {summary['pose_median_mT']:.3f} / "
          f"{summary['pose_p95_mT']:.3f} / {summary['pose_max_mT']:.3f} mT")


def yaw_plane_folds(poses, n_folds=3):
    yaw_groups = np.round(poses[:, 0] / 20.0) * 20.0
    levels = np.unique(yaw_groups)
    for fold_index in range(n_folds):
        test_levels = levels[fold_index::n_folds]
        test = np.isin(yaw_groups, test_levels)
        yield test_levels, ~test, test


def cross_validate(poses, measurements, x0, mode, loss, f_scale):
    print(f"\n{mode} grouped yaw-plane cross-validation")
    test_rms_values = []
    for fold_index, (levels, train, test) in enumerate(yaw_plane_folds(poses), 1):
        result, train_residual = fit_model(
            poses[train], measurements[train], x0, mode=mode, loss=loss,
            f_scale=f_scale, max_nfev=600)
        test_residual = predict(result.x, poses[test], mode=mode) - measurements[test]
        train_rms = residual_summary(train_residual)["rms_mT"]
        test_rms = residual_summary(test_residual)["rms_mT"]
        test_rms_values.append(test_rms)
        level_text = ",".join(f"{level:g}" for level in levels)
        print(f"  fold {fold_index}: held yaw [{level_text}]  "
              f"train {train_rms:.3f} mT  test {test_rms:.3f} mT")
    print(f"  mean held-plane RMS: {np.mean(test_rms_values):.3f} mT")


def cross_validate_correction(poses, measurements, x0, mode, loss, f_scale,
                              alphas=DEFAULT_ALPHAS):
    """Choose correction regularization using strictly held-out yaw planes."""
    fold_models = []
    for levels, train, test in yaw_plane_folds(poses):
        result, train_error = fit_model(
            poses[train], measurements[train], x0, mode=mode, loss=loss,
            f_scale=f_scale, max_nfev=600)
        fold_models.append((levels, train, test, result.x, -train_error))

    base_test_rms = []
    for _, _, test, fold_x, _ in fold_models:
        error = predict(fold_x, poses[test], mode=mode) - measurements[test]
        base_test_rms.append(np.sqrt(np.mean(error ** 2)))

    print(f"\n{mode} residual-correction yaw-plane cross-validation")
    print(f"  physical-model mean held-plane RMS: {np.mean(base_test_rms):.3f} mT")
    best = None
    for alpha in alphas:
        fold_rms = []
        for _, train, test, fold_x, train_residual in fold_models:
            correction = fit_correction(poses[train], train_residual, alpha)
            corrected = (
                predict(fold_x, poses[test], mode=mode)
                + predict_correction(correction, poses[test]))
            fold_rms.append(np.sqrt(np.mean(
                (corrected - measurements[test]) ** 2)))
        mean_rms = float(np.mean(fold_rms))
        print(f"  alpha {alpha:7g}: held RMS {mean_rms:.3f} mT  folds "
              + " ".join(f"{value:.3f}" for value in fold_rms))
        if best is None or mean_rms < best[0]:
            best = (mean_rms, alpha)
    print(f"  selected alpha {best[1]:g}: mean held-plane RMS {best[0]:.3f} mT")
    return best[1]


def estimate_dataset(x, mode, measurements, correction=None, n_starts=5):
    yaw_values = np.arange(sim.YAW_RANGE[0], sim.YAW_RANGE[1] + 1, 10)
    pitch_values = np.arange(sim.PITCH_RANGE[0], sim.PITCH_RANGE[1] + 1, 2)
    roll_values = np.arange(sim.ROLL_RANGE[0], sim.ROLL_RANGE[1] + 1, 2)
    grid_poses = np.array([
        (yaw, pitch, roll)
        for yaw in yaw_values
        for pitch in pitch_values
        for roll in roll_values
    ], dtype=float)
    def corrected_predict(poses):
        return (predict(x, poses, mode=mode)
                + predict_correction(correction, poses))

    grid_fields = corrected_predict(grid_poses)
    lower = np.array([sim.YAW_RANGE[0] - 5, sim.PITCH_RANGE[0] - 5,
                      sim.ROLL_RANGE[0] - 5], dtype=float)
    upper = np.array([sim.YAW_RANGE[1] + 5, sim.PITCH_RANGE[1] + 5,
                      sim.ROLL_RANGE[1] + 5], dtype=float)

    estimates = []
    for measured in measurements:
        distances = np.linalg.norm(grid_fields - measured, axis=1)
        starts = grid_poses[np.argsort(distances)[:n_starts]]
        best = None
        for start in starts:
            result = least_squares(
                lambda angles: corrected_predict([angles])[0] - measured,
                start, bounds=(lower, upper), max_nfev=200)
            if best is None or result.cost < best.cost:
                best = result
        estimates.append(best.x)
    return np.asarray(estimates)


def angle_summary(estimates, truth):
    error = np.abs((estimates - truth + 180.0) % 360.0 - 180.0)
    worst = error.max(axis=1)
    return error, {
        "axis_median": np.median(error, axis=0),
        "axis_p95": np.percentile(error, 95, axis=0),
        "worst_median": float(np.median(worst)),
        "worst_p95": float(np.percentile(worst, 95)),
        "worst_max": float(worst.max()),
    }


def print_angle_report(name, estimates, truth):
    _, summary = angle_summary(estimates, truth)
    print(f"\n{name} angle errors (degrees)")
    print("  y/p/r median: " + " ".join(
        f"{value:.2f}" for value in summary["axis_median"]))
    print("  y/p/r p95:    " + " ".join(
        f"{value:.2f}" for value in summary["axis_p95"]))
    print(f"  worst-axis median/p95/max: {summary['worst_median']:.2f} / "
          f"{summary['worst_p95']:.2f} / {summary['worst_max']:.2f}")
    return summary


def geometry_dict(x, mode, n_poses, residual, correction=None):
    yaw, biases, gains, ambient, sensor_homes = unpack_extended(x, mode)
    summary = residual_summary(residual)
    return {
        "comment": f"{mode} model fitted by calibrate.py",
        "model": mode,
        "pose_euler_sequence": FIT_EULER_SEQUENCE,
        "polarization": float(NOMINAL_POLARIZATION * x[0]),
        "magnet_position": x[1:4].tolist(),
        "tilt_y": float(x[4]),
        "tilt_z": float(x[5]),
        "chip_rotvec_S1": x[6:9].tolist(),
        "chip_rotvec_S2": x[9:12].tolist(),
        "imu_mount_rotvec": [float(x[12]), float(x[13]), 0.0],
        "yaw_alignment_deg": float(yaw),
        "sensor_bias_mT_S1": biases[:3].tolist(),
        "sensor_bias_mT_S2": biases[3:].tolist(),
        "sensor_gain_S1": gains[:3].tolist(),
        "sensor_gain_S2": gains[3:].tolist(),
        "ambient_field_mT": ambient.tolist(),
        "sensor_home_S1": sensor_homes[0].tolist(),
        "sensor_home_S2": sensor_homes[1].tolist(),
        "rms_residual_mT": summary["rms_mT"],
        "n_poses": int(n_poses),
        "field_correction": correction,
    }


def write_geometry(path, geometry):
    with open(path, "w") as f:
        json.dump(geometry, f, indent=2)
    print(f"wrote {path}")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", default=DATA_PATH)
    parser.add_argument("--offsets", default=OFFSETS_PATH)
    parser.add_argument("--verify", default=VERIFY_PATH)
    parser.add_argument("--output", default=OUT_PATH)
    parser.add_argument("--mode", choices=("baseline", "extended", "both"),
                        default="both")
    parser.add_argument("--loss", choices=("linear", "soft_l1", "huber"),
                        default="soft_l1")
    parser.add_argument("--f-scale", type=float, default=0.15)
    parser.add_argument("--skip-cv", action="store_true")
    parser.add_argument("--no-correction", action="store_true",
                        help="fit only the physical model")
    parser.add_argument("--correction-alpha", type=float, default=None,
                        help="override correction regularization selection")
    parser.add_argument("--activate", action="store_true",
                        help="also replace calibrated_geometry.json")
    return parser.parse_args()


def resolve_verify_path(path):
    if os.path.exists(path):
        return path
    if path == VERIFY_PATH and os.path.exists(LEGACY_VERIFY_PATH):
        print(f"{VERIFY_PATH} not found; using legacy {LEGACY_VERIFY_PATH}")
        return LEGACY_VERIFY_PATH
    return path


def main():
    args = parse_args()
    poses, measurements = load_data(args.data, args.offsets)
    print(f"loaded {len(poses)} calibration poses ({measurements.size} field values)")

    baseline_seed = seed_from_geometry()
    fitted = {}
    if args.mode in ("baseline", "both"):
        baseline_result, baseline_residual = fit_model(
            poses, measurements, baseline_seed, mode="baseline",
            loss=args.loss, f_scale=args.f_scale)
        print_fit("baseline", baseline_result, baseline_residual)
        fitted["baseline"] = (baseline_result.x, baseline_residual)
    else:
        baseline_result, _ = fit_model(
            poses, measurements, baseline_seed, mode="baseline",
            loss=args.loss, f_scale=args.f_scale)

    if args.mode in ("extended", "both"):
        extended_seed = extend_seed(baseline_result.x)
        extended_result, extended_residual = fit_model(
            poses, measurements, extended_seed, mode="extended",
            loss=args.loss, f_scale=args.f_scale)
        print_fit("extended", extended_result, extended_residual)
        fitted["extended"] = (extended_result.x, extended_residual)

    selected_mode = "extended" if "extended" in fitted else "baseline"
    selected_x, selected_residual = fitted[selected_mode]

    correction = None
    if not args.no_correction:
        alpha = args.correction_alpha
        if alpha is None and not args.skip_cv:
            alpha = cross_validate_correction(
                poses, measurements, selected_x, selected_mode,
                args.loss, args.f_scale)
        if alpha is None:
            alpha = 10.0
        correction = fit_correction(poses, -selected_residual, alpha)
        selected_residual = (
            predict(selected_x, poses, mode=selected_mode)
            + predict_correction(correction, poses) - measurements)
        summary = residual_summary(selected_residual)
        print(f"\ncorrected training field RMS: {summary['rms_mT']:.3f} mT "
              f"(alpha {alpha:g})")
    elif not args.skip_cv:
        cross_validate(
            poses, measurements, selected_x, selected_mode, args.loss, args.f_scale)

    verify_path = resolve_verify_path(args.verify)
    if verify_path and os.path.exists(verify_path):
        verify_poses, verify_measurements = load_data(verify_path, args.offsets)
        estimates = estimate_dataset(
            selected_x, selected_mode, verify_measurements, correction=correction)
        label = f"{selected_mode}{'+correction' if correction else ''}"
        print_angle_report(f"{label} verification ({verify_path})",
                           estimates, verify_poses)
    else:
        print(f"verification skipped: {verify_path} not found")

    geometry = geometry_dict(
        selected_x, selected_mode, len(poses), selected_residual,
        correction=correction)
    write_geometry(args.output, geometry)
    if args.activate:
        write_geometry(ACTIVE_PATH, geometry)
        print("active geometry changed; rebuild lookup_table.npz")


if __name__ == "__main__":
    main()
