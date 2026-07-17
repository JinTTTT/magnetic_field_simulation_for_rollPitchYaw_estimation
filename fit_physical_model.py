#!/usr/bin/env python3
"""Fit the finite-cylinder physical model using calibration data only."""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/magnetic_orientation_matplotlib")

import magpylib as magpy
import numpy as np
import scipy
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from physical_model import CHANNELS, predict_mT, sensor_home_positions_mm


BASE_MAGNET_ROTATION = Rotation.from_euler("y", 90.0, degrees=True)


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_json(path):
    with path.open() as source:
        return json.load(source)


def manifest_entry(manifest, role):
    try:
        return manifest["files"][role]
    except KeyError as error:
        raise ValueError(f"manifest has no {role!r} entry") from error


def verify_locked_input(manifest, role):
    entry = manifest_entry(manifest, role)
    path = Path(entry["path"])
    actual = sha256_file(path)
    if actual != entry["sha256"]:
        raise ValueError(
            f"locked {role} input changed: {path}; rerun freeze_datasets.py "
            "only if the change was intentional"
        )
    return path, actual


def load_calibration(path):
    with path.open(newline="") as source:
        rows = list(csv.DictReader(source))
    if not rows:
        raise ValueError(f"{path} contains no calibration rows")

    angles = np.asarray([
        [float(row[name]) for name in ("yaw_deg", "pitch_deg", "roll_deg")]
        for row in rows
    ])
    measured = np.asarray([
        [float(row[f"{name}_corrected_mT"]) for name in CHANNELS]
        for row in rows
    ])
    sample_stddev = np.asarray([
        [float(row[f"{name}_stddev_mT"]) for name in CHANNELS]
        for row in rows
    ])
    arrays = (angles, measured, sample_stddev)
    if not all(np.isfinite(array).all() for array in arrays):
        raise ValueError(f"{path} contains non-finite calibration values")
    return rows, angles, measured, sample_stddev


def parameter_vectors(config, polarization_prior):
    params = config["parameters"]
    x0 = np.zeros(16, dtype=float)
    x0[5] = polarization_prior
    lower = np.r_[
        params["magnet_center_delta_mm"]["lower"],
        params["magnet_axis_tilt_deg"]["lower"],
        params["polarization_magnitude_T"]["lower"],
        params["sensor_plane_delta_mm"]["lower"],
        params["ring_radius_delta_mm"]["lower"],
        params["sensor_separation_delta_deg"]["lower"],
        params["imu_axis_alignment_deg"]["lower"],
        params["chip_rotation_refinement_deg"]["lower"],
    ].astype(float)
    upper = np.r_[
        params["magnet_center_delta_mm"]["upper"],
        params["magnet_axis_tilt_deg"]["upper"],
        params["polarization_magnitude_T"]["upper"],
        params["sensor_plane_delta_mm"]["upper"],
        params["ring_radius_delta_mm"]["upper"],
        params["sensor_separation_delta_deg"]["upper"],
        params["imu_axis_alignment_deg"]["upper"],
        params["chip_rotation_refinement_deg"]["upper"],
    ].astype(float)
    return x0, lower, upper


class FitProblem:
    def __init__(self, angles, measured, sample_stddev, geometry, config):
        self.angles = angles
        self.measured = measured
        self.sample_stddev = sample_stddev
        self.geometry = geometry
        self.config = config
        self.raw_pose = Rotation.from_euler("ZYX", angles, degrees=True)

        sensors = geometry["sensors"]
        magnet = geometry["magnet"]
        self.prior_center = np.asarray(
            magnet["home_center_position_mm"]["value"], dtype=float
        )
        self.prior_plane = float(sensors["sensor_plane_z_mm"]["value"])
        self.prior_radius = float(sensors["ring_radius_mm"]["value"])
        self.prior_separation = float(sensors["angular_separation_deg"]["value"])
        self.magnet_dimension_mm = np.asarray((
            magnet["diameter_mm"]["value"], magnet["height_mm"]["value"]
        ), dtype=float)
        self.polarization_prior = abs(float(magnet["nominal_polarization_T"]["value"]))

        floor = float(config["weighting"]["minimum_channel_scale_mT"])
        self.field_scale = np.maximum(sample_stddev, floor)
        self.polarity_sign, self.chip_initial, self.sign_trials = (
            self._choose_polarity_and_chip_initialization()
        )

    def _source(self, center_mm, tilt_y_deg, tilt_z_deg, magnitude_T):
        tilt = Rotation.from_rotvec(np.radians((0.0, tilt_y_deg, tilt_z_deg)))
        return magpy.magnet.Cylinder(
            position=np.asarray(center_mm) / 1000.0,
            orientation=tilt * BASE_MAGNET_ROTATION,
            dimension=self.magnet_dimension_mm / 1000.0,
            polarization=(0.0, 0.0, self.polarity_sign * magnitude_T),
        )

    def _prior_rig_fields(self, sign):
        source = magpy.magnet.Cylinder(
            position=self.prior_center / 1000.0,
            orientation=BASE_MAGNET_ROTATION,
            dimension=self.magnet_dimension_mm / 1000.0,
            polarization=(0.0, 0.0, sign * self.polarization_prior),
        )
        homes = sensor_home_positions_mm(
            self.prior_plane, self.prior_radius, self.prior_separation
        )
        fields = []
        for home in homes:
            observers = self.raw_pose.apply(
                np.tile(home / 1000.0, (len(self.angles), 1))
            )
            world = source.getB(observers) * 1000.0
            fields.append(self.raw_pose.inv().apply(world))
        return fields

    def _choose_polarity_and_chip_initialization(self):
        trials = []
        for sign in (-1.0, 1.0):
            rig_fields = self._prior_rig_fields(sign)
            rotations = []
            predictions = []
            for sensor_index, field in enumerate(rig_fields):
                channel = slice(3 * sensor_index, 3 * sensor_index + 3)
                rotation, rssd = Rotation.align_vectors(
                    self.measured[:, channel], field
                )
                rotations.append(rotation)
                predictions.append(rotation.apply(field))
            residual = np.hstack(predictions) - self.measured
            trials.append({
                "sign": sign,
                "rmse_mT": float(np.sqrt(np.mean(residual ** 2))),
                "alignment_rssd": [
                    float(Rotation.align_vectors(
                        self.measured[:, slice(3 * i, 3 * i + 3)],
                        rig_fields[i]
                    )[1])
                    for i in range(2)
                ],
                "rotations": rotations,
            })
        selected = min(trials, key=lambda item: item["rmse_mT"])
        public_trials = [
            {key: value for key, value in trial.items() if key != "rotations"}
            for trial in trials
        ]
        return selected["sign"], selected["rotations"], public_trials

    def unpack(self, x):
        return {
            "center_mm": self.prior_center + x[0:3],
            "tilt_y_deg": x[3],
            "tilt_z_deg": x[4],
            "polarization_magnitude_T": x[5],
            "sensor_plane_z_mm": self.prior_plane + x[6],
            "ring_radius_mm": self.prior_radius + x[7],
            "angular_separation_deg": self.prior_separation + x[8],
            "imu_axis_alignment_deg": x[9],
        }

    def pose(self, alignment_deg):
        alignment = Rotation.from_euler("z", alignment_deg, degrees=True)
        return alignment * self.raw_pose * alignment.inv()

    def chip_rotations(self, x):
        result = []
        for sensor_index in range(2):
            start = 10 + 3 * sensor_index
            refinement = Rotation.from_rotvec(np.radians(x[start:start + 3]))
            result.append(refinement * self.chip_initial[sensor_index])
        return result

    def predict(self, x):
        values = self.unpack(x)
        pose = self.pose(values["imu_axis_alignment_deg"])
        source = self._source(
            values["center_mm"], values["tilt_y_deg"], values["tilt_z_deg"],
            values["polarization_magnitude_T"]
        )
        homes = sensor_home_positions_mm(
            values["sensor_plane_z_mm"], values["ring_radius_mm"],
            values["angular_separation_deg"]
        )
        output = []
        for home, chip in zip(homes, self.chip_rotations(x)):
            observers = pose.apply(np.tile(home / 1000.0, (len(self.angles), 1)))
            world_mT = source.getB(observers) * 1000.0
            rig_mT = pose.inv().apply(world_mT)
            output.append(chip.apply(rig_mT))
        return np.hstack(output)

    def prior_residual(self, x):
        params = self.config["parameters"]
        return np.r_[
            x[0:3] / np.asarray(params["magnet_center_delta_mm"]["prior_sigma"]),
            x[3:5] / np.asarray(params["magnet_axis_tilt_deg"]["prior_sigma"]),
            (x[5] - self.polarization_prior)
            / float(params["polarization_magnitude_T"]["prior_sigma"]),
            x[6] / float(params["sensor_plane_delta_mm"]["prior_sigma"]),
            x[7] / float(params["ring_radius_delta_mm"]["prior_sigma"]),
            x[8] / float(params["sensor_separation_delta_deg"]["prior_sigma"]),
            x[9] / float(params["imu_axis_alignment_deg"]["prior_sigma"]),
            x[10:16]
            / float(params["chip_rotation_refinement_deg"]["prior_sigma"]),
        ]

    def residual(self, x):
        field = ((self.predict(x) - self.measured) / self.field_scale).ravel()
        return np.r_[field, self.prior_residual(x)]

    def model_payload(self, x, calibration_hash, geometry_hash, config_hash):
        values = self.unpack(x)
        chips = self.chip_rotations(x)
        homes = sensor_home_positions_mm(
            values["sensor_plane_z_mm"], values["ring_radius_mm"],
            values["angular_separation_deg"]
        )
        return {
            "schema_version": 1,
            "created_at_utc": datetime.now(timezone.utc).isoformat(),
            "status": "calibration_only_physical_model_candidate",
            "model_type": self.config["model"]["type"],
            "input_identifiers": {
                "calibration_sha256": calibration_hash,
                "geometry_priors_sha256": geometry_hash,
                "fit_config_sha256": config_hash,
                "verification_data_used": False,
            },
            "units": {"length": "mm", "field": "mT", "angle": "degree"},
            "pose_convention": {
                "order": ["yaw", "pitch", "roll"],
                "euler_sequence": "ZYX",
                "imu_axis_alignment_rule": "R_rig = Rz(a) * R_imu * Rz(a)^-1",
            },
            "imu_axis_alignment_deg": float(values["imu_axis_alignment_deg"]),
            "magnet": {
                "type": "finite_cylinder",
                "center_mm": values["center_mm"].tolist(),
                "dimension_mm": self.magnet_dimension_mm.tolist(),
                "axis_tilt_y_deg": float(values["tilt_y_deg"]),
                "axis_tilt_z_deg": float(values["tilt_z_deg"]),
                "signed_polarization_T": float(
                    self.polarity_sign * values["polarization_magnitude_T"]
                ),
                "polarity_sign_selected_from_calibration": self.polarity_sign,
            },
            "sensors": {
                "sensor_plane_z_mm": float(values["sensor_plane_z_mm"]),
                "ring_radius_mm": float(values["ring_radius_mm"]),
                "sensor_1_azimuth_deg": 0.0,
                "angular_separation_deg": float(values["angular_separation_deg"]),
                "home_positions_mm": homes.tolist(),
                "rig_to_chip_rotation_matrices": [
                    chip.as_matrix().tolist() for chip in chips
                ],
                "rig_to_chip_rotvec_rad": [
                    chip.as_rotvec().tolist() for chip in chips
                ],
            },
            "fitted_deltas_from_priors": {
                "magnet_center_mm": x[0:3].tolist(),
                "magnet_axis_tilt_deg": x[3:5].tolist(),
                "polarization_magnitude_T": float(
                    values["polarization_magnitude_T"] - self.polarization_prior
                ),
                "sensor_plane_z_mm": float(x[6]),
                "ring_radius_mm": float(x[7]),
                "sensor_separation_deg": float(x[8]),
                "imu_axis_alignment_deg": float(x[9]),
                "chip_rotation_refinement_deg": x[10:16].reshape(2, 3).tolist(),
            },
        }


def scalar_metrics(residual):
    values = np.asarray(residual, dtype=float).ravel()
    absolute = np.abs(values)
    return {
        "rmse_mT": float(np.sqrt(np.mean(values ** 2))),
        "mean_mT": float(np.mean(values)),
        "median_absolute_mT": float(np.median(absolute)),
        "p95_absolute_mT": float(np.percentile(absolute, 95)),
        "maximum_absolute_mT": float(np.max(absolute)),
    }


def residual_report(residual):
    return {
        "all_channels": scalar_metrics(residual),
        "per_channel": {
            name: scalar_metrics(residual[:, index])
            for index, name in enumerate(CHANNELS)
        },
    }


def boundary_hits(x, lower, upper, fraction):
    span = upper - lower
    names = (
        "magnet_dx", "magnet_dy", "magnet_dz", "axis_tilt_y", "axis_tilt_z",
        "polarization_magnitude", "sensor_plane_dz", "ring_radius_delta",
        "sensor_separation_delta", "imu_axis_alignment",
        "S1_chip_rx", "S1_chip_ry", "S1_chip_rz",
        "S2_chip_rx", "S2_chip_ry", "S2_chip_rz",
    )
    result = []
    for name, value, lo, hi, width in zip(names, x, lower, upper, span):
        if min(value - lo, hi - value) <= fraction * width:
            result.append({"parameter": name, "value": float(value),
                           "lower": float(lo), "upper": float(hi)})
    return result


def jacobian_diagnostics(jacobian):
    singular_values = np.linalg.svd(np.asarray(jacobian), compute_uv=False)
    tolerance = (
        np.finfo(float).eps * max(jacobian.shape) * singular_values[0]
    )
    rank = int(np.count_nonzero(singular_values > tolerance))
    condition = (
        float(singular_values[0] / singular_values[-1])
        if singular_values[-1] > 0 else float("inf")
    )
    return {
        "rows": int(jacobian.shape[0]),
        "columns": int(jacobian.shape[1]),
        "numerical_rank": rank,
        "full_column_rank": rank == jacobian.shape[1],
        "condition_number": condition,
        "singular_values": singular_values.tolist(),
    }


def write_predictions(path, rows, measured, predicted):
    residual = predicted - measured
    with path.open("w", newline="") as output:
        writer = csv.writer(output, lineterminator="\n")
        writer.writerow((
            "pose_id", "session_id", "yaw_deg", "pitch_deg", "roll_deg",
            *(f"{name}_measured_mT" for name in CHANNELS),
            *(f"{name}_predicted_mT" for name in CHANNELS),
            *(f"{name}_residual_mT" for name in CHANNELS),
        ))
        for source, measured_row, predicted_row, residual_row in zip(
                rows, measured, predicted, residual):
            writer.writerow((
                source["pose_id"], source["session_id"],
                source["yaw_deg"], source["pitch_deg"], source["roll_deg"],
                *(f"{value:.9f}" for value in measured_row),
                *(f"{value:.9f}" for value in predicted_row),
                *(f"{value:.9f}" for value in residual_row),
            ))


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=Path("dataset_manifest.json"))
    parser.add_argument("--config", type=Path,
                        default=Path("physical_model_fit_config.json"))
    parser.add_argument("--model-output", type=Path, default=Path("physical_model.json"))
    parser.add_argument("--report-output", type=Path,
                        default=Path("physical_model_calibration_report.json"))
    parser.add_argument("--predictions-output", type=Path,
                        default=Path("physical_model_calibration_predictions.csv"))
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    outputs = (args.model_output, args.report_output, args.predictions_output)
    existing = [path for path in outputs if path.exists()]
    if existing and not args.force:
        raise SystemExit(
            "refusing to overwrite output(s): " + ", ".join(map(str, existing))
            + "; pass --force"
        )

    manifest = load_json(args.manifest)
    calibration_path, calibration_hash = verify_locked_input(
        manifest, "calibration"
    )
    geometry_path, geometry_hash = verify_locked_input(manifest, "geometry_priors")
    # Deliberately do not resolve, hash, or open the manifest's verification path.
    if manifest_entry(manifest, "verification")["policy"] != (
            "untouched_final_evaluation_only"):
        raise ValueError("verification manifest policy is not locked for final evaluation")

    geometry = load_json(geometry_path)
    config = load_json(args.config)
    if geometry["coordinate_system"]["pose_euler_sequence"] != "ZYX":
        raise ValueError("geometry priors must use intrinsic ZYX")
    rows, angles, measured, sample_stddev = load_calibration(calibration_path)
    problem = FitProblem(angles, measured, sample_stddev, geometry, config)
    x0, lower, upper = parameter_vectors(config, problem.polarization_prior)

    baseline_prediction = problem.predict(x0)
    optimizer = config["optimizer"]
    result = least_squares(
        problem.residual,
        x0,
        bounds=(lower, upper),
        loss=optimizer["loss"],
        f_scale=float(optimizer["f_scale"]),
        max_nfev=int(optimizer["max_nfev"]),
        x_scale=optimizer["x_scale"],
    )
    fitted_prediction = problem.predict(result.x)

    config_hash = sha256_file(args.config)
    model = problem.model_payload(
        result.x, calibration_hash, geometry_hash, config_hash
    )
    runtime_prediction = predict_mT(angles, model)
    runtime_difference = float(np.max(np.abs(runtime_prediction - fitted_prediction)))
    if runtime_difference > 1e-9:
        raise RuntimeError(
            f"serialized runtime model differs from fitter by {runtime_difference:g} mT"
        )

    fitted_residual = fitted_prediction - measured
    baseline_residual = baseline_prediction - measured
    fraction = float(config["gate"]["bound_proximity_fraction"])
    hits = boundary_hits(result.x, lower, upper, fraction)
    fitted_rmse = float(np.sqrt(np.mean(fitted_residual ** 2)))
    gate_limit = float(config["gate"]["maximum_calibration_rmse_mT"])
    prior_limit = float(config["gate"]["maximum_absolute_prior_sigma"])
    maximum_prior_sigma = float(np.max(np.abs(problem.prior_residual(result.x))))
    gate_passed = bool(
        result.success
        and not hits
        and fitted_rmse <= gate_limit
        and maximum_prior_sigma <= prior_limit
    )
    model["status"] = (
        "accepted_calibration_only_physical_model"
        if gate_passed else "rejected_calibration_only_physical_model"
    )
    model["calibration_gate_passed"] = gate_passed

    report = {
        "schema_version": 1,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "report_type": "calibration_only_physical_model_fit",
        "verification_data_loaded": False,
        "inputs": {
            "manifest": str(args.manifest),
            "calibration_path": str(calibration_path),
            "calibration_sha256": calibration_hash,
            "calibration_rows": len(rows),
            "geometry_priors_path": str(geometry_path),
            "geometry_priors_sha256": geometry_hash,
            "fit_config_path": str(args.config),
            "fit_config_sha256": config_hash,
        },
        "software": {
            "numpy": np.__version__,
            "scipy": scipy.__version__,
            "magpylib": magpy.__version__,
        },
        "initialization": {
            "polarity_trials": problem.sign_trials,
            "selected_polarity_sign": problem.polarity_sign,
            "chip_initial_rotvec_rad": [
                rotation.as_rotvec().tolist() for rotation in problem.chip_initial
            ],
        },
        "optimizer": {
            "success": bool(result.success),
            "status": int(result.status),
            "message": result.message,
            "function_evaluations": int(result.nfev),
            "cost": float(result.cost),
            "optimality": float(result.optimality),
            "active_mask": result.active_mask.tolist(),
            "jacobian": jacobian_diagnostics(result.jac),
        },
        "baseline_residual": residual_report(baseline_residual),
        "fitted_residual": residual_report(fitted_residual),
        "measured_channel_sample_stddev_mT": {
            "median": float(np.median(sample_stddev)),
            "p95": float(np.percentile(sample_stddev, 95)),
            "maximum": float(np.max(sample_stddev)),
        },
        "prior_residual_at_fit": problem.prior_residual(result.x).tolist(),
        "maximum_absolute_prior_sigma": maximum_prior_sigma,
        "boundary_hits": hits,
        "runtime_serialization_max_difference_mT": runtime_difference,
        "acceptance_gate": {
            "maximum_calibration_rmse_mT": gate_limit,
            "optimizer_converged": bool(result.success),
            "no_parameter_at_bound": not hits,
            "maximum_absolute_prior_sigma": prior_limit,
            "fitted_maximum_absolute_prior_sigma": maximum_prior_sigma,
            "prior_displacements_within_limit": maximum_prior_sigma <= prior_limit,
            "passed": gate_passed,
        },
        "model_output": str(args.model_output),
        "predictions_output": str(args.predictions_output),
    }

    with args.model_output.open("w") as output:
        json.dump(model, output, indent=2)
        output.write("\n")
    with args.report_output.open("w") as output:
        json.dump(report, output, indent=2)
        output.write("\n")
    write_predictions(args.predictions_output, rows, measured, fitted_prediction)

    print("Calibration-only physical model fit")
    print(f"rows:                 {len(rows)}")
    print("verification loaded:  no")
    print(f"selected polarity:    {problem.polarity_sign:+.0f}")
    print(f"baseline RMSE:        {scalar_metrics(baseline_residual)['rmse_mT']:.6f} mT")
    print(f"fitted RMSE:          {fitted_rmse:.6f} mT")
    print(f"median sample stddev: {np.median(sample_stddev):.6f} mT")
    print(f"optimizer:            {'converged' if result.success else 'failed'}")
    print(f"parameters at bounds: {len(hits)}")
    print(f"physical-model gate:  {'PASS' if gate_passed else 'FAIL'}")
    print(f"wrote model:          {args.model_output}")
    print(f"wrote report:         {args.report_output}")
    print(f"wrote predictions:    {args.predictions_output}")


if __name__ == "__main__":
    main()
