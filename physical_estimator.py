#!/usr/bin/env python3
"""Invert the accepted physical model from six magnetic channels to a pose."""

import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

from physical_model import load_model, predict_mT


def load_yaw_zero_offset(correction_path, model_path):
    """Load an optional legacy model-frame-to-dial-frame yaw offset."""
    if correction_path is None:
        return 0.0
    correction_path = Path(correction_path)
    if not correction_path.exists():
        return 0.0
    with correction_path.open() as source:
        correction = json.load(source)
    if correction.get("schema_version") != 1:
        raise ValueError(f"unsupported yaw correction schema in {correction_path}")
    digest = hashlib.sha256()
    with Path(model_path).open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    if correction["inputs"]["model_sha256"] != digest.hexdigest():
        raise ValueError(
            f"{correction_path} was fitted for a different physical model"
        )
    return float(correction["yaw_zero_offset_deg"])


class PhysicalModelEstimator:
    """Coarse global field search followed by bounded nonlinear refinement.

    Poses are searched and reported in the mechanical dial frame. Current
    models are fitted in that frame directly; ``correction_path`` remains
    available only for legacy models fitted in a rotated yaw frame.
    """

    def __init__(self, model_path=Path("physical_model.json"),
                 geometry_path=Path("geometry_priors.json"),
                 correction_path=None,
                 yaw_step_deg=10.0, tilt_step_deg=5.0):
        self.model = load_model(model_path)
        if not self.model.get("calibration_gate_passed", False):
            raise ValueError(f"{model_path} has not passed its calibration gate")
        self.yaw_zero_offset_deg = load_yaw_zero_offset(correction_path, model_path)
        with Path(geometry_path).open() as source:
            geometry = json.load(source)
        workspace = geometry["workspace_deg"]
        self.lower = np.asarray((workspace["yaw"][0], workspace["pitch"][0],
                                 workspace["roll"][0]), dtype=float)
        self.upper = np.asarray((workspace["yaw"][1], workspace["pitch"][1],
                                 workspace["roll"][1]), dtype=float)
        self.x_scale = np.maximum((self.upper - self.lower) / 4.0, 1.0)
        self.yaw_step_deg = yaw_step_deg
        self.tilt_step_deg = tilt_step_deg
        self._build_grid()

    def _build_grid(self):
        yaw_values = inclusive_range(self.lower[0], self.upper[0], self.yaw_step_deg)
        pitch_values = inclusive_range(
            self.lower[1], self.upper[1], self.tilt_step_deg
        )
        roll_values = inclusive_range(
            self.lower[2], self.upper[2], self.tilt_step_deg
        )
        self.grid_poses = np.asarray([
            (yaw, pitch, roll)
            for yaw in yaw_values
            for pitch in pitch_values
            for roll in roll_values
        ])
        self.grid_fields_mT = self._predict(self.grid_poses)

    def widen_yaw_bounds(self, margin_deg):
        """Extend the yaw search range, e.g. when truth labels carry a frame
        offset that pushes poses past the mechanical workspace edge."""
        if margin_deg:
            self.lower[0] -= float(margin_deg)
            self.upper[0] += float(margin_deg)
            self._build_grid()

    def _predict(self, angles):
        shifted = np.atleast_2d(np.asarray(angles, dtype=float)).copy()
        shifted[:, 0] += self.yaw_zero_offset_deg
        return predict_mT(shifted, self.model)

    def _residual(self, angles, measured_mT):
        return self._predict(angles)[0] - measured_mT

    def _refine(self, start, measured_mT):
        return least_squares(
            self._residual,
            np.clip(np.asarray(start, dtype=float), self.lower, self.upper),
            args=(measured_mT,),
            bounds=(self.lower, self.upper),
            x_scale=self.x_scale,
            max_nfev=150,
            xtol=1e-9,
            ftol=1e-9,
            gtol=1e-9,
        )

    def global_starts(self, measured_mT, count):
        distances = np.sum((self.grid_fields_mT - measured_mT) ** 2, axis=1)
        indexes = np.argsort(distances)[:max(1, int(count))]
        return [self.grid_poses[index] for index in indexes]

    def estimate(self, measured_mT, seed=None, global_starts=3,
                 reacquire_threshold_mT=0.25):
        measured = np.asarray(measured_mT, dtype=float)
        if measured.shape != (6,) or not np.isfinite(measured).all():
            raise ValueError("measurement must contain six finite channels in mT")

        tracking_result = None
        if seed is not None:
            tracking_result = self._refine(seed, measured)
            tracking_rms = float(np.sqrt(np.mean(tracking_result.fun ** 2)))
            if tracking_rms <= reacquire_threshold_mT:
                return estimation_result(tracking_result, reacquired=False)

        best = tracking_result
        for start in self.global_starts(measured, global_starts):
            candidate = self._refine(start, measured)
            if best is None or np.sum(candidate.fun ** 2) < np.sum(best.fun ** 2):
                best = candidate
        return estimation_result(best, reacquired=seed is not None)


def inclusive_range(lower, upper, step):
    if step <= 0:
        raise ValueError("grid step must be positive")
    count = int(np.floor((upper - lower) / step + 0.5))
    values = lower + np.arange(count + 1) * step
    if values[-1] < upper - 1e-9:
        values = np.r_[values, upper]
    else:
        values[-1] = upper
    return values


def estimation_result(result, reacquired):
    return {
        "angles_deg": result.x.copy(),
        "model_rms_mT": float(np.sqrt(np.mean(result.fun ** 2))),
        "success": bool(result.success),
        "function_evaluations": int(result.nfev),
        "reacquired": bool(reacquired),
    }
