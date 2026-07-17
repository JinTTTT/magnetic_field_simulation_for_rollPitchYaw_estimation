"""Estimate dial-frame yaw, pitch, and roll from six magnetic channels."""

import numpy as np
from scipy.optimize import least_squares

from .model import DEFAULT_MODEL_PATH, load_model, predict_mT


class PoseEstimator:
    """Coarse global search followed by bounded nonlinear refinement."""

    def __init__(self, model_path=DEFAULT_MODEL_PATH, yaw_step=10.0, tilt_step=5.0):
        self.model = load_model(model_path)
        if not self.model.get("calibration_gate_passed", False):
            raise ValueError(f"{model_path} has not passed its calibration gate")

        workspace = self.model["pose_convention"]["workspace_deg"]
        self.lower = np.array([workspace[name][0] for name in ("yaw", "pitch", "roll")])
        self.upper = np.array([workspace[name][1] for name in ("yaw", "pitch", "roll")])
        self.scale = np.maximum((self.upper - self.lower) / 4.0, 1.0)
        self.yaw_step = yaw_step
        self.tilt_step = tilt_step
        self._build_grid()

    def _build_grid(self):
        axes = (
            inclusive_range(self.lower[0], self.upper[0], self.yaw_step),
            inclusive_range(self.lower[1], self.upper[1], self.tilt_step),
            inclusive_range(self.lower[2], self.upper[2], self.tilt_step),
        )
        self.grid_poses = np.array([
            (yaw, pitch, roll)
            for yaw in axes[0]
            for pitch in axes[1]
            for roll in axes[2]
        ])
        self.grid_fields_mT = predict_mT(self.grid_poses, self.model)

    def widen_yaw_bounds(self, margin_deg):
        if margin_deg:
            self.lower[0] -= float(margin_deg)
            self.upper[0] += float(margin_deg)
            self._build_grid()

    def _residual(self, angles, measured_mT):
        return predict_mT([angles], self.model)[0] - measured_mT

    def _refine(self, start, measured_mT):
        return least_squares(
            self._residual,
            np.clip(np.asarray(start, dtype=float), self.lower, self.upper),
            args=(measured_mT,),
            bounds=(self.lower, self.upper),
            x_scale=self.scale,
            max_nfev=150,
            xtol=1e-9,
            ftol=1e-9,
            gtol=1e-9,
        )

    def _global_starts(self, measured_mT, count):
        distances = np.sum((self.grid_fields_mT - measured_mT) ** 2, axis=1)
        return self.grid_poses[np.argsort(distances)[:max(1, int(count))]]

    def estimate(self, measured_mT, seed=None, global_starts=3,
                 reacquire_threshold_mT=0.25):
        measured = np.asarray(measured_mT, dtype=float)
        if measured.shape != (6,) or not np.isfinite(measured).all():
            raise ValueError("measurement must contain six finite channels")

        best = None
        if seed is not None:
            best = self._refine(seed, measured)
            rms = float(np.sqrt(np.mean(best.fun ** 2)))
            if rms <= reacquire_threshold_mT:
                return result_dict(best, reacquired=False)

        for start in self._global_starts(measured, global_starts):
            candidate = self._refine(start, measured)
            if best is None or np.sum(candidate.fun ** 2) < np.sum(best.fun ** 2):
                best = candidate
        return result_dict(best, reacquired=seed is not None)


def inclusive_range(lower, upper, step):
    if step <= 0:
        raise ValueError("grid step must be positive")
    values = np.arange(lower, upper + step / 2.0, step)
    values[-1] = upper
    return values


def result_dict(result, reacquired):
    return {
        "angles_deg": result.x.copy(),
        "model_rms_mT": float(np.sqrt(np.mean(result.fun ** 2))),
        "reacquired": bool(reacquired),
    }


# Compatibility name for archived notebooks or external imports.
PhysicalModelEstimator = PoseEstimator
