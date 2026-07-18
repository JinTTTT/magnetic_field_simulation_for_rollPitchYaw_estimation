"""Precomputed lookup-table pose estimation in six-channel field space."""

import hashlib
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from .config import LOOKUP_PATH
from .model import DEFAULT_MODEL_PATH, load_model, predict_mT


LOOKUP_SCHEMA_VERSION = 1
DEFAULT_YAW_STEP_DEG = 0.5
DEFAULT_TILT_STEP_DEG = 1.0


def inclusive_range(lower, upper, step):
    if step <= 0:
        raise ValueError("grid step must be positive")
    values = np.arange(lower, upper + step / 2.0, step)
    values[-1] = upper
    return values


def model_sha256(model_path):
    return hashlib.sha256(Path(model_path).read_bytes()).hexdigest()


def lookup_grid(model, yaw_step_deg, tilt_step_deg):
    workspace = model["pose_convention"]["workspace_deg"]
    axes = (
        inclusive_range(*workspace["yaw"], yaw_step_deg),
        inclusive_range(*workspace["pitch"], tilt_step_deg),
        inclusive_range(*workspace["roll"], tilt_step_deg),
    )
    return np.array([
        (yaw, pitch, roll)
        for yaw in axes[0]
        for pitch in axes[1]
        for roll in axes[2]
    ])


def build_lookup_table(output_path=LOOKUP_PATH, model_path=DEFAULT_MODEL_PATH,
                       yaw_step_deg=DEFAULT_YAW_STEP_DEG,
                       tilt_step_deg=DEFAULT_TILT_STEP_DEG):
    """Build and save a model-fingerprinted lookup table."""
    model = load_model(model_path)
    if not model.get("calibration_gate_passed", False):
        raise ValueError(f"{model_path} has not passed its calibration gate")
    poses = lookup_grid(model, yaw_step_deg, tilt_step_deg)
    fields = predict_mT(poses, model)

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("wb") as output:
        np.savez_compressed(
            output,
            schema_version=np.array(LOOKUP_SCHEMA_VERSION),
            model_sha256=np.array(model_sha256(model_path)),
            yaw_step_deg=np.array(float(yaw_step_deg)),
            tilt_step_deg=np.array(float(tilt_step_deg)),
            poses_deg=poses,
            fields_mT=fields,
        )
    return len(poses)


def load_lookup_table(path, model_path):
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(
            f"{path} does not exist; build it with tools/build_lookup_table.py"
        )
    with np.load(path, allow_pickle=False) as payload:
        required = {
            "schema_version", "model_sha256", "yaw_step_deg",
            "tilt_step_deg", "poses_deg", "fields_mT",
        }
        missing = required.difference(payload.files)
        if missing:
            raise ValueError(f"{path} is missing: {', '.join(sorted(missing))}")
        if int(payload["schema_version"].item()) != LOOKUP_SCHEMA_VERSION:
            raise ValueError(f"unsupported lookup-table schema in {path}")
        expected_hash = model_sha256(model_path)
        if str(payload["model_sha256"].item()) != expected_hash:
            raise ValueError(
                f"{path} was built for a different model; rebuild the lookup table"
            )
        poses = np.asarray(payload["poses_deg"], dtype=float).copy()
        fields = np.asarray(payload["fields_mT"], dtype=float).copy()
        yaw_step = float(payload["yaw_step_deg"].item())
        tilt_step = float(payload["tilt_step_deg"].item())

    if poses.ndim != 2 or poses.shape[1] != 3:
        raise ValueError(f"{path} must contain an N-by-3 pose array")
    if fields.shape != (len(poses), 6):
        raise ValueError(f"{path} must contain a matching N-by-6 field array")
    if not np.isfinite(poses).all() or not np.isfinite(fields).all():
        raise ValueError(f"{path} contains non-finite values")
    if yaw_step <= 0 or tilt_step <= 0:
        raise ValueError(f"{path} contains invalid grid spacing")
    return poses, fields, yaw_step, tilt_step


class PoseEstimator:
    """Return the nearest precomputed pose using a six-dimensional KD-tree."""

    def __init__(self, model_path=DEFAULT_MODEL_PATH, lookup_path=LOOKUP_PATH):
        self.model = load_model(model_path)
        if not self.model.get("calibration_gate_passed", False):
            raise ValueError(f"{model_path} has not passed its calibration gate")
        (
            self.grid_poses,
            self.grid_fields_mT,
            self.yaw_step,
            self.tilt_step,
        ) = load_lookup_table(lookup_path, model_path)
        self.tree = cKDTree(self.grid_fields_mT)

    def estimate(self, measured_mT):
        measured = np.asarray(measured_mT, dtype=float)
        if measured.shape != (6,) or not np.isfinite(measured).all():
            raise ValueError("measurement must contain six finite channels")
        distance, index = self.tree.query(measured, k=1)
        return {
            "angles_deg": self.grid_poses[int(index)].copy(),
            "model_rms_mT": float(distance / np.sqrt(6.0)),
        }
