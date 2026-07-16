"""Smooth pose-dependent correction for the six magnetic field channels."""

import numpy as np

YAW_HARMONICS = 3
DEFAULT_ALPHAS = (1e-4, 1e-3, 1e-2, 1e-1, 1.0, 10.0)


def correction_features(poses):
    """Return smooth Fourier-yaw and quadratic pitch/roll features."""
    poses = np.atleast_2d(np.asarray(poses, dtype=float))
    yaw = np.radians(poses[:, 0])
    pitch = poses[:, 1] / 10.0
    roll = poses[:, 2] / 10.0

    yaw_basis = [np.ones(len(poses))]
    for harmonic in range(1, YAW_HARMONICS + 1):
        yaw_basis.extend([
            np.sin(harmonic * yaw), np.cos(harmonic * yaw)
        ])
    pose_basis = [
        np.ones(len(poses)), pitch, roll, pitch ** 2, roll ** 2, pitch * roll
    ]
    return np.column_stack([
        yaw_term * pose_term
        for yaw_term in yaw_basis
        for pose_term in pose_basis
    ])


def fit_correction(poses, residuals_mT, alpha):
    """Fit a ridge-regularized six-channel residual correction."""
    poses = np.atleast_2d(np.asarray(poses, dtype=float))
    residuals = np.atleast_2d(np.asarray(residuals_mT, dtype=float))
    design = correction_features(poses)
    feature_scale = np.sqrt(np.mean(design ** 2, axis=0))
    feature_scale[feature_scale < 1e-12] = 1.0
    scaled = design / feature_scale

    penalty = np.eye(scaled.shape[1]) * alpha
    penalty[0, 0] = 0.0
    coefficients = np.linalg.solve(
        scaled.T @ scaled + penalty, scaled.T @ residuals)
    return {
        "type": "fourier_yaw_quadratic_pitch_roll",
        "yaw_harmonics": YAW_HARMONICS,
        "alpha": float(alpha),
        "feature_scale": feature_scale.tolist(),
        "coefficients_mT": coefficients.tolist(),
        "trained_range_deg": {
            "yaw": [float(poses[:, 0].min()), float(poses[:, 0].max())],
            "pitch": [float(poses[:, 1].min()), float(poses[:, 1].max())],
            "roll": [float(poses[:, 2].min()), float(poses[:, 2].max())],
        },
    }


def predict_correction(correction, poses):
    """Return the correction in mT for one or more [yaw,pitch,roll] poses."""
    if not correction:
        return np.zeros((len(np.atleast_2d(poses)), 6), dtype=float)
    scale = np.asarray(correction["feature_scale"], dtype=float)
    coefficients = np.asarray(correction["coefficients_mT"], dtype=float)
    return (correction_features(poses) / scale) @ coefficients
