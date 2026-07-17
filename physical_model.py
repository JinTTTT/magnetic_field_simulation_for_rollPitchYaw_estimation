#!/usr/bin/env python3
"""Reusable finite-cylinder forward model for the clean calibration pipeline."""

import json
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/magnetic_orientation_matplotlib")

import magpylib as magpy
import numpy as np
from scipy.spatial.transform import Rotation


CHANNELS = ("S1_Bx", "S1_By", "S1_Bz", "S2_Bx", "S2_By", "S2_Bz")


def sensor_home_positions_mm(sensor_plane_z_mm, ring_radius_mm,
                             angular_separation_deg):
    azimuths = np.radians((0.0, -float(angular_separation_deg)))
    return np.column_stack((
        ring_radius_mm * np.cos(azimuths),
        ring_radius_mm * np.sin(azimuths),
        np.full(2, sensor_plane_z_mm, dtype=float),
    ))


def effective_pose_rotations(angles_deg, imu_axis_alignment_deg):
    angles = np.atleast_2d(np.asarray(angles_deg, dtype=float))
    if angles.shape[1] != 3:
        raise ValueError("angles must have columns yaw, pitch, roll")
    raw = Rotation.from_euler("ZYX", angles, degrees=True)
    alignment = Rotation.from_euler("z", imu_axis_alignment_deg, degrees=True)
    return alignment * raw * alignment.inv()


def predict_mT(angles_deg, model):
    """Predict six offset-corrected magnetic channels for yaw/pitch/roll rows."""
    magnet = model["magnet"]
    sensors = model["sensors"]
    pose = effective_pose_rotations(
        angles_deg, model["imu_axis_alignment_deg"]
    )
    count = len(np.atleast_2d(angles_deg))

    base = Rotation.from_euler("y", 90.0, degrees=True)
    tilt = Rotation.from_rotvec(np.radians((
        0.0,
        magnet["axis_tilt_y_deg"],
        magnet["axis_tilt_z_deg"],
    )))
    source = magpy.magnet.Cylinder(
        position=np.asarray(magnet["center_mm"], dtype=float) / 1000.0,
        orientation=tilt * base,
        dimension=np.asarray(magnet["dimension_mm"], dtype=float) / 1000.0,
        polarization=(0.0, 0.0, magnet["signed_polarization_T"]),
    )

    homes = sensor_home_positions_mm(
        sensors["sensor_plane_z_mm"],
        sensors["ring_radius_mm"],
        sensors["angular_separation_deg"],
    )
    output = []
    for home, matrix in zip(homes, sensors["rig_to_chip_rotation_matrices"]):
        positions_m = pose.apply(np.tile(home / 1000.0, (count, 1)))
        field_world_mT = np.atleast_2d(source.getB(positions_m)) * 1000.0
        field_rig_mT = pose.inv().apply(field_world_mT)
        chip_rotation = Rotation.from_matrix(np.asarray(matrix, dtype=float))
        output.append(chip_rotation.apply(field_rig_mT))
    return np.hstack(output)


def load_model(path=Path("physical_model.json")):
    with Path(path).open() as source:
        payload = json.load(source)
    if payload.get("schema_version") != 1:
        raise ValueError(f"unsupported physical model schema in {path}")
    return payload
