"""Shared geometry for the presentation configuration with a raised magnet."""

import numpy as np
from scipy.spatial.transform import Rotation

from magnetic_pose.model import build_magnet_source


ROTATION_CENTER_MM = np.array([0.0, 0.0, 24.0])
MAGNET_CENTER_MM = np.array([0.0, 0.0, 34.0])
MAGNET_DIMENSION_MM = np.array([10.0, 5.0])  # diameter, height
MAGNET_POLARIZATION_T = -1.1199896964866292
SENSOR_1_HOME_MM = np.array([0.0, -24.0, 24.0])

OFFCENTER_MODEL = {
    "magnet": {
        "center_mm": MAGNET_CENTER_MM.tolist(),
        "dimension_mm": MAGNET_DIMENSION_MM.tolist(),
        "axis_tilt_y_deg": 0.0,
        "axis_tilt_z_deg": 0.0,
        "signed_polarization_T": MAGNET_POLARIZATION_T,
    }
}


def build_offcenter_magnet():
    return build_magnet_source(OFFCENTER_MODEL)


def sensor_1_field_for_rotation_mT(source, rotation):
    """Return Sensor 1's field after rotating about the joint center."""
    home_offset_m = (SENSOR_1_HOME_MM - ROTATION_CENTER_MM) / 1000.0
    rotation_center_m = ROTATION_CENTER_MM / 1000.0
    position_m = rotation_center_m + rotation.apply(home_offset_m)
    field_world_t = np.asarray(source.getB(position_m.reshape(1, 3))).reshape(3)
    return rotation.inv().apply(field_world_t) * 1000.0


def sensor_1_field_at_yaw_mT(source, yaw_deg):
    """Return Sensor 1's field in its co-rotating frame."""
    pose = Rotation.from_euler("z", yaw_deg, degrees=True)
    return sensor_1_field_for_rotation_mT(source, pose)


def sensor_1_field_at_pitch_mT(source, pitch_deg):
    """Return Sensor 1's field for pitch about the joint-centered Y axis."""
    pose = Rotation.from_euler("y", pitch_deg, degrees=True)
    return sensor_1_field_for_rotation_mT(source, pose)


def sensor_1_field_at_roll_mT(source, roll_deg):
    """Return Sensor 1's field for roll about the joint-centered X axis."""
    pose = Rotation.from_euler("x", roll_deg, degrees=True)
    return sensor_1_field_for_rotation_mT(source, pose)
