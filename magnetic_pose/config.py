"""Runtime configuration loading."""

import json
from pathlib import Path

import numpy as np

from .tlv493d import READER_TYPE


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "config" / "model.json"
OFFSETS_PATH = ROOT / "config" / "sensor_offsets.json"
LOOKUP_PATH = ROOT / "config" / "pose_lookup.npz"
VERIFICATION_PATH = ROOT / "data" / "verification.csv"
VERIFICATION_REPORT_PATH = ROOT / "results" / "verification_report.json"


def load_sensor_offsets(path=OFFSETS_PATH):
    with Path(path).open() as source:
        data = json.load(source)
    if data.get("reader_type") != READER_TYPE:
        raise ValueError(f"{path} uses an unsupported sensor reader")
    offsets = np.asarray(data["offsets_mT"]["S1"] + data["offsets_mT"]["S2"])
    if offsets.shape != (6,) or not np.isfinite(offsets).all():
        raise ValueError(f"{path} must contain six finite offsets")
    return offsets
