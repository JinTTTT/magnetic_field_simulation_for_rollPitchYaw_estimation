#!/usr/bin/env python3
"""One command: report model performance on the verification data.

Runs the IMU heading compensation of the verification labels and then the
evaluation, with the default model and correction. Yaw numbers measure
pose-dependent + random model error (the smooth-in-time heading part is
removed against the model itself); absolute yaw is checked separately with
dial-labeled poses.
"""

import subprocess
import sys

STEPS = (
    ["compensate_verification_yaw.py", "--force"],
    ["evaluate_physical_model.py",
     "--data", "verification_data_yaw_compensated.csv",
     "--yaw-column", "yaw_compensated_deg",
     "--yaw-margin-deg", "10",
     "--global-starts", "3",
     "--output", "physical_model_compensated_verification_report.json",
     "--predictions-output",
     "physical_model_compensated_verification_predictions.csv",
     "--force"],
)


def main():
    for step in STEPS:
        print(f"\n=== {step[0]} ===")
        result = subprocess.run([sys.executable, *step])
        if result.returncode != 0:
            raise SystemExit(result.returncode)


if __name__ == "__main__":
    main()
