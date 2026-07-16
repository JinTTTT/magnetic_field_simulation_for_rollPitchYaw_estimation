#!/usr/bin/env python3
"""Record an independent verification dataset with the live calibration UI."""

from log_calibration import main


if __name__ == "__main__":
    main(default_csv="verification_data.csv", default_target=25,
         dataset_name="verification")
