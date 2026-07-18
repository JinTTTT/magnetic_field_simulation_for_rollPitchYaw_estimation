# Recorded verification session

This directory contains byte-for-byte copies of the four raw CSV recordings and
their corresponding screen recordings from 2026-07-18.

## Data audit

| Motion | CSV rows | Data duration | Median interval | IMU yaw range | IMU pitch range | IMU roll range | Video duration |
|---|---:|---:|---:|---:|---:|---:|---:|
| Yaw | 686 | 26.875 s | 36.68 ms | 71.50° | 4.95° | 3.33° | 33.294 s |
| Roll | 506 | 19.867 s | 36.80 ms | 1.82° | 11.58° | 20.02° | 26.504 s |
| Pitch | 570 | 22.061 s | 37.19 ms | 1.87° | 21.11° | 14.58° | 29.175 s |
| Random | 981 | 38.037 s | 36.92 ms | 42.15° | 21.82° | 19.46° | 44.638 s |

All CSV files have the expected ten columns, finite numeric values, strictly
increasing timestamps, and no missing or all-zero magnetic rows. Maximum sample
gaps are below 95 ms. The intended axis has the largest IMU range in each of the
three sweep files.

The roll sweep also contains 11.58° of pitch motion, and the pitch sweep contains
14.58° of roll motion. They are therefore not pure single-axis sweeps, although
their intended axes remain dominant. The yaw sweep covers approximately -30.3°
to +41.2°, rather than the model's complete ±60° workspace.

Yaw begins within 0.011° of zero in every file, confirming that per-recording
yaw zeroing worked. Pitch and roll were not zeroed by the recorder; their first
values include home offsets of up to about 0.69° pitch and 1.59° roll.

The videos are valid H.264, 1920×1080 files and are longer than their CSVs by
roughly 6.4–7.1 seconds because they include setup and shutdown time. FFmpeg
reports duplicate/non-monotonic video timestamps in `yaw_sweeping.mp4` and
`random.mp4`. Both decode and play, but precise timestamp-based frame extraction
may require regenerating presentation timestamps or remuxing first. No such
warning was found for the roll or pitch video.
