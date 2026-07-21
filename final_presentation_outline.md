# Background, Motivation, And Goal

Time: 1 minute

## Slide 1: Why Measure the Joint Directly?

  - Laser weeding requires accurate end-effector positioning.
  - Current joint angle is inferred from the kinematic model.
  - Wear and mechanical play can make the model differ from the real joint position.

## Slide 2: Project Goal

Develop a compact, low-cost magnetic sensing system to estimate the rod-end joint’s yaw, pitch, and roll in real time.

- The estimate can correct the kinematic model and support wear detection.

# Requirements

Time: 1 minute

Working range: yaw +/-60 degrees; pitch and roll +/-20 degrees.

## Slide 3: System Requirements

   Requirement         Minimum                             Desired
  ━━━━━━━━━━━━━━━━━━  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
   Pose information    Yaw                                 Yaw, pitch, and roll
  ──────────────────  ──────────────────────────────────  ──────────────────────────────────────────────────
   Accuracy            ±10%   (12 deg)                             ±5% (6deg, 2deg, 2deg)
  ──────────────────  ──────────────────────────────────  ──────────────────────────────────────────────────
   Update rate         10 Hz                               100 Hz
  ──────────────────  ──────────────────────────────────  ──────────────────────────────────────────────────
   Mounting            Adhesive mounting for one sensor    3D-printed clip-on mount for up to three sensors


# Final Solution Concept

Time: 6 minutes

## Slide 1: Can a Magnetic Field Encode Joint Orientation?

Show `magnetic_field_lines.png`.

Can a rotating sensor use the magnetic field to distinguish joint orientation?

## Slide 2: Centered Reference Configuration

The magnetic field is spatially structured. As the sensor moves and rotates
with the joint, its local Bx, By, and Bz readings may change.

Show `configuration_ideal`.

## Slide 3: What Does the Sensor Observe in Its Own Frame?

Show the three ideal sweep plots side by side.

Roll is not observable in this centered configuration.

Why? The cylindrical magnet is symmetric about its magnet axis, which is also
the joint roll axis in this configuration. Rotation about that axis does not
change the sensor's relative magnetic situation.

## Slide 4: Can We Break This Symmetry?

Show `configuration_offcenter`.

- Move the magnet away from the joint centre.
- The sensor now sees different field regions during roll.

## Slide 5: What Does the Sensor Observe in Its Own Frame?

Show the off-centre yaw, pitch, and roll sweep plots.

Off-centre geometry makes the single sensor sensitive to all three rotation axes.

## Slide 6: Final Hardware Concept

  - Fixed cylindrical magnet
  - Two spatially separated TLV493D sensors on the rotating joint side
  - Input: one six-channel magnetic measurement
  - The two sensors provide a more distinctive pose signature
  - Output: yaw, pitch, and roll

## Slide 7: System Pipeline And Architecture

Show `system_pipeline.png`.


# Results And Validation

## Slides 1-4: Live Tracking Videos (1-2 minutes)

Show the four videos: yaw, pitch, roll, and random motion.

## Slide 5: Estimated Angles vs Reference Angles

## Slide 6: Estimation Error

Show the accuracy and standard-deviation plot.

## Slide 7: Overall Performance

   Measure                  Result    Requirement Context
  ━━━━━━━━━━━━━━━━━━━  ━━━━━━━━━━━━━  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
   Yaw accuracy, MAE         0.71°    Desired three-axis pose estimate
  ───────────────────  ─────────────  ───────────────────────────────────────────────────────────
   Pitch accuracy, MAE       0.38°    Desired three-axis pose estimate
  ───────────────────  ─────────────  ───────────────────────────────────────────────────────────
   Roll accuracy, MAE        0.70°    Desired three-axis pose estimate
  ───────────────────  ─────────────  ───────────────────────────────────────────────────────────
   Update rate         about 27 Hz    Meets 10 Hz minimum; below 100 Hz desired
  ───────────────────  ─────────────  ───────────────────────────────────────────────────────────


# Current Limitations And Next Steps

## Slide 1: Current Limitations

- Pitch and roll are currently calibrated over +/-10 degrees, rather than the desired +/-20 degrees; the calibration data remain sparse.
- The system assumes that environmental magnetic disturbance is static and can be removed through magnet-out offset calibration. This may not hold in practice.
- The TLV493D measurement resolution is limited. A higher-resolution, lower-noise sensor could support finer estimates.
- The hardware geometry has not yet been optimized to maximize the distinction between pose signatures.

## Slide 2: Next Steps

- Develop an enclosure and evaluate mitigation of external magnetic disturbance.
- Use servo-controlled yaw, pitch, and roll sweeps to collect denser, repeatable calibration data.
- Optimize sensor and magnet placement for pose distinguishability.
- Integrate the system on the robot and assess mechanical fit, interference, and robustness.

# Schedule And Status

Show the updated schedule and completion status.
