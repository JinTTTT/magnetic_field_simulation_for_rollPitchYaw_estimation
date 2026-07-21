# Background, Motivation, And Goal

Time: 1 minute
## SLide 1: Why Measure the joint directly?

  - Laser weeding requires accurate end-effector positioning.
  - Current joint angle is inferred from the kinematic model.
  - Wear and mechanical play can make the model differ from the real joint position.

## SLide 2 : project goal

Develop a compact, low-cost magnetic sensing system to estimate the rod-end joint’s yaw, pitch, and roll in real time.

- The estimate can correct the kinematic model and support wear detection.

# Requirements

Time: 1 minute

Working range: yaw +/-60 degrees; pitch and roll +/-20 degrees.

## Slide3: System Requirements

   Requirement         Minimum                             Desired
  ━━━━━━━━━━━━━━━━━━  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
   Pose information    Yaw                                 Yaw, pitch, and roll
  ──────────────────  ──────────────────────────────────  ──────────────────────────────────────────────────
   Accuracy            ±10%                                ±5%
  ──────────────────  ──────────────────────────────────  ──────────────────────────────────────────────────
   Update rate         10 Hz                               100 Hz
  ──────────────────  ──────────────────────────────────  ──────────────────────────────────────────────────
   Mounting            Adhesive mounting for one sensor    3D-printed clip-on mount for up to three sensors


# Final Solution Concept

Time: 6 minutes
## slide 1
Show magnetic_field_lines.png.

Can a rotating sensor use the magnetic field to distinguish joint orientation?? 
## slide 2
the magnetic field is spatially structured. As the sensor moves and rotates with the joint, its local Bx, By, and Bz readings may change.
Show configuration_ideal.

# slide 3  What Does the Sensor Observe in its own frame?

Show the three ideal sweep plots side by side.

Roll is not observable in this centered configuration.

why? The cylindrical magnet is symmetric about its magnet axis, which is also the joint roll axis in this configuration. Rotation about that axis does not change the sensor’s relative magnetic situation.

## slide 4 Can We Break This Symmetry?
Show configuration_offcenter.

- Move the magnet away from the joint centre.
- The sensor now sees different field regions during roll.

## slide 5 What Does the Sensor Observe in its own frame?

 Show the off-centre yaw, pitch, and roll sweep plots.

Off-centre geometry makes the single sensor sensitive to all three rotation axes.

## slide 6 Final concept

  - Fixed cylindrical magnet
  - Two TLV493D sensors on the rotating joint side, 2 sensors provide the final, more distinctive measurement signature
  - input : 6 channel reading of 2 snesors
  - Output: yaw, pitch, roll

## slide 7  System pipeline and architecure

show  system_pipeline.png




# Development Journey And Evidence

Time: 4 minutes 30 seconds

# Validation And Takeaway

Time: 1 minute 30 seconds

# Schedule And Status

Time: 1 minute
