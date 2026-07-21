# Off-Center Configuration Figures

This configuration keeps the joint center and Sensor 1 home position at the
ideal height of `Z = 24 mm`, while raising the magnet center by `10 mm` to
`Z = 34 mm`. The sensor rotates about the fixed joint X, Y, and Z axes.

The offset breaks the magnet's symmetry about the joint X axis, so the roll
sweep now produces a measurable field change in Sensor 1's frame.

## Structure

- `plots/`: generated configuration, yaw, pitch, and roll figures
- `scripts/offcenter_configuration.py`: shared geometry and field calculations
- `scripts/sweep_plotting.py`: shared sweep styling and rotation-axis inset
- `scripts/plot_configuration_offcenter.py`: raised-magnet geometry
- `scripts/plot_sensor1_yaw_sweep.py`: yaw sweep about joint Z
- `scripts/plot_sensor1_pitch_sweep.py`: pitch sweep about joint Y
- `scripts/plot_sensor1_roll_sweep.py`: roll sweep about joint X

## Regenerate

Run from the repository root:

```bash
.venv/bin/python figures/configuration_offcenter/scripts/plot_configuration_offcenter.py
.venv/bin/python figures/configuration_offcenter/scripts/plot_sensor1_yaw_sweep.py
.venv/bin/python figures/configuration_offcenter/scripts/plot_sensor1_pitch_sweep.py
.venv/bin/python figures/configuration_offcenter/scripts/plot_sensor1_roll_sweep.py
```
