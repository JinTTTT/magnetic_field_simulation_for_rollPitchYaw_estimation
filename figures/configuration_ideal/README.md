# Ideal Configuration Figures

This folder contains the centered, untilted magnet configuration used for the
presentation figures. Sensor 1 and the magnet share the same height, with no
off-center displacement.

## Structure

- `plots/`: generated presentation images
- `scripts/ideal_configuration.py`: shared ideal geometry and field calculations
- `scripts/sweep_plotting.py`: shared sweep styling and rotation-axis inset
- `scripts/plot_configuration_ideal.py`: magnet and Sensor 1 geometry
- `scripts/plot_sensor1_yaw_sweep.py`: yaw sweep about magnet Z
- `scripts/plot_sensor1_pitch_sweep.py`: pitch sweep about magnet Y
- `scripts/plot_sensor1_roll_sweep.py`: roll sweep about magnet X

## Regenerate

Run from the repository root:

```bash
.venv/bin/python figures/configuration_ideal/scripts/plot_configuration_ideal.py
.venv/bin/python figures/configuration_ideal/scripts/plot_sensor1_yaw_sweep.py
.venv/bin/python figures/configuration_ideal/scripts/plot_sensor1_pitch_sweep.py
.venv/bin/python figures/configuration_ideal/scripts/plot_sensor1_roll_sweep.py
```
