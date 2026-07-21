# System Pipeline Slide

The flowchart separates the offline model-identification and lookup-table build
from the real-time pose-estimation loop. The offline path starts from measured
hardware geometry, builds a baseline MagPy model, records synchronized magnetic
and IMU sweeps, optimizes the simulated geometry, and generates the dense
six-dimensional lookup. Runtime uses a filtered six-channel measurement and a
nearest-neighbor KD-tree search to return yaw, pitch, and roll.

Regenerate from the repository root:

```bash
.venv/bin/python figures/system_pipeline/plot_system_pipeline.py
```

Use `system_pipeline.png` directly in the presentation or
`system_pipeline.svg` when a scalable vector graphic is preferred.
