# Verification Accuracy Plot

The chart compares the lookup-table estimate with the Xsens angles in the four
raw recordings. It reproduces the live processing path: stored offset
correction, EMA filtering with `alpha = 0.2`, and nearest-neighbor KD-tree
lookup. No offline alignment or time shifting is applied.

Regenerate from the repository root:

```bash
.venv/bin/python figures/verification_accuracy/plot_verification_accuracy.py
.venv/bin/python figures/verification_accuracy/plot_random_tracking.py
```

The bars show mean absolute error. Symmetric whiskers show the population
standard deviation of the absolute error, matching a conventional mean-plus-or-
minus-standard-deviation bar chart. The JSON also retains the signed-error
standard deviation for reference.

`random_tracking.png` and `random_tracking.svg` show the magnetic estimate and
Xsens ground truth over time for yaw, pitch, and roll in the random recording.
