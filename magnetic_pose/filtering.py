"""Small stateful filters shared by the live acquisition programs."""

import numpy as np


DEFAULT_EMA_ALPHA = 0.2


class ExponentialMovingAverage:
    """Apply an exponential moving average to fixed-shape numeric samples."""

    def __init__(self, alpha):
        self.alpha = float(alpha)
        if not 0.0 < self.alpha <= 1.0:
            raise ValueError("EMA alpha must be greater than zero and at most one")
        self.value = None

    def update(self, sample):
        sample = np.asarray(sample, dtype=float)
        if not np.isfinite(sample).all():
            raise ValueError("EMA sample must contain finite values")
        if self.value is None:
            self.value = sample.copy()
        elif sample.shape != self.value.shape:
            raise ValueError("EMA sample shape changed")
        else:
            self.value += self.alpha * (sample - self.value)
        return self.value.copy()
