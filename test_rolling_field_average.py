#!/usr/bin/env python3
"""Unit tests for the live rolling magnetic-field average."""

import unittest

import numpy as np

from tools.rolling_field_average import RollingFieldAverage


class RollingFieldAverageTests(unittest.TestCase):
    def test_window_replaces_oldest_sample(self):
        average = RollingFieldAverage(window_size=3, channel_count=2)
        average.append((1.0, 10.0), 0.0, 0.1)
        average.append((2.0, 20.0), 0.1, 0.2)
        third = average.wait_for_snapshot(after_version=1, timeout=0.01)
        self.assertEqual(third.sample_count, 2)
        self.assertFalse(third.ready)

        average.append((3.0, 30.0), 0.2, 0.3)
        full = average.snapshot()
        self.assertTrue(full.ready)
        np.testing.assert_allclose(full.mean_mT, (2.0, 20.0))
        self.assertEqual((full.start_time, full.end_time), (0.0, 0.3))

        average.append((7.0, 70.0), 0.3, 0.4)
        rolled = average.snapshot()
        np.testing.assert_allclose(rolled.mean_mT, (4.0, 40.0))
        self.assertEqual((rolled.start_time, rolled.end_time), (0.1, 0.4))
        self.assertEqual(rolled.version, 4)

    def test_rejects_invalid_values_and_timestamps(self):
        average = RollingFieldAverage(window_size=2, channel_count=2)
        with self.assertRaises(ValueError):
            average.append((1.0,), 0.0, 0.1)
        with self.assertRaises(ValueError):
            average.append((1.0, np.nan), 0.0, 0.1)
        with self.assertRaises(ValueError):
            average.append((1.0, 2.0), 0.2, 0.1)

    def test_close_wakes_waiters_and_prevents_append(self):
        average = RollingFieldAverage(window_size=2, channel_count=1)
        average.close()
        self.assertIsNone(average.wait_for_snapshot(timeout=0.01))
        with self.assertRaises(RuntimeError):
            average.append((1.0,), 0.0, 0.1)


if __name__ == "__main__":
    unittest.main()
