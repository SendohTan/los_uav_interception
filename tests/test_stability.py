import unittest

import numpy as np

from los_uav_interception import analyze_oscillation


class StabilityMetricsTest(unittest.TestCase):
    def test_constant_command_is_not_oscillatory(self) -> None:
        desired = np.tile(np.array([10.0, 0.0, 0.0]), (100, 1))
        actual = desired.copy()
        acceleration = np.zeros_like(desired)
        metrics = analyze_oscillation(desired, actual, acceleration, 0.1)
        self.assertFalse(metrics.oscillatory)
        self.assertEqual(metrics.maximum_command_reversal_rate_hz, 0.0)

    def test_repeated_high_energy_reversal_is_oscillatory(self) -> None:
        time = np.arange(200) * 0.1
        desired = np.column_stack(
            [np.zeros_like(time), 4.0 * np.sin(2.0 * np.pi * time), np.zeros_like(time)]
        )
        actual = np.zeros_like(desired)
        acceleration = desired.copy()
        metrics = analyze_oscillation(desired, actual, acceleration, 0.1)
        self.assertTrue(metrics.oscillatory)


if __name__ == "__main__":
    unittest.main()
