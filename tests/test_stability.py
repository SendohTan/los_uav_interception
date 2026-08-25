import unittest

import numpy as np

from los_uav_interception import analyze_oscillation


class StabilityMetricsTest(unittest.TestCase):
    def test_constant_command_is_not_oscillatory(self) -> None:
        commands = np.zeros((100, 1, 3))
        acceleration = np.zeros_like(commands)
        metrics = analyze_oscillation(commands, acceleration, 0.1)
        self.assertFalse(metrics.oscillatory)
        self.assertEqual(metrics.max_command_reversal_rate_hz, 0.0)

    def test_repeated_high_energy_reversal_is_oscillatory(self) -> None:
        time = np.arange(200) * 0.1
        signal = np.sin(2.0 * np.pi * time)
        commands = np.column_stack(
            [np.zeros_like(time), signal, np.zeros_like(time)]
        )[:, None, :]
        acceleration = np.column_stack(
            [np.zeros_like(time), 2.0 * signal, np.zeros_like(time)]
        )[:, None, :]
        metrics = analyze_oscillation(commands, acceleration, 0.1)
        self.assertTrue(metrics.oscillatory)


if __name__ == "__main__":
    unittest.main()
