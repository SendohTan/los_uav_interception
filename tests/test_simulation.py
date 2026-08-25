import unittest

import numpy as np

from los_uav_interception import SimulationConfig, run_multi, run_single
from los_uav_interception.simulation import accurate_delayed_los_rate


class SimulationTest(unittest.TestCase):
    def test_accurate_los_rate_is_evaluated_at_delayed_frame(self) -> None:
        relative_position = np.array([300.0, 0.0, 0.0])
        relative_velocity = np.array([0.0, 30.0, 0.0])
        delayed, los_rate = accurate_delayed_los_rate(
            relative_position,
            relative_velocity,
            1.0 / 30.0,
        )
        expected_delayed = np.array([300.0, -1.0, 0.0])
        expected_rate = np.cross(expected_delayed, relative_velocity) / np.dot(
            expected_delayed,
            expected_delayed,
        )
        np.testing.assert_allclose(delayed, expected_delayed)
        np.testing.assert_allclose(los_rate, expected_rate)

    def test_single_line_interception_reaches_success_radius(self) -> None:
        result = run_single(SimulationConfig(target_motion="line"), seed=5)
        self.assertTrue(result.success)
        self.assertLess(result.minimum_distance, 0.6)
        self.assertEqual(result.target_visible.shape[1], 1)
        self.assertEqual(result.range_error_fraction.shape, result.target_visible.shape)
        self.assertEqual(result.command_actions.shape, result.interceptor_positions.shape)
        self.assertEqual(
            result.actual_accelerations.shape, result.interceptor_positions.shape
        )

    def test_multi_line_interception_reaches_success_radius(self) -> None:
        result = run_multi(SimulationConfig(target_motion="line"), seed=5)
        self.assertTrue(result.success)
        self.assertLess(result.minimum_distance, 0.6)
        self.assertEqual(result.target_visible.shape[1], 3)
        self.assertEqual(result.range_error_fraction.shape, result.target_visible.shape)
        self.assertEqual(result.command_actions.shape, result.interceptor_positions.shape)
        self.assertEqual(
            result.actual_accelerations.shape, result.interceptor_positions.shape
        )


if __name__ == "__main__":
    unittest.main()
