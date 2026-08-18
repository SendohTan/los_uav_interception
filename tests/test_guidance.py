import unittest

import numpy as np

from los_uav_interception import (
    AIRCRAFT_LIMITS,
    LOSGuidanceConfig,
    LOSGuidanceController,
)
from los_uav_interception.guidance import LOSRateEstimator


class GuidanceTest(unittest.TestCase):
    def test_constant_bearing_has_zero_los_rate(self) -> None:
        estimator = LOSRateEstimator(filter_alpha=0.85, window_size=2)
        first = estimator.update(np.array([1.0, 0.0, 0.0]), dt=0.1)
        second = estimator.update(np.array([1.0, 0.0, 0.0]), dt=0.1)
        np.testing.assert_allclose(first, np.zeros(3), atol=1e-12)
        np.testing.assert_allclose(second, np.zeros(3), atol=1e-12)

    def test_controller_respects_acceleration_limits(self) -> None:
        limits = AIRCRAFT_LIMITS["A"]
        controller = LOSGuidanceController()
        controller.command(
            relative_position=np.array([300.0, 0.0, 0.0]),
            own_velocity=np.array([15.0, 0.0, 0.0]),
            limits=limits,
            dt=0.1,
        )
        command = controller.command(
            relative_position=np.array([300.0, 30.0, -20.0]),
            own_velocity=np.array([15.0, 0.0, 0.0]),
            limits=limits,
            dt=0.1,
        )
        self.assertLessEqual(
            np.linalg.norm(command.acceleration[:2]),
            limits.max_horizontal_acceleration + 1e-9,
        )
        self.assertLessEqual(
            abs(command.acceleration[2]),
            limits.max_vertical_acceleration + 1e-9,
        )

    def test_stable_profile_limits_first_velocity_command_change(self) -> None:
        controller = LOSGuidanceController()
        command = controller.command(
            relative_position=np.array([300.0, 0.0, 0.0]),
            own_velocity=np.zeros(3),
            measured_los_rate=np.zeros(3),
            limits=AIRCRAFT_LIMITS["A"],
            dt=0.1,
        )
        self.assertLessEqual(
            np.linalg.norm(command.desired_velocity[:2]),
            0.4 + 1e-12,
        )

    def test_soft_threshold_rejects_small_los_rate(self) -> None:
        controller = LOSGuidanceController()
        command = controller.command(
            relative_position=np.array([300.0, 0.0, 0.0]),
            own_velocity=np.array([15.0, 0.0, 0.0]),
            measured_los_rate=np.array([0.0, 0.0, 0.001]),
            limits=AIRCRAFT_LIMITS["A"],
            dt=0.1,
        )
        np.testing.assert_allclose(command.line_of_sight_rate, np.zeros(3))

    def test_legacy_profile_preserves_unfiltered_parameters(self) -> None:
        config = LOSGuidanceConfig.legacy()
        self.assertEqual(config.pursuit_gain, 0.05)
        self.assertEqual(config.los_rate_soft_threshold_rad_s, 0.0)
        self.assertEqual(config.los_rate_window_size, 2)
        self.assertEqual(config.desired_velocity_filter_tau_s, 0.0)
        self.assertEqual(config.desired_velocity_slew_limit_mps2, 0.0)
        self.assertEqual(config.acceleration_filter_tau_s, 0.0)

    def test_invalid_dt_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            LOSGuidanceController().command(
                relative_position=np.array([300.0, 0.0, 0.0]),
                own_velocity=np.array([15.0, 0.0, 0.0]),
                limits=AIRCRAFT_LIMITS["A"],
                dt=0.0,
            )


if __name__ == "__main__":
    unittest.main()
