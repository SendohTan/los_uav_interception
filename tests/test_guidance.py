import unittest

import numpy as np

from los_uav_interception import AIRCRAFT_LIMITS, LOSGuidanceController
from los_uav_interception.guidance import LOSRateEstimator


class GuidanceTest(unittest.TestCase):
    def test_constant_bearing_has_zero_los_rate(self) -> None:
        estimator = LOSRateEstimator(filter_alpha=0.85)
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


if __name__ == "__main__":
    unittest.main()
