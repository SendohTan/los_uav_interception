import unittest

import numpy as np

from los_uav_interception import AIRCRAFT_LIMITS, PointMassUAV


class DynamicsTest(unittest.TestCase):
    def test_command_delay_defers_acceleration(self) -> None:
        model = PointMassUAV(
            AIRCRAFT_LIMITS["A"],
            dt=0.1,
            command_delay_steps=1,
        )
        model.step(np.array([3.0, 0.0, 0.0]))
        np.testing.assert_allclose(model.acceleration, np.zeros(3))
        model.step(np.array([3.0, 0.0, 0.0]))
        self.assertGreater(model.acceleration[0], 0.0)

    def test_jerk_limit_is_applied(self) -> None:
        limits = AIRCRAFT_LIMITS["A"]
        model = PointMassUAV(limits, dt=0.1)
        model.step(np.array([3.0, 0.0, 0.0]))
        self.assertLessEqual(
            np.linalg.norm(model.acceleration),
            limits.max_jerk * model.dt + 1e-9,
        )

    def test_speed_limit_is_applied(self) -> None:
        limits = AIRCRAFT_LIMITS["A"]
        model = PointMassUAV(
            limits,
            dt=0.1,
            velocity=np.array([limits.max_horizontal_speed, 0.0, 0.0]),
        )
        for _ in range(20):
            model.step(np.array([3.0, 0.0, 0.0]))
        self.assertLessEqual(
            np.linalg.norm(model.velocity[:2]),
            limits.max_horizontal_speed + 1e-9,
        )


if __name__ == "__main__":
    unittest.main()
