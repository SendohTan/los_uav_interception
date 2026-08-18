import unittest

import numpy as np

from los_uav_interception import (
    AIRCRAFT_LIMITS,
    GuidanceSafetyGate,
    LOSGuidanceController,
)


class GuidanceSafetyGateTest(unittest.TestCase):
    def setUp(self) -> None:
        self.gate = GuidanceSafetyGate(
            LOSGuidanceController(),
            AIRCRAFT_LIMITS["A"],
        )
        self.position = np.array([300.0, 20.0, -10.0])
        self.velocity = np.array([15.0, 0.0, 0.0])

    def command(self, current: float, measured: float, visible: bool = True):
        return self.gate.command(
            current_time_s=current,
            measurement_time_s=measured,
            target_visible=visible,
            relative_position_ned_m=self.position,
            own_velocity_ned_mps=self.velocity,
        )

    def test_first_sample_warms_up_estimator(self) -> None:
        self.assertIsNone(self.command(1.0, 1.0))
        self.assertIsNotNone(self.command(1.1, 1.1))

    def test_stale_sample_releases_guidance(self) -> None:
        self.assertIsNone(self.command(1.0, 0.7))

    def test_target_loss_resets_guidance(self) -> None:
        self.assertIsNone(self.command(1.0, 1.0))
        self.assertIsNone(self.command(1.1, 1.1, visible=False))
        self.assertIsNone(self.command(1.2, 1.2))


if __name__ == "__main__":
    unittest.main()
