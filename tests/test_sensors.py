import unittest

import numpy as np

from los_uav_interception import CameraFOV


class CameraFOVTest(unittest.TestCase):
    def setUp(self) -> None:
        self.fov = CameraFOV(horizontal_deg=24.0, vertical_deg=16.0)
        self.forward = np.array([1.0, 0.0, 0.0])

    def direction(self, horizontal_deg: float, vertical_deg: float) -> np.ndarray:
        horizontal = np.radians(horizontal_deg)
        vertical = np.radians(vertical_deg)
        return np.array(
            [
                np.cos(vertical) * np.cos(horizontal),
                np.cos(vertical) * np.sin(horizontal),
                np.sin(vertical),
            ]
        )

    def test_forward_target_is_visible(self) -> None:
        self.assertTrue(self.fov.contains(self.direction(0.0, 0.0), self.forward))

    def test_horizontal_half_angle_is_enforced(self) -> None:
        self.assertTrue(self.fov.contains(self.direction(12.0, 0.0), self.forward))
        self.assertFalse(self.fov.contains(self.direction(12.1, 0.0), self.forward))

    def test_vertical_half_angle_is_enforced(self) -> None:
        self.assertTrue(self.fov.contains(self.direction(0.0, 8.0), self.forward))
        self.assertFalse(self.fov.contains(self.direction(0.0, 8.1), self.forward))


if __name__ == "__main__":
    unittest.main()
