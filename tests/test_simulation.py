import unittest

from los_uav_interception import SimulationConfig, run_multi, run_single


class SimulationTest(unittest.TestCase):
    def test_single_line_interception_reaches_success_radius(self) -> None:
        result = run_single(SimulationConfig(target_motion="line"), seed=5)
        self.assertTrue(result.success)
        self.assertLess(result.minimum_distance, 0.6)

    def test_multi_line_interception_reaches_success_radius(self) -> None:
        result = run_multi(SimulationConfig(target_motion="line"), seed=5)
        self.assertTrue(result.success)
        self.assertLess(result.minimum_distance, 0.6)


if __name__ == "__main__":
    unittest.main()
