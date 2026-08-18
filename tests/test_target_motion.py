import unittest

import numpy as np

from los_uav_interception.dynamics import (
    AIRCRAFT_LIMITS,
    PointMassUAV,
    velocity_along_direction,
)
from los_uav_interception.simulation import (
    EVALUATION_MOTION_MODES,
    GoalDirectedTargetController,
)


class TargetMotionTest(unittest.TestCase):
    def test_formal_evaluation_uses_six_motion_types(self) -> None:
        self.assertEqual(
            EVALUATION_MOTION_MODES,
            ("line", "cosine", "arc", "random", "multi_sine", "bspline"),
        )

    def test_all_motion_types_remain_goal_directed(self) -> None:
        initial_position = np.array([400.0, 30.0, -20.0])
        limits = AIRCRAFT_LIMITS["C"]
        for index, motion in enumerate(EVALUATION_MOTION_MODES):
            target = PointMassUAV(
                limits,
                dt=0.1,
                position=initial_position,
                velocity=velocity_along_direction(-initial_position, limits, 0.82),
            )
            controller = GoalDirectedTargetController(
                motion,
                0.1,
                initial_position,
                500 + index,
            )
            minimum_goal_distance = float(np.linalg.norm(target.position))
            for step in range(1000):
                target.step(controller.command(target, step * 0.1, 1e9))
                minimum_goal_distance = min(
                    minimum_goal_distance,
                    float(np.linalg.norm(target.position)),
                )
            self.assertLess(minimum_goal_distance, 1.0, motion)


if __name__ == "__main__":
    unittest.main()
