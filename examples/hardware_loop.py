"""Minimal integration pattern for a flight computer or hardware-in-the-loop loop."""

import numpy as np

from los_uav_interception import (
    AIRCRAFT_LIMITS,
    LOSGuidanceController,
)


controller = LOSGuidanceController()
limits = AIRCRAFT_LIMITS["A"]
dt = 0.1


def control_tick(
    own_velocity_ned_mps: np.ndarray,
    observed_relative_position_ned_m: np.ndarray,
) -> np.ndarray:
    command = controller.command(
        relative_position=observed_relative_position_ned_m,
        own_velocity=own_velocity_ned_mps,
        limits=limits,
        dt=dt,
    )
    return command.acceleration


if __name__ == "__main__":
    example_acceleration = control_tick(
        own_velocity_ned_mps=np.array([15.0, 0.0, 0.0]),
        observed_relative_position_ned_m=np.array([300.0, 40.0, -20.0]),
    )
    print(example_acceleration)
