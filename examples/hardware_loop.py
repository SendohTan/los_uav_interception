"""Minimal integration pattern for a flight computer or hardware-in-the-loop loop."""

import numpy as np

from los_uav_interception import (
    AIRCRAFT_LIMITS,
    GuidanceSafetyGate,
    LOSGuidanceConfig,
    LOSGuidanceController,
)


controller = LOSGuidanceController(LOSGuidanceConfig.flight_test())
limits = AIRCRAFT_LIMITS["A"]
safety_gate = GuidanceSafetyGate(controller, limits)


def control_tick(
    current_time_s: float,
    measurement_time_s: float,
    target_visible: bool,
    own_velocity_ned_mps: np.ndarray,
    observed_relative_position_ned_m: np.ndarray,
) -> np.ndarray | None:
    command = safety_gate.command(
        current_time_s=current_time_s,
        measurement_time_s=measurement_time_s,
        target_visible=target_visible,
        relative_position_ned_m=observed_relative_position_ned_m,
        own_velocity_ned_mps=own_velocity_ned_mps,
    )
    return None if command is None else command.acceleration


if __name__ == "__main__":
    control_tick(
        current_time_s=1.0,
        measurement_time_s=1.0,
        target_visible=True,
        own_velocity_ned_mps=np.array([15.0, 0.0, 0.0]),
        observed_relative_position_ned_m=np.array([300.0, 40.0, -20.0]),
    )
    example_acceleration = control_tick(
        current_time_s=1.1,
        measurement_time_s=1.1,
        target_visible=True,
        own_velocity_ned_mps=np.array([15.0, 0.0, 0.0]),
        observed_relative_position_ned_m=np.array([298.5, 39.8, -19.9]),
    )
    print(example_acceleration)
