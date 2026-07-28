"""Use a LOS-rate measurement supplied by a radar or tracking filter."""

import numpy as np

from los_uav_interception import AIRCRAFT_LIMITS, LOSGuidanceController


controller = LOSGuidanceController()
command = controller.command(
    relative_position=np.array([250.0, 30.0, -15.0]),
    own_velocity=np.array([16.0, 1.0, 0.0]),
    measured_los_rate=np.array([0.001, -0.003, 0.008]),
    limits=AIRCRAFT_LIMITS["A"],
    dt=0.1,
)
print(command.acceleration)
