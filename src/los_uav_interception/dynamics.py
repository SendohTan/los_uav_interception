from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class AircraftLimits:
    max_horizontal_speed: float
    max_climb_speed: float
    max_descent_speed: float
    max_horizontal_acceleration: float
    max_vertical_acceleration: float
    max_jerk: float


AIRCRAFT_LIMITS = {
    "A": AircraftLimits(20.0, 8.0, 6.0, 3.0, 2.0, 5.0),
    "B": AircraftLimits(45.0, 15.0, 10.0, 10.0, 5.0, 10.0),
    "C": AircraftLimits(20.0, 8.0, 6.0, 3.0, 2.0, 5.0),
    "D": AircraftLimits(30.0, 10.0, 8.0, 6.0, 3.0, 6.0),
}


def limit_acceleration(command: np.ndarray, limits: AircraftLimits) -> np.ndarray:
    acceleration = np.asarray(command, dtype=np.float64).copy()
    horizontal_norm = float(np.linalg.norm(acceleration[:2]))
    if horizontal_norm > limits.max_horizontal_acceleration:
        acceleration[:2] *= limits.max_horizontal_acceleration / horizontal_norm
    acceleration[2] = np.clip(
        acceleration[2],
        -limits.max_vertical_acceleration,
        limits.max_vertical_acceleration,
    )
    return acceleration


def velocity_along_direction(
    direction: np.ndarray,
    limits: AircraftLimits,
    ratio: float = 1.0,
) -> np.ndarray:
    direction = np.asarray(direction, dtype=np.float64)
    norm = float(np.linalg.norm(direction))
    if norm < 1e-12:
        return np.zeros(3, dtype=np.float64)

    unit = direction / norm
    speed_limits: list[float] = []
    horizontal_norm = float(np.linalg.norm(unit[:2]))
    if horizontal_norm > 1e-12:
        speed_limits.append(limits.max_horizontal_speed / horizontal_norm)
    if abs(unit[2]) > 1e-12:
        vertical_limit = (
            limits.max_descent_speed if unit[2] > 0.0 else limits.max_climb_speed
        )
        speed_limits.append(vertical_limit / abs(unit[2]))

    maximum = min(speed_limits) if speed_limits else limits.max_horizontal_speed
    return unit * np.clip(ratio, 0.0, 1.0) * maximum


class PointMassUAV:
    """NED point-mass dynamics with speed, acceleration, and jerk limits."""

    def __init__(
        self,
        limits: AircraftLimits,
        dt: float = 0.1,
        position: np.ndarray | None = None,
        velocity: np.ndarray | None = None,
    ) -> None:
        self.limits = limits
        self.dt = float(dt)
        self.position = np.zeros(3, dtype=np.float64)
        self.velocity = np.zeros(3, dtype=np.float64)
        self.acceleration = np.zeros(3, dtype=np.float64)
        self.reset(position, velocity)

    def reset(
        self,
        position: np.ndarray | None = None,
        velocity: np.ndarray | None = None,
    ) -> None:
        self.position = np.zeros(3, dtype=np.float64) if position is None else np.asarray(
            position, dtype=np.float64
        ).copy()
        self.velocity = np.zeros(3, dtype=np.float64) if velocity is None else np.asarray(
            velocity, dtype=np.float64
        ).copy()
        self.acceleration = np.zeros(3, dtype=np.float64)

    def step(self, acceleration_command: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        limited = limit_acceleration(acceleration_command, self.limits)
        acceleration_delta = limited - self.acceleration
        maximum_delta = self.limits.max_jerk * self.dt
        delta_norm = float(np.linalg.norm(acceleration_delta))
        if delta_norm > maximum_delta:
            acceleration_delta *= maximum_delta / delta_norm
        self.acceleration += acceleration_delta

        self.velocity += self.acceleration * self.dt
        horizontal_speed = float(np.linalg.norm(self.velocity[:2]))
        if horizontal_speed > self.limits.max_horizontal_speed:
            self.velocity[:2] *= self.limits.max_horizontal_speed / horizontal_speed
        self.velocity[2] = np.clip(
            self.velocity[2],
            -self.limits.max_climb_speed,
            self.limits.max_descent_speed,
        )
        self.position += self.velocity * self.dt
        return self.position.copy(), self.velocity.copy()
