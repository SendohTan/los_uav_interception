from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .dynamics import (
    AircraftLimits,
    limit_acceleration,
    velocity_along_direction,
)


@dataclass(frozen=True)
class LOSGuidanceConfig:
    navigation_constant: float = 4.0
    pursuit_gain: float = 0.05
    lead_time: float = 3.0
    desired_speed_ratio: float = 0.98
    los_rate_filter_alpha: float = 0.85


@dataclass(frozen=True)
class GuidanceCommand:
    acceleration: np.ndarray
    normalized_action: np.ndarray
    line_of_sight: np.ndarray
    line_of_sight_rate: np.ndarray
    aim_direction: np.ndarray
    distance: float


class LOSRateEstimator:
    """Estimate the LOS angular-rate vector from consecutive bearing samples."""

    def __init__(self, filter_alpha: float = 0.85) -> None:
        if not 0.0 <= filter_alpha < 1.0:
            raise ValueError("filter_alpha must be in [0, 1)")
        self.filter_alpha = float(filter_alpha)
        self.previous_los: np.ndarray | None = None
        self.filtered_rate: np.ndarray | None = None

    def reset(self) -> None:
        self.previous_los = None
        self.filtered_rate = None

    def update(self, line_of_sight: np.ndarray, dt: float) -> np.ndarray:
        los = np.asarray(line_of_sight, dtype=np.float64)
        norm = float(np.linalg.norm(los))
        if norm < 1e-12:
            return np.zeros(3, dtype=np.float64)
        los /= norm

        if self.previous_los is None:
            raw_rate = np.zeros(3, dtype=np.float64)
        else:
            cross = np.cross(self.previous_los, los)
            sine = float(np.linalg.norm(cross))
            cosine = float(np.clip(np.dot(self.previous_los, los), -1.0, 1.0))
            angle = float(np.arctan2(sine, cosine))
            raw_rate = (
                np.zeros(3, dtype=np.float64)
                if sine < 1e-12
                else cross / sine * angle / float(dt)
            )

        if self.filtered_rate is None:
            filtered = raw_rate
        else:
            alpha = self.filter_alpha
            filtered = alpha * self.filtered_rate + (1.0 - alpha) * raw_rate
        self.previous_los = los.copy()
        self.filtered_rate = filtered.copy()
        return filtered


class LOSGuidanceController:
    """Position + LOS-rate guidance without target velocity or future-path input."""

    def __init__(self, config: LOSGuidanceConfig | None = None) -> None:
        self.config = LOSGuidanceConfig() if config is None else config
        self.rate_estimator = LOSRateEstimator(self.config.los_rate_filter_alpha)

    def reset(self) -> None:
        self.rate_estimator.reset()

    def command(
        self,
        relative_position: np.ndarray,
        own_velocity: np.ndarray,
        limits: AircraftLimits,
        dt: float,
        measured_los_rate: np.ndarray | None = None,
    ) -> GuidanceCommand:
        relative_position = np.asarray(relative_position, dtype=np.float64)
        own_velocity = np.asarray(own_velocity, dtype=np.float64)
        distance = float(np.linalg.norm(relative_position))
        if distance < 1e-12:
            zeros = np.zeros(3, dtype=np.float64)
            return GuidanceCommand(zeros, zeros, zeros, zeros, zeros, distance)

        los = relative_position / distance
        if measured_los_rate is None:
            los_rate = self.rate_estimator.update(los, dt)
        else:
            los_rate = np.asarray(measured_los_rate, dtype=np.float64)
            self.rate_estimator.previous_los = los.copy()

        los_direction_rate = np.cross(los_rate, los)
        own_speed = float(np.linalg.norm(own_velocity))
        effective_lead_time = min(
            self.config.lead_time,
            distance / max(own_speed, 1.0),
        )
        aim_direction = los + effective_lead_time * los_direction_rate
        aim_norm = float(np.linalg.norm(aim_direction))
        aim_direction = los if aim_norm < 1e-12 else aim_direction / aim_norm

        desired_velocity = velocity_along_direction(
            aim_direction,
            limits,
            self.config.desired_speed_ratio,
        )
        pursuit_acceleration = (
            self.config.pursuit_gain * (desired_velocity - own_velocity) / float(dt)
        )

        if own_speed < 1e-12:
            proportional_navigation_acceleration = np.zeros(3, dtype=np.float64)
        else:
            velocity_direction = own_velocity / own_speed
            proportional_navigation_acceleration = (
                self.config.navigation_constant
                * own_speed
                * np.cross(los_rate, velocity_direction)
            )

        acceleration = limit_acceleration(
            pursuit_acceleration + proportional_navigation_acceleration,
            limits,
        )
        scale = np.array(
            [
                limits.max_horizontal_acceleration,
                limits.max_horizontal_acceleration,
                limits.max_vertical_acceleration,
            ],
            dtype=np.float64,
        )
        normalized_action = np.clip(acceleration / scale, -1.0, 1.0)
        return GuidanceCommand(
            acceleration=acceleration,
            normalized_action=normalized_action,
            line_of_sight=los,
            line_of_sight_rate=los_rate,
            aim_direction=aim_direction,
            distance=distance,
        )
