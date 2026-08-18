from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .dynamics import AircraftLimits
from .guidance import GuidanceCommand, LOSGuidanceController


@dataclass(frozen=True)
class GuidanceSafetyGateConfig:
    minimum_dt_s: float = 0.02
    maximum_dt_s: float = 0.20
    maximum_measurement_age_s: float = 0.20

    def __post_init__(self) -> None:
        if self.minimum_dt_s <= 0.0:
            raise ValueError("minimum_dt_s must be positive")
        if self.maximum_dt_s < self.minimum_dt_s:
            raise ValueError("maximum_dt_s must not be smaller than minimum_dt_s")
        if self.maximum_measurement_age_s <= 0.0:
            raise ValueError("maximum_measurement_age_s must be positive")


class GuidanceSafetyGate:
    """Reject invalid or stale target observations before LOS guidance runs.

    A ``None`` result means the flight controller must retain or enter its own
    safe fallback mode. This outer-loop helper never commands attitude or motors.
    """

    def __init__(
        self,
        controller: LOSGuidanceController,
        limits: AircraftLimits,
        config: GuidanceSafetyGateConfig | None = None,
    ) -> None:
        self.controller = controller
        self.limits = limits
        self.config = GuidanceSafetyGateConfig() if config is None else config
        self.previous_sample_time_s: float | None = None

    def reset(self) -> None:
        self.previous_sample_time_s = None
        self.controller.reset()

    def command(
        self,
        *,
        current_time_s: float,
        measurement_time_s: float,
        target_visible: bool,
        relative_position_ned_m: np.ndarray,
        own_velocity_ned_mps: np.ndarray,
        measured_los_rate_rad_s: np.ndarray | None = None,
    ) -> GuidanceCommand | None:
        timestamps = np.asarray([current_time_s, measurement_time_s], dtype=np.float64)
        if not np.all(np.isfinite(timestamps)):
            self.reset()
            return None
        measurement_age = current_time_s - measurement_time_s
        if (
            not target_visible
            or measurement_age < 0.0
            or measurement_age > self.config.maximum_measurement_age_s
        ):
            self.reset()
            return None
        relative_position = np.asarray(relative_position_ned_m, dtype=np.float64)
        own_velocity = np.asarray(own_velocity_ned_mps, dtype=np.float64)
        if (
            relative_position.shape != (3,)
            or own_velocity.shape != (3,)
            or not np.all(np.isfinite(relative_position))
            or not np.all(np.isfinite(own_velocity))
        ):
            self.reset()
            return None
        if self.previous_sample_time_s is None:
            self.previous_sample_time_s = measurement_time_s
            self.controller.reset()
            return None
        dt = measurement_time_s - self.previous_sample_time_s
        self.previous_sample_time_s = measurement_time_s
        if dt < self.config.minimum_dt_s or dt > self.config.maximum_dt_s:
            self.controller.reset()
            return None
        return self.controller.command(
            relative_position=relative_position,
            own_velocity=own_velocity,
            limits=self.limits,
            dt=dt,
            measured_los_rate=measured_los_rate_rad_s,
        )
