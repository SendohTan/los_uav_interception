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
    navigation_constant: float = 8.0
    pursuit_gain: float = 0.06
    lead_time: float = 3.0
    desired_speed_ratio: float = 0.98
    los_rate_filter_alpha: float = 0.75
    los_rate_window_size: int = 11
    los_rate_soft_threshold_rad_s: float = 0.01
    desired_velocity_filter_tau_s: float = 0.30
    desired_velocity_slew_limit_mps2: float = 4.0
    acceleration_filter_tau_s: float = 0.20

    def __post_init__(self) -> None:
        if self.navigation_constant < 0.0:
            raise ValueError("navigation_constant must be non-negative")
        if self.pursuit_gain < 0.0:
            raise ValueError("pursuit_gain must be non-negative")
        if self.lead_time < 0.0:
            raise ValueError("lead_time must be non-negative")
        if not 0.0 < self.desired_speed_ratio <= 1.0:
            raise ValueError("desired_speed_ratio must be in (0, 1]")
        if not 0.0 <= self.los_rate_filter_alpha < 1.0:
            raise ValueError("los_rate_filter_alpha must be in [0, 1)")
        if self.los_rate_window_size < 2:
            raise ValueError("los_rate_window_size must be at least 2")
        if self.los_rate_soft_threshold_rad_s < 0.0:
            raise ValueError("los_rate_soft_threshold_rad_s must be non-negative")
        if self.desired_velocity_filter_tau_s < 0.0:
            raise ValueError("desired_velocity_filter_tau_s must be non-negative")
        if self.desired_velocity_slew_limit_mps2 < 0.0:
            raise ValueError("desired_velocity_slew_limit_mps2 must be non-negative")
        if self.acceleration_filter_tau_s < 0.0:
            raise ValueError("acceleration_filter_tau_s must be non-negative")

    @classmethod
    def legacy(cls, **overrides: float) -> "LOSGuidanceConfig":
        values = {
            "navigation_constant": 4.0,
            "pursuit_gain": 0.05,
            "lead_time": 3.0,
            "desired_speed_ratio": 0.98,
            "los_rate_filter_alpha": 0.85,
            "los_rate_window_size": 2,
            "los_rate_soft_threshold_rad_s": 0.0,
            "desired_velocity_filter_tau_s": 0.0,
            "desired_velocity_slew_limit_mps2": 0.0,
            "acceleration_filter_tau_s": 0.0,
        }
        values.update(overrides)
        return cls(**values)

    @classmethod
    def conservative(cls, **overrides: float) -> "LOSGuidanceConfig":
        values = {
            "navigation_constant": 8.0,
            "pursuit_gain": 0.06,
            "lead_time": 3.0,
            "desired_speed_ratio": 0.98,
            "los_rate_filter_alpha": 0.85,
            "los_rate_window_size": 11,
            "los_rate_soft_threshold_rad_s": 0.015,
            "desired_velocity_filter_tau_s": 0.30,
            "desired_velocity_slew_limit_mps2": 4.0,
            "acceleration_filter_tau_s": 0.30,
        }
        values.update(overrides)
        return cls(**values)

    @classmethod
    def flight_test(cls, **overrides: float) -> "LOSGuidanceConfig":
        values = {
            "navigation_constant": 10.0,
            "pursuit_gain": 0.07,
            "lead_time": 3.0,
            "desired_speed_ratio": 0.98,
            "los_rate_filter_alpha": 0.90,
            "los_rate_window_size": 13,
            "los_rate_soft_threshold_rad_s": 0.020,
            "desired_velocity_filter_tau_s": 0.40,
            "desired_velocity_slew_limit_mps2": 2.0,
            "acceleration_filter_tau_s": 0.40,
        }
        values.update(overrides)
        return cls(**values)


@dataclass(frozen=True)
class GuidanceCommand:
    acceleration: np.ndarray
    normalized_action: np.ndarray
    line_of_sight: np.ndarray
    line_of_sight_rate: np.ndarray
    aim_direction: np.ndarray
    raw_desired_velocity: np.ndarray
    desired_velocity: np.ndarray
    pursuit_acceleration: np.ndarray
    proportional_navigation_acceleration: np.ndarray
    distance: float


class LOSRateEstimator:
    """Estimate the LOS angular-rate vector from consecutive bearing samples."""

    def __init__(self, filter_alpha: float = 0.65, window_size: int = 7) -> None:
        if not 0.0 <= filter_alpha < 1.0:
            raise ValueError("filter_alpha must be in [0, 1)")
        if window_size < 2:
            raise ValueError("window_size must be at least 2")
        self.filter_alpha = float(filter_alpha)
        self.window_size = int(window_size)
        self.previous_los: np.ndarray | None = None
        self.filtered_los: np.ndarray | None = None
        self.filtered_rate: np.ndarray | None = None
        self.los_history: list[np.ndarray] = []
        self.time_history: list[float] = []
        self.elapsed_time = 0.0

    def reset(self) -> None:
        self.previous_los = None
        self.filtered_los = None
        self.filtered_rate = None
        self.los_history.clear()
        self.time_history.clear()
        self.elapsed_time = 0.0

    def update(self, line_of_sight: np.ndarray, dt: float) -> np.ndarray:
        los = np.asarray(line_of_sight, dtype=np.float64)
        norm = float(np.linalg.norm(los))
        if norm < 1e-12:
            return np.zeros(3, dtype=np.float64)
        los /= norm

        if self.previous_los is not None:
            self.elapsed_time += float(dt)
        self.los_history.append(los.copy())
        self.time_history.append(self.elapsed_time)
        if len(self.los_history) > self.window_size:
            self.los_history.pop(0)
            self.time_history.pop(0)

        if len(self.los_history) < 2:
            fitted_los = los.copy()
            raw_rate = np.zeros(3, dtype=np.float64)
        else:
            times = np.asarray(self.time_history, dtype=np.float64)
            centered_times = times - float(np.mean(times))
            denominator = float(np.dot(centered_times, centered_times))
            if denominator < 1e-12:
                fitted_los = los.copy()
                raw_rate = np.zeros(3, dtype=np.float64)
            else:
                history = np.asarray(self.los_history, dtype=np.float64)
                mean_los = np.mean(history, axis=0)
                los_direction_rate = np.sum(
                    centered_times[:, None] * history,
                    axis=0,
                ) / denominator
                fitted_los = mean_los + los_direction_rate * centered_times[-1]
                fitted_norm = float(np.linalg.norm(fitted_los))
                fitted_los = (
                    los.copy()
                    if fitted_norm < 1e-12
                    else fitted_los / fitted_norm
                )
                raw_rate = np.cross(fitted_los, los_direction_rate)

        if self.filtered_rate is None:
            filtered = raw_rate
        else:
            alpha = self.filter_alpha
            filtered = alpha * self.filtered_rate + (1.0 - alpha) * raw_rate
        self.previous_los = los.copy()
        self.filtered_los = fitted_los.copy()
        self.filtered_rate = filtered.copy()
        return filtered


class LOSGuidanceController:
    """Position + LOS-rate guidance without target velocity or future-path input."""

    def __init__(self, config: LOSGuidanceConfig | None = None) -> None:
        self.config = LOSGuidanceConfig() if config is None else config
        self.rate_estimator = LOSRateEstimator(
            self.config.los_rate_filter_alpha,
            self.config.los_rate_window_size,
        )
        self.filtered_desired_velocity: np.ndarray | None = None
        self.filtered_acceleration = np.zeros(3, dtype=np.float64)

    def reset(self) -> None:
        self.rate_estimator.reset()
        self.filtered_desired_velocity = None
        self.filtered_acceleration.fill(0.0)

    def command(
        self,
        relative_position: np.ndarray,
        own_velocity: np.ndarray,
        limits: AircraftLimits,
        dt: float,
        measured_los_rate: np.ndarray | None = None,
    ) -> GuidanceCommand:
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt must be finite and positive")
        relative_position = np.asarray(relative_position, dtype=np.float64)
        own_velocity = np.asarray(own_velocity, dtype=np.float64)
        if relative_position.shape != (3,) or own_velocity.shape != (3,):
            raise ValueError("relative_position and own_velocity must have shape (3,)")
        if not np.all(np.isfinite(relative_position)) or not np.all(
            np.isfinite(own_velocity)
        ):
            raise ValueError("guidance inputs must contain only finite values")
        distance = float(np.linalg.norm(relative_position))
        if distance < 1e-12:
            zeros = np.zeros(3, dtype=np.float64)
            return GuidanceCommand(
                acceleration=zeros,
                normalized_action=zeros,
                line_of_sight=zeros,
                line_of_sight_rate=zeros,
                aim_direction=zeros,
                raw_desired_velocity=zeros,
                desired_velocity=zeros,
                pursuit_acceleration=zeros,
                proportional_navigation_acceleration=zeros,
                distance=distance,
            )

        measured_los = relative_position / distance
        if measured_los_rate is None:
            los_rate = self.rate_estimator.update(measured_los, dt)
        else:
            los_rate = np.asarray(measured_los_rate, dtype=np.float64)
            if los_rate.shape != (3,) or not np.all(np.isfinite(los_rate)):
                raise ValueError("measured_los_rate must be a finite shape-(3,) vector")
            self.rate_estimator.update(measured_los, dt)
        los = (
            measured_los
            if self.rate_estimator.filtered_los is None
            else self.rate_estimator.filtered_los.copy()
        )

        threshold = self.config.los_rate_soft_threshold_rad_s
        los_rate_norm = float(np.linalg.norm(los_rate))
        if threshold > 0.0:
            if los_rate_norm <= threshold:
                los_rate = np.zeros(3, dtype=np.float64)
            else:
                los_rate = los_rate * (1.0 - threshold / los_rate_norm)

        los_direction_rate = np.cross(los_rate, los)
        own_speed = float(np.linalg.norm(own_velocity))
        effective_lead_time = min(
            self.config.lead_time,
            distance / max(own_speed, 1.0),
        )
        aim_direction = los + effective_lead_time * los_direction_rate
        aim_norm = float(np.linalg.norm(aim_direction))
        aim_direction = los if aim_norm < 1e-12 else aim_direction / aim_norm

        raw_desired_velocity = velocity_along_direction(
            aim_direction,
            limits,
            self.config.desired_speed_ratio,
        )
        if self.filtered_desired_velocity is None:
            self.filtered_desired_velocity = own_velocity.copy()
        desired_velocity = raw_desired_velocity.copy()
        velocity_filter_tau = self.config.desired_velocity_filter_tau_s
        if velocity_filter_tau > 0.0:
            alpha = float(np.exp(-dt / velocity_filter_tau))
            desired_velocity = (
                alpha * self.filtered_desired_velocity
                + (1.0 - alpha) * desired_velocity
            )
        velocity_slew_limit = self.config.desired_velocity_slew_limit_mps2
        if velocity_slew_limit > 0.0:
            velocity_delta = desired_velocity - self.filtered_desired_velocity
            maximum_delta = velocity_slew_limit * dt
            horizontal_delta_norm = float(np.linalg.norm(velocity_delta[:2]))
            if horizontal_delta_norm > maximum_delta:
                velocity_delta[:2] *= maximum_delta / horizontal_delta_norm
            velocity_delta[2] = np.clip(
                velocity_delta[2],
                -maximum_delta,
                maximum_delta,
            )
            desired_velocity = self.filtered_desired_velocity + velocity_delta
        self.filtered_desired_velocity = desired_velocity.copy()
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

        raw_acceleration = (
            pursuit_acceleration + proportional_navigation_acceleration
        )
        acceleration_filter_tau = self.config.acceleration_filter_tau_s
        if acceleration_filter_tau > 0.0:
            alpha = float(np.exp(-dt / acceleration_filter_tau))
            raw_acceleration = (
                alpha * self.filtered_acceleration
                + (1.0 - alpha) * raw_acceleration
            )
        self.filtered_acceleration = raw_acceleration.copy()
        acceleration = limit_acceleration(raw_acceleration, limits)
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
            raw_desired_velocity=raw_desired_velocity,
            desired_velocity=desired_velocity,
            pursuit_acceleration=pursuit_acceleration,
            proportional_navigation_acceleration=proportional_navigation_acceleration,
            distance=distance,
        )
