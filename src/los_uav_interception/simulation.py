from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from .dynamics import AIRCRAFT_LIMITS, PointMassUAV, velocity_along_direction
from .guidance import LOSGuidanceConfig, LOSGuidanceController
from .metrics import hit_probability_components
from .sensors import BearingRangeNoise, RelativePositionSensor


MOTION_MODES = ("line", "arc", "multi_sine", "jink")


@dataclass(frozen=True)
class SimulationConfig:
    dt: float = 0.1
    max_time: float = 100.0
    success_radius: float = 0.6
    escape_radius: float = 10.0
    interceptor_type: str = "A"
    target_type: str = "C"
    target_motion: str = "line"
    range_bias_fraction: float = 0.0
    range_jitter_std: float = 0.0
    angle_noise_std_deg: float = 0.0
    guidance_config: LOSGuidanceConfig | None = None


@dataclass(frozen=True)
class SimulationResult:
    success: bool
    elapsed_time: float
    minimum_distance: float
    interceptor_positions: np.ndarray
    target_positions: np.ndarray
    distances: np.ndarray
    hit_metrics: dict[str, float | bool]
    interceptor_velocities: np.ndarray
    desired_velocities: np.ndarray
    guidance_accelerations: np.ndarray


def _segment_minimum_distance(
    interceptor_previous: np.ndarray,
    interceptor_current: np.ndarray,
    target_previous: np.ndarray,
    target_current: np.ndarray,
) -> float:
    relative_previous = target_previous - interceptor_previous
    relative_delta = (
        target_current - target_previous
    ) - (interceptor_current - interceptor_previous)
    denominator = float(np.dot(relative_delta, relative_delta))
    if denominator < 1e-12:
        return float(np.linalg.norm(relative_previous))
    fraction = float(
        np.clip(
            -np.dot(relative_previous, relative_delta) / denominator,
            0.0,
            1.0,
        )
    )
    return float(np.linalg.norm(relative_previous + fraction * relative_delta))


class GoalDirectedTargetController:
    def __init__(self, motion: str, dt: float) -> None:
        if motion not in MOTION_MODES:
            raise ValueError(f"motion must be one of {MOTION_MODES}")
        self.motion = motion
        self.dt = float(dt)
        self.jink_started = False
        self.jink_sign = 1.0

    def command(
        self,
        target: PointMassUAV,
        time_seconds: float,
        closest_interceptor_distance: float,
    ) -> np.ndarray:
        to_goal = -target.position
        goal_norm = float(np.linalg.norm(to_goal))
        if goal_norm < 1e-12:
            return -target.velocity / self.dt
        goal_direction = to_goal / goal_norm
        lateral = np.array([-goal_direction[1], goal_direction[0], 0.0])
        lateral_norm = float(np.linalg.norm(lateral))
        if lateral_norm < 1e-12:
            lateral = np.array([0.0, 1.0, 0.0])
        else:
            lateral /= lateral_norm
        vertical = np.cross(goal_direction, lateral)
        vertical /= float(np.linalg.norm(vertical)) + 1e-12

        lateral_gain = 0.0
        vertical_gain = 0.0
        if self.motion == "arc":
            lateral_gain = 0.35
        elif self.motion == "multi_sine":
            lateral_gain = (
                0.34 * np.sin(0.42 * time_seconds)
                + 0.18 * np.sin(0.83 * time_seconds + 0.7)
                + 0.10 * np.sin(1.37 * time_seconds - 0.4)
            )
            vertical_gain = (
                0.11 * np.sin(0.51 * time_seconds + 0.3)
                + 0.06 * np.sin(1.11 * time_seconds)
            )
        elif self.motion == "jink":
            lateral_gain = 0.18 * np.sin(0.45 * time_seconds)
            if 80.0 <= closest_interceptor_distance <= 150.0:
                self.jink_started = True
            if self.jink_started:
                phase = int(max(time_seconds, 0.0) / 1.5)
                self.jink_sign = -1.0 if phase % 2 else 1.0
                lateral_gain += 0.9 * self.jink_sign

        desired_direction = (
            goal_direction + lateral_gain * lateral + vertical_gain * vertical
        )
        desired_velocity = velocity_along_direction(
            desired_direction,
            target.limits,
            ratio=0.82,
        )
        acceleration = (desired_velocity - target.velocity) / self.dt
        if self.motion == "jink" and self.jink_started:
            acceleration += (
                self.jink_sign
                * target.limits.max_horizontal_acceleration
                * lateral
            )
        return acceleration


def _make_sensor(config: SimulationConfig, seed: int) -> RelativePositionSensor:
    return RelativePositionSensor(
        BearingRangeNoise(
            range_bias_fraction=config.range_bias_fraction,
            range_jitter_std=config.range_jitter_std,
            angle_noise_std_deg=config.angle_noise_std_deg,
        ),
        seed=seed,
    )


def _initial_target(config: SimulationConfig, seed: int) -> PointMassUAV:
    rng = np.random.default_rng(seed)
    distance = float(rng.uniform(300.0, 500.0))
    azimuth = float(rng.uniform(-np.pi, np.pi))
    elevation = float(rng.uniform(-np.pi / 6.0, np.pi / 6.0))
    target_position = np.array(
        [
            distance * np.cos(elevation) * np.cos(azimuth),
            distance * np.cos(elevation) * np.sin(azimuth),
            -distance * np.sin(elevation),
        ],
        dtype=np.float64,
    )
    target_limits = AIRCRAFT_LIMITS[config.target_type]
    target_velocity = velocity_along_direction(
        -target_position,
        target_limits,
        ratio=0.82,
    )
    return PointMassUAV(
        target_limits,
        dt=config.dt,
        position=target_position,
        velocity=target_velocity,
    )


def run_single(config: SimulationConfig, seed: int = 1) -> SimulationResult:
    target = _initial_target(config, seed)
    interceptor_limits = AIRCRAFT_LIMITS[config.interceptor_type]
    interceptor_velocity = velocity_along_direction(
        target.position,
        interceptor_limits,
        ratio=0.75,
    )
    interceptor = PointMassUAV(
        interceptor_limits,
        dt=config.dt,
        velocity=interceptor_velocity,
    )
    controller = LOSGuidanceController(config.guidance_config)
    sensor = _make_sensor(config, seed)
    target_controller = GoalDirectedTargetController(config.target_motion, config.dt)

    interceptor_history = [interceptor.position.copy()]
    interceptor_velocity_history = [interceptor.velocity.copy()]
    desired_velocity_history = [interceptor.velocity.copy()]
    acceleration_history = [np.zeros(3, dtype=np.float64)]
    target_history = [target.position.copy()]
    distance_history = [float(np.linalg.norm(target.position - interceptor.position))]
    minimum_distance = distance_history[0]
    success = False
    steps = int(np.ceil(config.max_time / config.dt))

    for step in range(steps):
        previous_interceptor = interceptor.position.copy()
        previous_target = target.position.copy()
        observed_relative_position = sensor.measure(target.position - interceptor.position)
        command = controller.command(
            observed_relative_position,
            interceptor.velocity,
            interceptor.limits,
            config.dt,
        )
        interceptor.step(command.acceleration)
        target_acceleration = target_controller.command(
            target,
            step * config.dt,
            distance_history[-1],
        )
        target.step(target_acceleration)

        segment_distance = _segment_minimum_distance(
            previous_interceptor,
            interceptor.position,
            previous_target,
            target.position,
        )
        distance = float(np.linalg.norm(target.position - interceptor.position))
        minimum_distance = min(minimum_distance, segment_distance, distance)
        interceptor_history.append(interceptor.position.copy())
        interceptor_velocity_history.append(interceptor.velocity.copy())
        desired_velocity_history.append(command.desired_velocity.copy())
        acceleration_history.append(command.acceleration.copy())
        target_history.append(target.position.copy())
        distance_history.append(distance)
        if minimum_distance < config.success_radius:
            success = True
            break
        if float(np.linalg.norm(target.position)) < config.escape_radius:
            break

    metrics = hit_probability_components(
        interceptor.position,
        interceptor.velocity,
        target.position,
        target.velocity,
    )
    return SimulationResult(
        success=success,
        elapsed_time=(len(distance_history) - 1) * config.dt,
        minimum_distance=minimum_distance,
        interceptor_positions=np.asarray(interceptor_history)[:, None, :],
        target_positions=np.asarray(target_history),
        distances=np.asarray(distance_history)[:, None],
        hit_metrics=metrics,
        interceptor_velocities=np.asarray(interceptor_velocity_history)[:, None, :],
        desired_velocities=np.asarray(desired_velocity_history)[:, None, :],
        guidance_accelerations=np.asarray(acceleration_history)[:, None, :],
    )


def run_multi(config: SimulationConfig, seed: int = 1) -> SimulationResult:
    target = _initial_target(config, seed)
    interceptor_limits = AIRCRAFT_LIMITS[config.interceptor_type]
    offsets = (
        np.array([0.0, 0.0, 0.0]),
        np.array([0.0, 2.0, 0.0]),
        np.array([0.0, -2.0, 2.5]),
    )
    speed_ratios = (0.75, 0.85, 0.95)
    base_guidance_config = (
        LOSGuidanceConfig()
        if config.guidance_config is None
        else config.guidance_config
    )
    navigation_constants = tuple(
        base_guidance_config.navigation_constant * factor
        for factor in (0.9, 1.0, 1.1)
    )
    interceptors: list[PointMassUAV] = []
    controllers: list[LOSGuidanceController] = []
    sensors: list[RelativePositionSensor] = []
    for index in range(3):
        direction = target.position - offsets[index]
        velocity = velocity_along_direction(
            direction,
            interceptor_limits,
            speed_ratios[index],
        )
        interceptors.append(
            PointMassUAV(
                interceptor_limits,
                dt=config.dt,
                position=offsets[index],
                velocity=velocity,
            )
        )
        controllers.append(
            LOSGuidanceController(
                replace(
                    base_guidance_config,
                    navigation_constant=navigation_constants[index],
                )
            )
        )
        sensors.append(_make_sensor(config, seed + index))

    target_controller = GoalDirectedTargetController(config.target_motion, config.dt)
    interceptor_history = [np.vstack([item.position for item in interceptors])]
    interceptor_velocity_history = [
        np.vstack([item.velocity for item in interceptors])
    ]
    desired_velocity_history = [
        np.vstack([item.velocity for item in interceptors])
    ]
    acceleration_history = [np.zeros((3, 3), dtype=np.float64)]
    target_history = [target.position.copy()]
    initial_distances = np.asarray(
        [np.linalg.norm(target.position - item.position) for item in interceptors]
    )
    distance_history = [initial_distances]
    minimum_distance = float(np.min(initial_distances))
    success = False
    steps = int(np.ceil(config.max_time / config.dt))

    for step in range(steps):
        previous_interceptors = [item.position.copy() for item in interceptors]
        previous_target = target.position.copy()
        step_desired_velocities = []
        step_accelerations = []
        for index, interceptor in enumerate(interceptors):
            observed_relative_position = sensors[index].measure(
                target.position - interceptor.position
            )
            command = controllers[index].command(
                observed_relative_position,
                interceptor.velocity,
                interceptor.limits,
                config.dt,
            )
            interceptor.step(command.acceleration)
            step_desired_velocities.append(command.desired_velocity.copy())
            step_accelerations.append(command.acceleration.copy())

        previous_distances = distance_history[-1]
        target.step(
            target_controller.command(
                target,
                step * config.dt,
                float(np.min(previous_distances)),
            )
        )
        distances = np.asarray(
            [np.linalg.norm(target.position - item.position) for item in interceptors]
        )
        segment_distances = [
            _segment_minimum_distance(
                previous_interceptors[index],
                interceptors[index].position,
                previous_target,
                target.position,
            )
            for index in range(3)
        ]
        minimum_distance = min(
            minimum_distance,
            float(np.min(distances)),
            float(np.min(segment_distances)),
        )
        interceptor_history.append(np.vstack([item.position for item in interceptors]))
        interceptor_velocity_history.append(
            np.vstack([item.velocity for item in interceptors])
        )
        desired_velocity_history.append(np.vstack(step_desired_velocities))
        acceleration_history.append(np.vstack(step_accelerations))
        target_history.append(target.position.copy())
        distance_history.append(distances)
        if minimum_distance < config.success_radius:
            success = True
            break
        if float(np.linalg.norm(target.position)) < config.escape_radius:
            break

    closest_index = int(np.argmin(distance_history[-1]))
    closest = interceptors[closest_index]
    metrics = hit_probability_components(
        closest.position,
        closest.velocity,
        target.position,
        target.velocity,
    )
    return SimulationResult(
        success=success,
        elapsed_time=(len(distance_history) - 1) * config.dt,
        minimum_distance=minimum_distance,
        interceptor_positions=np.asarray(interceptor_history),
        target_positions=np.asarray(target_history),
        distances=np.asarray(distance_history),
        hit_metrics=metrics,
        interceptor_velocities=np.asarray(interceptor_velocity_history),
        desired_velocities=np.asarray(desired_velocity_history),
        guidance_accelerations=np.asarray(acceleration_history),
    )
