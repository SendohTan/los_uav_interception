from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from .dynamics import AIRCRAFT_LIMITS, PointMassUAV, velocity_along_direction
from .guidance import LOSGuidanceConfig, LOSGuidanceController
from .integration import GuidanceSafetyGate
from .metrics import hit_probability_components
from .sensors import BearingRangeNoise, CameraFOV, RelativePositionSensor


EVALUATION_MOTION_MODES = (
    "line",
    "cosine",
    "arc",
    "random",
    "multi_sine",
    "bspline",
)
MOTION_MODES = (*EVALUATION_MOTION_MODES, "jink")


@dataclass(frozen=True)
class SimulationConfig:
    dt: float = 0.1
    max_time: float = 100.0
    success_radius: float = 0.6
    escape_radius: float = 10.0
    interceptor_type: str = "A"
    target_type: str = "C"
    target_motion: str = "line"
    camera_constant_pixel_m: float = 1200.0
    pixel_error_max: int = 1
    angle_noise_std_deg: float = 0.5
    fov_enabled: bool = True
    fov_horizontal_deg: float = 24.0
    fov_vertical_deg: float = 16.0
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
    target_visible: np.ndarray
    range_error_fraction: np.ndarray


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
    def __init__(
        self,
        motion: str,
        dt: float,
        initial_position: np.ndarray,
        seed: int,
    ) -> None:
        if motion not in MOTION_MODES:
            raise ValueError(f"motion must be one of {MOTION_MODES}")
        self.motion = motion
        self.dt = float(dt)
        self.initial_position = np.asarray(initial_position, dtype=np.float64).copy()
        self.initial_range = max(float(np.linalg.norm(self.initial_position)), 1e-12)
        radial = self.initial_position / self.initial_range
        lateral = np.array([-radial[1], radial[0], 0.0], dtype=np.float64)
        if float(np.linalg.norm(lateral)) < 1e-12:
            lateral = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        self.lateral = lateral / float(np.linalg.norm(lateral))
        self.vertical = np.cross(radial, self.lateral)
        self.vertical /= float(np.linalg.norm(self.vertical)) + 1e-12
        self.rng = np.random.default_rng(seed)
        self.lookahead = 0.12
        self.curve_strength = 0.45
        self.multi_sine_frequencies = self.rng.uniform(
            [0.65, 1.15, 1.75],
            [0.95, 1.55, 2.25],
        )
        self.multi_sine_lateral_weights = self.rng.uniform(0.45, 1.0, 3)
        self.multi_sine_vertical_weights = self.rng.uniform(0.25, 0.75, 3)
        self.multi_sine_lateral_phases = self.rng.uniform(-np.pi, np.pi, 3)
        self.multi_sine_vertical_phases = self.rng.uniform(-np.pi, np.pi, 3)
        self.random_lateral, self.random_vertical = self._random_control_points(
            int(self.rng.integers(5, 8)),
            0.12,
            0.04,
        )
        self.bspline_lateral, self.bspline_vertical = self._random_control_points(
            int(self.rng.integers(7, 10)),
            0.14,
            0.06,
        )
        self.jink_started = False
        self.jink_sign = 1.0

    def _random_control_points(
        self,
        count: int,
        lateral_ratio: float,
        vertical_ratio: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        progress = np.linspace(0.0, 1.0, count)
        envelope = np.clip(np.sin(np.pi * progress), 0.0, None) ** 1.5
        lateral = self.rng.uniform(-1.0, 1.0, count) * envelope
        vertical = self.rng.uniform(-1.0, 1.0, count) * envelope
        lateral[0] = 0.0
        vertical[0] = 0.0
        lateral[-2:] = 0.0
        vertical[-2:] = 0.0
        lateral *= self.curve_strength * self.initial_range * lateral_ratio
        vertical *= self.curve_strength * self.initial_range * vertical_ratio
        return lateral, vertical

    @staticmethod
    def _bspline_value(control_points: np.ndarray, progress: float) -> float:
        points = np.asarray(control_points, dtype=np.float64)
        degree = min(3, len(points) - 1)
        last_index = len(points) - 1
        interior_count = last_index - degree
        knots = np.concatenate(
            (
                np.zeros(degree + 1),
                np.arange(1, interior_count + 1) / (interior_count + 1),
                np.ones(degree + 1),
            )
        )
        value = float(np.clip(progress, 0.0, 1.0))
        span = (
            last_index
            if value >= 1.0
            else int(np.searchsorted(knots, value, side="right") - 1)
        )
        span = int(np.clip(span, degree, last_index))
        work = points[span - degree : span + 1].copy()
        for level in range(1, degree + 1):
            for index in range(degree, level - 1, -1):
                knot_index = span - degree + index
                denominator = (
                    knots[knot_index + degree - level + 1] - knots[knot_index]
                )
                alpha = 0.0 if denominator <= 1e-12 else (
                    value - knots[knot_index]
                ) / denominator
                work[index] = (
                    (1.0 - alpha) * work[index - 1] + alpha * work[index]
                )
        return float(work[degree])

    def _curve_offset(self, progress: float) -> tuple[float, float]:
        progress = float(np.clip(progress, 0.0, 1.0))
        envelope = np.sin(np.pi * progress) ** 2
        if self.motion == "arc":
            lateral = 0.10 * self.curve_strength * self.initial_range * envelope
            vertical = (
                0.02
                * self.curve_strength
                * self.initial_range
                * envelope
                * np.sin(2.0 * np.pi * progress)
            )
            return float(lateral), float(vertical)
        if self.motion == "cosine":
            return (
                float(
                    0.35
                    * self.curve_strength
                    * self.initial_range
                    * envelope
                    * np.sin(2.0 * np.pi * progress)
                ),
                float(
                    0.10
                    * self.curve_strength
                    * self.initial_range
                    * envelope
                    * np.sin(3.0 * np.pi * progress)
                ),
            )
        if self.motion == "random":
            return (
                self._bspline_value(self.random_lateral, progress),
                self._bspline_value(self.random_vertical, progress),
            )
        if self.motion == "multi_sine":
            lateral_wave = np.sum(
                self.multi_sine_lateral_weights
                * np.sin(
                    2.0 * np.pi * self.multi_sine_frequencies * progress
                    + self.multi_sine_lateral_phases
                )
            ) / np.sum(np.abs(self.multi_sine_lateral_weights))
            vertical_wave = np.sum(
                self.multi_sine_vertical_weights
                * np.sin(
                    2.0 * np.pi * self.multi_sine_frequencies * progress
                    + self.multi_sine_vertical_phases
                )
            ) / np.sum(np.abs(self.multi_sine_vertical_weights))
            return (
                float(
                    0.18
                    * self.curve_strength
                    * self.initial_range
                    * envelope
                    * lateral_wave
                ),
                float(
                    0.06
                    * self.curve_strength
                    * self.initial_range
                    * envelope
                    * vertical_wave
                ),
            )
        if self.motion == "bspline":
            terminal_envelope = np.clip((1.0 - progress) / 0.30, 0.0, 1.0) ** 2
            return (
                self._bspline_value(self.bspline_lateral, progress)
                * terminal_envelope,
                self._bspline_value(self.bspline_vertical, progress)
                * terminal_envelope,
            )
        return 0.0, 0.0

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
        desired_direction = goal_direction
        if self.motion in EVALUATION_MOTION_MODES and self.motion != "line":
            travelled = float(
                np.dot(
                    target.position - self.initial_position,
                    -self.initial_position / self.initial_range,
                )
            )
            progress = max(travelled / self.initial_range, 0.0)
            lookahead_progress = min(progress + self.lookahead, 1.0)
            path_center = self.initial_position * (1.0 - lookahead_progress)
            lateral_offset, vertical_offset = self._curve_offset(lookahead_progress)
            aim_point = (
                path_center
                + lateral_offset * self.lateral
                + vertical_offset * self.vertical
            )
            path_direction = aim_point - target.position
            if float(np.linalg.norm(path_direction)) > 1e-12:
                desired_direction = path_direction
        elif self.motion == "jink":
            lateral_gain = 0.18 * np.sin(0.45 * time_seconds)
            if 80.0 <= closest_interceptor_distance <= 150.0:
                self.jink_started = True
            if self.jink_started:
                phase = int(max(time_seconds, 0.0) / 1.5)
                self.jink_sign = -1.0 if phase % 2 else 1.0
                lateral_gain += 0.9 * self.jink_sign
            desired_direction = goal_direction + lateral_gain * self.lateral

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
                * self.lateral
            )
        return acceleration


def _make_sensor(config: SimulationConfig, seed: int) -> RelativePositionSensor:
    return RelativePositionSensor(
        BearingRangeNoise(
            camera_constant_pixel_m=config.camera_constant_pixel_m,
            pixel_error_max=config.pixel_error_max,
            angle_noise_std_deg=config.angle_noise_std_deg,
        ),
        seed=seed,
    )


def _make_fov(config: SimulationConfig) -> CameraFOV:
    return CameraFOV(
        horizontal_deg=config.fov_horizontal_deg,
        vertical_deg=config.fov_vertical_deg,
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
    safety_gate = GuidanceSafetyGate(controller, interceptor_limits)
    sensor = _make_sensor(config, seed)
    camera_fov = _make_fov(config)
    target_controller = GoalDirectedTargetController(
        config.target_motion,
        config.dt,
        target.position,
        seed,
    )

    interceptor_history = [interceptor.position.copy()]
    interceptor_velocity_history = [interceptor.velocity.copy()]
    desired_velocity_history = [interceptor.velocity.copy()]
    acceleration_history = [np.zeros(3, dtype=np.float64)]
    visibility_history = [True]
    range_error_history = [np.nan]
    target_history = [target.position.copy()]
    distance_history = [float(np.linalg.norm(target.position - interceptor.position))]
    minimum_distance = distance_history[0]
    success = False
    steps = int(np.ceil(config.max_time / config.dt))

    for step in range(steps):
        previous_interceptor = interceptor.position.copy()
        previous_target = target.position.copy()
        true_relative_position = target.position - interceptor.position
        target_visible = (
            not config.fov_enabled
            or camera_fov.contains(true_relative_position, interceptor.velocity)
        )
        observed_relative_position = (
            sensor.measure(true_relative_position)
            if target_visible
            else np.zeros(3, dtype=np.float64)
        )
        range_error_fraction = (
            sensor.last_range_error_fraction if target_visible else np.nan
        )
        command = safety_gate.command(
            current_time_s=step * config.dt,
            measurement_time_s=step * config.dt,
            target_visible=target_visible,
            relative_position_ned_m=observed_relative_position,
            own_velocity_ned_mps=interceptor.velocity,
        )
        command_acceleration = (
            np.zeros(3, dtype=np.float64)
            if command is None
            else command.acceleration
        )
        command_velocity = (
            interceptor.velocity.copy()
            if command is None
            else command.desired_velocity
        )
        interceptor.step(command_acceleration)
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
        desired_velocity_history.append(command_velocity.copy())
        acceleration_history.append(command_acceleration.copy())
        visibility_history.append(target_visible)
        range_error_history.append(range_error_fraction)
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
        target_visible=np.asarray(visibility_history, dtype=bool)[:, None],
        range_error_fraction=np.asarray(range_error_history, dtype=np.float64)[:, None],
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
    safety_gates: list[GuidanceSafetyGate] = []
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
        safety_gates.append(
            GuidanceSafetyGate(controllers[-1], interceptor_limits)
        )
        sensors.append(_make_sensor(config, seed + index))
    camera_fov = _make_fov(config)

    target_controller = GoalDirectedTargetController(
        config.target_motion,
        config.dt,
        target.position,
        seed,
    )
    interceptor_history = [np.vstack([item.position for item in interceptors])]
    interceptor_velocity_history = [
        np.vstack([item.velocity for item in interceptors])
    ]
    desired_velocity_history = [
        np.vstack([item.velocity for item in interceptors])
    ]
    acceleration_history = [np.zeros((3, 3), dtype=np.float64)]
    visibility_history = [np.ones(3, dtype=bool)]
    range_error_history = [np.full(3, np.nan, dtype=np.float64)]
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
        step_visibility = []
        step_range_errors = []
        for index, interceptor in enumerate(interceptors):
            true_relative_position = target.position - interceptor.position
            target_visible = (
                not config.fov_enabled
                or camera_fov.contains(true_relative_position, interceptor.velocity)
            )
            observed_relative_position = (
                sensors[index].measure(true_relative_position)
                if target_visible
                else np.zeros(3, dtype=np.float64)
            )
            range_error_fraction = (
                sensors[index].last_range_error_fraction if target_visible else np.nan
            )
            command = safety_gates[index].command(
                current_time_s=step * config.dt,
                measurement_time_s=step * config.dt,
                target_visible=target_visible,
                relative_position_ned_m=observed_relative_position,
                own_velocity_ned_mps=interceptor.velocity,
            )
            command_acceleration = (
                np.zeros(3, dtype=np.float64)
                if command is None
                else command.acceleration
            )
            command_velocity = (
                interceptor.velocity.copy()
                if command is None
                else command.desired_velocity
            )
            interceptor.step(command_acceleration)
            step_desired_velocities.append(command_velocity.copy())
            step_accelerations.append(command_acceleration.copy())
            step_visibility.append(target_visible)
            step_range_errors.append(range_error_fraction)

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
        visibility_history.append(np.asarray(step_visibility, dtype=bool))
        range_error_history.append(np.asarray(step_range_errors, dtype=np.float64))
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
        target_visible=np.asarray(visibility_history, dtype=bool),
        range_error_fraction=np.asarray(range_error_history, dtype=np.float64),
    )
