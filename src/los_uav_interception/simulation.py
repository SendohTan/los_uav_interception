from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np
from scipy.interpolate import make_interp_spline

from .dynamics import AIRCRAFT_LIMITS, PointMassUAV, velocity_along_direction
from .guidance import LOSGuidanceConfig, LOSGuidanceController
from .integration import GuidanceSafetyGate, GuidanceSafetyGateConfig
from .metrics import hit_probability_components
from .sensors import (
    BearingRangeNoise,
    BodyAttitudeCameraModel,
    CameraFOV,
    RelativePositionSensor,
)


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
    camera_attitude_response_tau_s: float = 0.25
    camera_gimbal_pitch_deg: float = 0.0
    camera_frame_rate_hz: float = 30.0
    sensor_latency_s: float = 1.0 / 30.0
    detection_dropout_probability: float = 0.01
    los_rate_source: str = "accurate_delayed"
    command_delay_steps: int = 1
    acceleration_response_tau_s: float = 0.2
    guidance_loss_behavior: str = "coast"
    guidance_config: LOSGuidanceConfig | None = None

    def __post_init__(self) -> None:
        if self.dt <= 0.0 or self.max_time <= 0.0:
            raise ValueError("simulation time parameters must be positive")
        if self.camera_frame_rate_hz <= 0.0:
            raise ValueError("camera_frame_rate_hz must be positive")
        if self.sensor_latency_s < 0.0:
            raise ValueError("sensor_latency_s must be non-negative")
        if self.camera_attitude_response_tau_s < 0.0:
            raise ValueError("camera attitude response time must be non-negative")
        if not 0.0 <= self.detection_dropout_probability <= 1.0:
            raise ValueError("detection dropout probability must be in [0, 1]")
        if self.los_rate_source not in ("accurate_delayed", "bearing_history"):
            raise ValueError("unsupported los_rate_source")
        if self.command_delay_steps < 0:
            raise ValueError("command_delay_steps must be non-negative")
        if self.acceleration_response_tau_s < 0.0:
            raise ValueError("acceleration_response_tau_s must be non-negative")
        if self.guidance_loss_behavior not in ("release", "coast"):
            raise ValueError("guidance_loss_behavior must be release or coast")


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
    command_actions: np.ndarray
    actual_accelerations: np.ndarray
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


def accurate_delayed_los_rate(
    relative_position: np.ndarray,
    relative_velocity: np.ndarray,
    latency_s: float,
) -> tuple[np.ndarray, np.ndarray]:
    relative_position = np.asarray(relative_position, dtype=np.float64)
    relative_velocity = np.asarray(relative_velocity, dtype=np.float64)
    delayed_position = relative_position - max(float(latency_s), 0.0) * relative_velocity
    range_squared = float(np.dot(delayed_position, delayed_position))
    if range_squared < 1e-12:
        return delayed_position, np.zeros(3, dtype=np.float64)
    los_rate = np.cross(delayed_position, relative_velocity) / range_squared
    return delayed_position, los_rate


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
        self.random_goal_direction = -radial
        self.random_path_knots = np.array([0.0, 1.0])
        self.random_lateral = np.zeros(2)
        self.random_vertical = np.zeros(2)
        self.random_lookahead = 0.1
        self.multi_sine_frequencies = np.array([0.8, 1.4, 2.1])
        self.multi_sine_lateral_weights = np.ones(3)
        self.multi_sine_vertical_weights = np.ones(3)
        self.multi_sine_lateral_phases = np.zeros(3)
        self.multi_sine_vertical_phases = np.zeros(3)
        self.bspline_lateral = None
        self.bspline_vertical = None
        if motion == "random":
            self._initialize_random_path()
        elif motion == "multi_sine":
            self._initialize_multi_sine_path()
        elif motion == "bspline":
            self._initialize_bspline_path()
        self.jink_started = False
        self.jink_sign = 1.0

    def _initialize_multi_sine_path(self) -> None:
        self.multi_sine_frequencies = np.array(
            [
                self.rng.uniform(0.65, 0.95),
                self.rng.uniform(1.15, 1.55),
                self.rng.uniform(1.75, 2.25),
            ]
        )
        self.multi_sine_lateral_weights = self.rng.uniform(0.45, 1.0, 3)
        self.multi_sine_vertical_weights = self.rng.uniform(0.25, 0.75, 3)
        self.multi_sine_lateral_phases = self.rng.uniform(-np.pi, np.pi, 3)
        self.multi_sine_vertical_phases = self.rng.uniform(-np.pi, np.pi, 3)

    def _initialize_bspline_path(self) -> None:
        knot_count = int(self.rng.integers(7, 10))
        knots = np.linspace(0.0, 1.0, knot_count)
        envelope = np.clip(np.sin(np.pi * knots), 0.0, None) ** 1.5
        lateral = self.rng.uniform(-1.0, 1.0, knot_count) * envelope
        vertical = self.rng.uniform(-1.0, 1.0, knot_count) * envelope
        lateral[0] = vertical[0] = 0.0
        lateral[-2:] = vertical[-2:] = 0.0
        lateral *= self.curve_strength * self.initial_range * 0.14
        vertical *= self.curve_strength * self.initial_range * 0.06
        self.bspline_lateral = make_interp_spline(knots, lateral, k=3)
        self.bspline_vertical = make_interp_spline(knots, vertical, k=3)

    def _initialize_random_path(self) -> None:
        knot_count = int(self.rng.integers(5, 8))
        self.random_path_knots = np.linspace(0.0, 1.0, knot_count)
        envelope = np.clip(np.sin(np.pi * self.random_path_knots), 0.0, None) ** 1.25
        lateral_walk = np.cumsum(self.rng.normal(0.0, 0.7, knot_count))
        vertical_walk = np.cumsum(self.rng.normal(0.0, 0.5, knot_count))
        lateral_walk -= np.linspace(lateral_walk[0], lateral_walk[-1], knot_count)
        vertical_walk -= np.linspace(vertical_walk[0], vertical_walk[-1], knot_count)
        self.random_lateral = (
            self.curve_strength
            * self.initial_range
            * 0.12
            * envelope
            * lateral_walk
            / max(float(np.max(np.abs(lateral_walk))), 1.0)
        )
        self.random_vertical = (
            self.curve_strength
            * self.initial_range
            * 0.04
            * envelope
            * vertical_walk
            / max(float(np.max(np.abs(vertical_walk))), 1.0)
        )
        self.random_lateral[-2:] = self.random_vertical[-2:] = 0.0
        self.random_lookahead = float(self.rng.uniform(0.18, 0.24))

    def _interpolate_random_offset(self, progress: float, offsets: np.ndarray) -> float:
        upper = int(np.searchsorted(self.random_path_knots, progress, side="right"))
        upper = int(np.clip(upper, 1, len(self.random_path_knots) - 1))
        lower = upper - 1
        start, end = self.random_path_knots[lower], self.random_path_knots[upper]
        fraction = np.clip((progress - start) / max(end - start, 1e-8), 0.0, 1.0)
        smooth = fraction * fraction * (3.0 - 2.0 * fraction)
        return float(offsets[lower] + smooth * (offsets[upper] - offsets[lower]))

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
                float(self.bspline_lateral(progress)) * terminal_envelope,
                float(self.bspline_vertical(progress)) * terminal_envelope,
            )
        return 0.0, 0.0

    def _path_direction(self, target: PointMassUAV) -> np.ndarray:
        goal_direction = -self.initial_position / self.initial_range
        travelled = float(np.dot(target.position - self.initial_position, goal_direction))
        progress = max(travelled / self.initial_range, 0.0)
        if self.motion == "random":
            if 0.55 <= progress < 1.0:
                to_goal = -target.position
                if np.linalg.norm(to_goal) > 1e-8:
                    return to_goal / np.linalg.norm(to_goal)
            if progress >= 1.25:
                return goal_direction
            lookahead_progress = min(progress + self.random_lookahead, 1.25)
            lateral = vertical = 0.0
            if lookahead_progress < 1.0:
                lateral = self._interpolate_random_offset(
                    lookahead_progress, self.random_lateral
                )
                vertical = self._interpolate_random_offset(
                    lookahead_progress, self.random_vertical
                )
        else:
            capture_progress = 0.5 if self.motion in ("multi_sine", "bspline") else 1.0
            if progress >= capture_progress:
                to_goal = -target.position
                return to_goal / (np.linalg.norm(to_goal) + 1e-12)
            lookahead_progress = min(progress + self.lookahead, 1.0)
            lateral, vertical = self._curve_offset(lookahead_progress)
        aim_point = (
            self.initial_position * (1.0 - lookahead_progress)
            + lateral * self.lateral
            + vertical * self.vertical
        )
        direction = aim_point - target.position
        return direction / (np.linalg.norm(direction) + 1e-12)

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
        if self.motion == "cosine":
            progress = float(
                np.clip(1.0 - goal_norm / self.initial_range, 0.0, 1.0)
            )
            envelope = np.sin(np.pi * progress) ** 2
            lateral_gain = (
                self.curve_strength
                * 0.35
                * envelope
                * np.sin(2.0 * np.pi * progress)
            )
            vertical_gain = (
                self.curve_strength
                * 0.10
                * envelope
                * np.sin(3.0 * np.pi * progress)
            )
            desired_direction = (
                goal_direction
                + lateral_gain * self.lateral
                + vertical_gain * self.vertical
            )
        elif self.motion in EVALUATION_MOTION_MODES and self.motion != "line":
            desired_direction = self._path_direction(target)
        elif self.motion == "jink":
            lateral_gain = 0.18 * np.sin(0.45 * time_seconds)
            if 80.0 <= closest_interceptor_distance <= 150.0:
                self.jink_started = True
            if self.jink_started:
                phase = int(max(time_seconds, 0.0) / 1.5)
                self.jink_sign = -1.0 if phase % 2 else 1.0
                lateral_gain += 0.9 * self.jink_sign
            desired_direction = goal_direction + lateral_gain * self.lateral

        desired_velocity = velocity_along_direction(desired_direction, target.limits, ratio=0.9)
        current_speed = float(np.linalg.norm(target.velocity))
        desired_speed = float(np.linalg.norm(desired_velocity))
        if self.motion in ("multi_sine", "bspline"):
            distance_to_goal = float(np.linalg.norm(target.position))
            safe_speed = np.sqrt(
                2.0
                * 0.10
                * target.limits.max_horizontal_acceleration
                * max(distance_to_goal - 5.0, 0.0)
            )
            if desired_speed > safe_speed:
                desired_velocity *= safe_speed / (desired_speed + 1e-12)
                desired_speed = float(np.linalg.norm(desired_velocity))
        if current_speed > 1e-6 and desired_speed > 1e-6:
            desired_velocity *= min(current_speed, desired_speed) / desired_speed
        acceleration = (desired_velocity - target.velocity) / self.dt
        if self.motion in ("multi_sine", "bspline"):
            travelled = float(
                np.dot(
                    target.position - self.initial_position,
                    -self.initial_position / self.initial_range,
                )
            )
            if travelled / self.initial_range >= 0.50:
                acceleration = -0.04 * target.position - 0.40 * target.velocity
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
    elevation = float(rng.uniform(-np.pi / 4.0, np.pi / 4.0))
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
        ratio=float(rng.uniform(0.55, 0.95)),
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
        ratio=0.85,
    )
    interceptor = PointMassUAV(
        interceptor_limits,
        dt=config.dt,
        velocity=interceptor_velocity,
        command_delay_steps=config.command_delay_steps,
        acceleration_response_tau_s=config.acceleration_response_tau_s,
    )
    controller = LOSGuidanceController(config.guidance_config)
    safety_gate = GuidanceSafetyGate(
        controller,
        interceptor_limits,
        GuidanceSafetyGateConfig(
            loss_behavior=config.guidance_loss_behavior,
            coast_dt_s=config.dt,
        ),
    )
    sensor = _make_sensor(config, seed)
    detection_rng = np.random.default_rng(seed + 10000)
    camera_fov = _make_fov(config)
    camera = BodyAttitudeCameraModel(
        config.dt,
        response_tau_s=config.camera_attitude_response_tau_s,
        gimbal_pitch_deg=config.camera_gimbal_pitch_deg,
    )
    camera.reset(interceptor.velocity)
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
    command_action_history = [np.zeros(3, dtype=np.float64)]
    actual_acceleration_history = [interceptor.acceleration.copy()]
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
        true_relative_velocity = target.velocity - interceptor.velocity
        delayed_relative_position, accurate_los_rate = accurate_delayed_los_rate(
            true_relative_position,
            true_relative_velocity,
            config.sensor_latency_s,
        )
        camera.update(interceptor.velocity, interceptor.acceleration)
        physically_visible = (
            not config.fov_enabled
            or camera_fov.contains_frame(delayed_relative_position, camera.frame())
        )
        target_visible = bool(
            physically_visible
            and detection_rng.random() >= config.detection_dropout_probability
        )
        observed_relative_position = (
            sensor.measure(delayed_relative_position)
            if target_visible
            else np.zeros(3, dtype=np.float64)
        )
        range_error_fraction = (
            sensor.last_range_error_fraction if target_visible else np.nan
        )
        command = safety_gate.command(
            current_time_s=step * config.dt,
            measurement_time_s=step * config.dt - config.sensor_latency_s,
            target_visible=target_visible,
            relative_position_ned_m=observed_relative_position,
            own_velocity_ned_mps=interceptor.velocity,
            measured_los_rate_rad_s=(
                accurate_los_rate
                if config.los_rate_source == "accurate_delayed"
                else None
            ),
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
        command_action = (
            np.zeros(3, dtype=np.float64)
            if command is None
            else command.normalized_action
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
        command_action_history.append(command_action.copy())
        actual_acceleration_history.append(interceptor.acceleration.copy())
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
        command_actions=np.asarray(command_action_history)[:, None, :],
        actual_accelerations=np.asarray(actual_acceleration_history)[:, None, :],
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
                command_delay_steps=config.command_delay_steps,
                acceleration_response_tau_s=config.acceleration_response_tau_s,
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
            GuidanceSafetyGate(
                controllers[-1],
                interceptor_limits,
                GuidanceSafetyGateConfig(
                    loss_behavior=config.guidance_loss_behavior,
                    coast_dt_s=config.dt,
                ),
            )
        )
        sensors.append(_make_sensor(config, seed + index))
    detection_rngs = [
        np.random.default_rng(seed + 10000 + index)
        for index in range(3)
    ]
    camera_fov = _make_fov(config)
    cameras = [
        BodyAttitudeCameraModel(
            config.dt,
            response_tau_s=config.camera_attitude_response_tau_s,
            gimbal_pitch_deg=config.camera_gimbal_pitch_deg,
        )
        for _ in range(3)
    ]
    for camera, interceptor in zip(cameras, interceptors):
        camera.reset(interceptor.velocity)

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
    command_action_history = [np.zeros((3, 3), dtype=np.float64)]
    actual_acceleration_history = [
        np.vstack([item.acceleration for item in interceptors])
    ]
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
        step_command_actions = []
        step_actual_accelerations = []
        step_visibility = []
        step_range_errors = []
        for index, interceptor in enumerate(interceptors):
            true_relative_position = target.position - interceptor.position
            true_relative_velocity = target.velocity - interceptor.velocity
            delayed_relative_position, accurate_los_rate = accurate_delayed_los_rate(
                true_relative_position,
                true_relative_velocity,
                config.sensor_latency_s,
            )
            cameras[index].update(interceptor.velocity, interceptor.acceleration)
            physically_visible = (
                not config.fov_enabled
                or camera_fov.contains_frame(
                    delayed_relative_position, cameras[index].frame()
                )
            )
            target_visible = bool(
                physically_visible
                and detection_rngs[index].random()
                >= config.detection_dropout_probability
            )
            observed_relative_position = (
                sensors[index].measure(delayed_relative_position)
                if target_visible
                else np.zeros(3, dtype=np.float64)
            )
            range_error_fraction = (
                sensors[index].last_range_error_fraction if target_visible else np.nan
            )
            command = safety_gates[index].command(
                current_time_s=step * config.dt,
                measurement_time_s=step * config.dt - config.sensor_latency_s,
                target_visible=target_visible,
                relative_position_ned_m=observed_relative_position,
                own_velocity_ned_mps=interceptor.velocity,
                measured_los_rate_rad_s=(
                    accurate_los_rate
                    if config.los_rate_source == "accurate_delayed"
                    else None
                ),
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
            command_action = (
                np.zeros(3, dtype=np.float64)
                if command is None
                else command.normalized_action
            )
            interceptor.step(command_acceleration)
            step_desired_velocities.append(command_velocity.copy())
            step_accelerations.append(command_acceleration.copy())
            step_command_actions.append(command_action.copy())
            step_actual_accelerations.append(interceptor.acceleration.copy())
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
        command_action_history.append(np.vstack(step_command_actions))
        actual_acceleration_history.append(np.vstack(step_actual_accelerations))
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
        command_actions=np.asarray(command_action_history),
        actual_accelerations=np.asarray(actual_acceleration_history),
        target_visible=np.asarray(visibility_history, dtype=bool),
        range_error_fraction=np.asarray(range_error_history, dtype=np.float64),
    )
