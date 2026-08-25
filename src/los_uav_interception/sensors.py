from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class BearingRangeNoise:
    camera_constant_pixel_m: float = 1200.0
    pixel_error_max: int = 1
    angle_noise_std_deg: float = 0.0

    def __post_init__(self) -> None:
        if self.camera_constant_pixel_m <= 0.0:
            raise ValueError("camera_constant_pixel_m must be positive")
        if self.pixel_error_max < 0:
            raise ValueError("pixel_error_max must be non-negative")
        if self.angle_noise_std_deg < 0.0:
            raise ValueError("angle_noise_std_deg must be non-negative")


def pixel_quantized_range(
    true_distance: float,
    camera_constant_pixel_m: float,
    pixel_error: int = 0,
) -> tuple[float, float, int]:
    if true_distance <= 0.0:
        raise ValueError("true_distance must be positive")
    if camera_constant_pixel_m <= 0.0:
        raise ValueError("camera_constant_pixel_m must be positive")
    apparent_pixels = camera_constant_pixel_m / true_distance
    quantized_pixels = max(int(np.rint(apparent_pixels)), 1)
    measured_pixels = max(quantized_pixels + int(pixel_error), 1)
    observed_distance = camera_constant_pixel_m / measured_pixels
    return float(observed_distance), float(apparent_pixels), measured_pixels


@dataclass(frozen=True)
class CameraFOV:
    horizontal_deg: float = 24.0
    vertical_deg: float = 16.0

    def __post_init__(self) -> None:
        if not 0.0 < self.horizontal_deg <= 360.0:
            raise ValueError("horizontal_deg must be in (0, 360]")
        if not 0.0 < self.vertical_deg <= 180.0:
            raise ValueError("vertical_deg must be in (0, 180]")

    def angular_offsets(
        self,
        relative_position: np.ndarray,
        boresight_direction: np.ndarray,
    ) -> tuple[float, float]:
        relative_position = np.asarray(relative_position, dtype=np.float64)
        boresight_direction = np.asarray(boresight_direction, dtype=np.float64)
        if relative_position.shape != (3,) or boresight_direction.shape != (3,):
            raise ValueError("FOV vectors must have shape (3,)")
        relative_norm = float(np.linalg.norm(relative_position))
        boresight_norm = float(np.linalg.norm(boresight_direction))
        if relative_norm < 1e-12 or boresight_norm < 1e-12:
            return 0.0, 0.0
        line_of_sight = relative_position / relative_norm
        forward = boresight_direction / boresight_norm
        ned_down = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        right = np.cross(ned_down, forward)
        right_norm = float(np.linalg.norm(right))
        if right_norm < 1e-12:
            right = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        else:
            right /= right_norm
        down = np.cross(forward, right)
        down /= float(np.linalg.norm(down)) + 1e-12
        forward_component = float(np.dot(line_of_sight, forward))
        right_component = float(np.dot(line_of_sight, right))
        down_component = float(np.dot(line_of_sight, down))
        horizontal_offset = float(np.arctan2(right_component, forward_component))
        vertical_offset = float(
            np.arctan2(
                down_component,
                np.hypot(forward_component, right_component),
            )
        )
        return horizontal_offset, vertical_offset

    def contains(
        self,
        relative_position: np.ndarray,
        boresight_direction: np.ndarray,
    ) -> bool:
        horizontal, vertical = self.angular_offsets(
            relative_position,
            boresight_direction,
        )
        return bool(
            abs(horizontal) <= np.radians(self.horizontal_deg) / 2.0
            and abs(vertical) <= np.radians(self.vertical_deg) / 2.0
        )

    def contains_frame(
        self,
        relative_position: np.ndarray,
        frame: tuple[np.ndarray, np.ndarray, np.ndarray],
    ) -> bool:
        relative_position = np.asarray(relative_position, dtype=np.float64)
        forward, right, down = (
            np.asarray(axis, dtype=np.float64) for axis in frame
        )
        forward_component = float(np.dot(relative_position, forward))
        right_component = float(np.dot(relative_position, right))
        down_component = float(np.dot(relative_position, down))
        horizontal = np.arctan2(right_component, forward_component)
        vertical = np.arctan2(
            down_component,
            np.hypot(forward_component, right_component),
        )
        return bool(
            abs(horizontal) <= np.radians(self.horizontal_deg) / 2.0
            and abs(vertical) <= np.radians(self.vertical_deg) / 2.0
        )


class BodyAttitudeCameraModel:
    def __init__(
        self,
        dt: float,
        response_tau_s: float = 0.25,
        gimbal_pitch_deg: float = 0.0,
        max_roll_deg: float = 45.0,
        max_pitch_deg: float = 30.0,
    ) -> None:
        self.dt = float(dt)
        self.response_tau_s = float(response_tau_s)
        self.gimbal_pitch_rad = float(np.radians(gimbal_pitch_deg))
        self.max_roll_rad = float(np.radians(max_roll_deg))
        self.max_pitch_rad = float(np.radians(max_pitch_deg))
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0

    @staticmethod
    def _wrap_angle(value: float) -> float:
        return float((value + np.pi) % (2.0 * np.pi) - np.pi)

    def reset(self, velocity: np.ndarray) -> None:
        velocity = np.asarray(velocity, dtype=np.float64)
        horizontal_speed = float(np.linalg.norm(velocity[:2]))
        if horizontal_speed > 1e-8:
            self.yaw = float(np.arctan2(velocity[1], velocity[0]))
            self.pitch = float(np.arctan2(-velocity[2], horizontal_speed))
        else:
            self.yaw = 0.0
            self.pitch = 0.0
        self.roll = 0.0

    def update(self, velocity: np.ndarray, acceleration: np.ndarray) -> None:
        velocity = np.asarray(velocity, dtype=np.float64)
        acceleration = np.asarray(acceleration, dtype=np.float64)
        horizontal_speed = float(np.linalg.norm(velocity[:2]))
        target_yaw = self.yaw
        if horizontal_speed > 1e-8:
            target_yaw = float(np.arctan2(velocity[1], velocity[0]))
        target_pitch = float(np.arctan2(-velocity[2], max(horizontal_speed, 1e-8)))
        right_horizontal = np.array(
            [-np.sin(target_yaw), np.cos(target_yaw), 0.0], dtype=np.float64
        )
        lateral_acceleration = float(np.dot(acceleration, right_horizontal))
        target_roll = float(np.arctan2(lateral_acceleration, 9.80665))
        target_roll = float(np.clip(target_roll, -self.max_roll_rad, self.max_roll_rad))
        target_pitch = float(
            np.clip(target_pitch, -self.max_pitch_rad, self.max_pitch_rad)
        )
        blend = (
            1.0
            if self.response_tau_s <= 0.0
            else 1.0 - float(np.exp(-self.dt / self.response_tau_s))
        )
        self.yaw = self._wrap_angle(
            self.yaw + blend * self._wrap_angle(target_yaw - self.yaw)
        )
        self.pitch += blend * (target_pitch - self.pitch)
        self.roll += blend * (target_roll - self.roll)

    @staticmethod
    def _rotation_x(angle: float) -> np.ndarray:
        cosine, sine = np.cos(angle), np.sin(angle)
        return np.array(
            [[1.0, 0.0, 0.0], [0.0, cosine, -sine], [0.0, sine, cosine]],
            dtype=np.float64,
        )

    @staticmethod
    def _rotation_y(angle: float) -> np.ndarray:
        cosine, sine = np.cos(angle), np.sin(angle)
        return np.array(
            [[cosine, 0.0, sine], [0.0, 1.0, 0.0], [-sine, 0.0, cosine]],
            dtype=np.float64,
        )

    @staticmethod
    def _rotation_z(angle: float) -> np.ndarray:
        cosine, sine = np.cos(angle), np.sin(angle)
        return np.array(
            [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

    def frame(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        body_to_ned = (
            self._rotation_z(self.yaw)
            @ self._rotation_y(self.pitch)
            @ self._rotation_x(self.roll)
        )
        camera_to_ned = body_to_ned @ self._rotation_y(self.gimbal_pitch_rad)
        return tuple(camera_to_ned[:, axis].copy() for axis in range(3))


class RelativePositionSensor:
    """Apply range and bearing noise to a true NED relative-position vector."""

    def __init__(
        self,
        noise: BearingRangeNoise | None = None,
        seed: int | None = None,
    ) -> None:
        self.noise = BearingRangeNoise() if noise is None else noise
        self.rng = np.random.default_rng(seed)
        self.last_apparent_pixels = np.nan
        self.last_measured_pixels = 0
        self.last_range_error_fraction = np.nan

    def measure(self, relative_position: np.ndarray) -> np.ndarray:
        relative_position = np.asarray(relative_position, dtype=np.float64)
        distance = float(np.linalg.norm(relative_position))
        if distance < 1e-12:
            return relative_position.copy()

        azimuth = float(np.arctan2(relative_position[1], relative_position[0]))
        elevation = float(
            np.arcsin(np.clip(-relative_position[2] / distance, -1.0, 1.0))
        )
        pixel_error = int(
            self.rng.integers(
                -self.noise.pixel_error_max,
                self.noise.pixel_error_max + 1,
            )
        )
        observed_distance, apparent_pixels, measured_pixels = pixel_quantized_range(
            distance,
            self.noise.camera_constant_pixel_m,
            pixel_error,
        )
        self.last_apparent_pixels = apparent_pixels
        self.last_measured_pixels = measured_pixels
        self.last_range_error_fraction = observed_distance / distance - 1.0
        angle_std = np.radians(self.noise.angle_noise_std_deg)
        angle_limit = 3.0 * angle_std
        observed_azimuth = azimuth + float(
            np.clip(self.rng.normal(0.0, angle_std), -angle_limit, angle_limit)
        )
        observed_elevation = elevation + float(
            np.clip(self.rng.normal(0.0, angle_std), -angle_limit, angle_limit)
        )
        return np.array(
            [
                observed_distance
                * np.cos(observed_elevation)
                * np.cos(observed_azimuth),
                observed_distance
                * np.cos(observed_elevation)
                * np.sin(observed_azimuth),
                -observed_distance * np.sin(observed_elevation),
            ],
            dtype=np.float64,
        )
