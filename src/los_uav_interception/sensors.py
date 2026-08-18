from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class BearingRangeNoise:
    range_bias_fraction: float = 0.0
    range_jitter_std: float = 0.0
    angle_noise_std_deg: float = 0.0


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


class RelativePositionSensor:
    """Apply range and bearing noise to a true NED relative-position vector."""

    def __init__(
        self,
        noise: BearingRangeNoise | None = None,
        seed: int | None = None,
    ) -> None:
        self.noise = BearingRangeNoise() if noise is None else noise
        self.rng = np.random.default_rng(seed)

    def measure(self, relative_position: np.ndarray) -> np.ndarray:
        relative_position = np.asarray(relative_position, dtype=np.float64)
        distance = float(np.linalg.norm(relative_position))
        if distance < 1e-12:
            return relative_position.copy()

        azimuth = float(np.arctan2(relative_position[1], relative_position[0]))
        elevation = float(
            np.arcsin(np.clip(-relative_position[2] / distance, -1.0, 1.0))
        )
        observed_distance = distance * (
            1.0
            + self.noise.range_bias_fraction
            + self.rng.normal(0.0, self.noise.range_jitter_std)
        )
        angle_std = np.radians(self.noise.angle_noise_std_deg)
        observed_azimuth = azimuth + self.rng.normal(0.0, angle_std)
        observed_elevation = elevation + self.rng.normal(0.0, angle_std)
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
