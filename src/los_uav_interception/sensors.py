from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class BearingRangeNoise:
    range_bias_fraction: float = 0.0
    range_jitter_std: float = 0.0
    angle_noise_std_deg: float = 0.0


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
