from __future__ import annotations

import numpy as np


def _azimuth(vector: np.ndarray) -> float:
    return float(np.degrees(np.arctan2(vector[1], vector[0])) % 360.0)


def _elevation(vector: np.ndarray) -> float:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-12:
        return 0.0
    return float(np.degrees(np.arcsin(np.clip(-vector[2] / norm, -1.0, 1.0))))


def _wrapped_difference(angle: float) -> float:
    return float((angle + 180.0) % 360.0 - 180.0)


def hit_probability_components(
    interceptor_position: np.ndarray,
    interceptor_velocity: np.ndarray,
    target_position: np.ndarray,
    target_velocity: np.ndarray,
) -> dict[str, float | bool]:
    relative_position = np.asarray(target_position) - np.asarray(interceptor_position)
    distance = float(np.linalg.norm(relative_position))
    if distance < 1e-6:
        return {
            "P_v": 1.0,
            "P_a": 1.0,
            "P_e": 1.0,
            "P_d": 1.0,
            "P": 1.0,
            "distance": distance,
            "dtheta_deg": 0.0,
            "dalpha_deg": 0.0,
            "is_head_on": False,
            "is_closing": True,
        }

    tracking_azimuth = _azimuth(relative_position)
    velocity_azimuth = _azimuth(interceptor_velocity)
    tracking_elevation = _elevation(relative_position)
    velocity_elevation = _elevation(interceptor_velocity)
    is_head_on = bool(np.dot(interceptor_velocity, target_velocity) < 0.0)
    velocity_difference = np.asarray(target_velocity) - np.asarray(interceptor_velocity)
    is_closing = bool(np.dot(velocity_difference, interceptor_velocity) <= 0.0)

    probability_velocity = 1.0 if is_closing else 0.0
    azimuth_difference = _wrapped_difference(tracking_azimuth - velocity_azimuth)
    azimuth_sigma = 10.0 if is_head_on else 15.0
    probability_azimuth = float(
        np.exp(-0.5 * (azimuth_difference / azimuth_sigma) ** 2)
    )
    elevation_difference = _wrapped_difference(
        tracking_elevation - velocity_elevation
    )
    probability_elevation = float(
        np.exp(-0.5 * ((elevation_difference - 9.0) / 3.0) ** 2)
    )
    probability_distance = float(np.exp(-0.5 * (distance / 50.0) ** 2))
    probability = float(
        np.clip(
            probability_velocity
            * probability_azimuth
            * probability_elevation
            * probability_distance,
            0.0,
            1.0,
        )
    )
    return {
        "P_v": probability_velocity,
        "P_a": probability_azimuth,
        "P_e": probability_elevation,
        "P_d": probability_distance,
        "P": probability,
        "distance": distance,
        "dtheta_deg": azimuth_difference,
        "dalpha_deg": elevation_difference,
        "is_head_on": is_head_on,
        "is_closing": is_closing,
    }
