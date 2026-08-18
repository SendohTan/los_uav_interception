from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class OscillationMetrics:
    oscillatory: bool
    velocity_error_peak_to_peak_mps: float
    maximum_command_reversal_rate_hz: float
    oscillation_band_power_ratio: float
    dominant_frequency_hz: float
    acceleration_jerk_rms_mps3: float


def _moving_average(values: np.ndarray, window: int) -> np.ndarray:
    if window <= 1 or len(values) < window:
        return values.copy()
    kernel = np.ones(window, dtype=np.float64) / window
    padded = np.pad(values, (window // 2, window - 1 - window // 2), mode="edge")
    return np.convolve(padded, kernel, mode="valid")


def _reversal_count(values: np.ndarray, deadband: float) -> int:
    signs = np.zeros(len(values), dtype=np.int8)
    signs[values > deadband] = 1
    signs[values < -deadband] = -1
    nonzero = signs[signs != 0]
    if len(nonzero) < 2:
        return 0
    return int(np.count_nonzero(nonzero[1:] != nonzero[:-1]))


def _spectral_metrics(values: np.ndarray, dt: float) -> tuple[float, float]:
    if len(values) < 16:
        return 0.0, 0.0
    trend = _moving_average(values, max(3, int(round(1.0 / dt))))
    windowed = (values - trend) * np.hanning(len(values))
    frequencies = np.fft.rfftfreq(len(windowed), d=dt)
    power = np.abs(np.fft.rfft(windowed)) ** 2
    selected = (frequencies >= 0.2) & (frequencies <= 2.5)
    total_power = float(np.sum(power[1:]))
    if not np.any(selected) or total_power <= 1e-12:
        return 0.0, 0.0
    selected_power = power[selected]
    selected_frequencies = frequencies[selected]
    peak_frequency = float(selected_frequencies[int(np.argmax(selected_power))])
    return peak_frequency, float(np.sum(selected_power) / total_power)


def analyze_oscillation(
    desired_velocities: np.ndarray,
    actual_velocities: np.ndarray,
    guidance_accelerations: np.ndarray,
    dt: float,
    *,
    warmup_seconds: float = 2.0,
    acceleration_deadband_mps2: float = 0.08,
    reversal_rate_threshold_hz: float = 0.4,
    band_power_threshold: float = 0.25,
    velocity_error_threshold_mps: float = 0.8,
) -> OscillationMetrics:
    if not np.isfinite(dt) or dt <= 0.0:
        raise ValueError("dt must be finite and positive")
    desired = np.asarray(desired_velocities, dtype=np.float64)
    actual = np.asarray(actual_velocities, dtype=np.float64)
    accelerations = np.asarray(guidance_accelerations, dtype=np.float64)
    if desired.ndim == 3:
        desired = desired[:, 0, :]
    if actual.ndim == 3:
        actual = actual[:, 0, :]
    if accelerations.ndim == 3:
        accelerations = accelerations[:, 0, :]
    if desired.shape != actual.shape or desired.shape != accelerations.shape:
        raise ValueError("all histories must have matching shapes")
    if desired.ndim != 2 or desired.shape[1] != 3:
        raise ValueError("histories must have shape (steps, 3) or (steps, agents, 3)")
    start = min(len(desired) - 1, max(0, int(round(warmup_seconds / dt))))
    desired = desired[start:]
    actual = actual[start:]
    accelerations = accelerations[start:]
    duration = max(len(desired) * dt, dt)
    velocity_error = desired - actual
    axis_rms = np.sqrt(np.mean(velocity_error**2, axis=0))
    spectral_axis = int(np.argmax(axis_rms))
    dominant_frequency, band_power_ratio = _spectral_metrics(
        velocity_error[:, spectral_axis], dt
    )
    reversal_rates = [
        _reversal_count(accelerations[:, axis], acceleration_deadband_mps2) / duration
        for axis in range(3)
    ]
    if len(accelerations) > 1:
        acceleration_jerk_rms = float(
            np.sqrt(
                np.mean(
                    np.linalg.norm(np.diff(accelerations, axis=0) / dt, axis=1) ** 2
                )
            )
        )
    else:
        acceleration_jerk_rms = 0.0
    velocity_error_peak_to_peak = float(np.max(np.ptp(velocity_error, axis=0)))
    oscillatory = bool(
        duration >= 4.0
        and max(reversal_rates) >= reversal_rate_threshold_hz
        and band_power_ratio >= band_power_threshold
        and velocity_error_peak_to_peak >= velocity_error_threshold_mps
    )
    return OscillationMetrics(
        oscillatory=oscillatory,
        velocity_error_peak_to_peak_mps=velocity_error_peak_to_peak,
        maximum_command_reversal_rate_hz=float(max(reversal_rates)),
        oscillation_band_power_ratio=band_power_ratio,
        dominant_frequency_hz=dominant_frequency,
        acceleration_jerk_rms_mps3=acceleration_jerk_rms,
    )
