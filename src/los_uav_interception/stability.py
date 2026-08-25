from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class OscillationMetrics:
    oscillatory: bool
    oscillatory_aircraft_count: int
    max_command_reversal_rate_hz: float
    dominant_frequency_hz: float
    oscillation_band_power_ratio: float
    acceleration_peak_to_peak_mps2: float
    actual_jerk_rms_mps3: float
    command_slew_rms_per_s: float
    command_saturation_fraction: float


def _count_reversals(signal: np.ndarray, deadband: float) -> int:
    signs = np.zeros(len(signal), dtype=np.int8)
    signs[signal > deadband] = 1
    signs[signal < -deadband] = -1
    nonzero = signs[signs != 0]
    if len(nonzero) < 2:
        return 0
    return int(np.count_nonzero(nonzero[1:] != nonzero[:-1]))


def _moving_average(values: np.ndarray, window: int) -> np.ndarray:
    window = max(1, min(int(window), len(values)))
    if window <= 1:
        return values.copy()
    kernel = np.ones(window, dtype=np.float64) / window
    padded = np.pad(values, (window // 2, window - 1 - window // 2), mode="edge")
    return np.convolve(padded, kernel, mode="valid")


def _spectral_metrics(
    values: np.ndarray,
    dt: float,
    minimum_frequency: float,
    maximum_frequency: float,
) -> tuple[float, float]:
    if len(values) < 16:
        return 0.0, 0.0
    trend = _moving_average(values, max(3, int(round(1.0 / dt))))
    windowed = (values - trend) * np.hanning(len(values))
    frequencies = np.fft.rfftfreq(len(windowed), d=dt)
    power = np.abs(np.fft.rfft(windowed)) ** 2
    selected = (frequencies >= minimum_frequency) & (
        frequencies <= maximum_frequency
    )
    selected_power = power[selected]
    total_dynamic_power = float(np.sum(power[1:]))
    if selected_power.size == 0 or total_dynamic_power <= 1e-12:
        return 0.0, 0.0
    selected_frequencies = frequencies[selected]
    peak_index = int(np.argmax(selected_power))
    return (
        float(selected_frequencies[peak_index]),
        float(np.sum(selected_power) / total_dynamic_power),
    )


def analyze_oscillation(
    command_actions: np.ndarray,
    actual_accelerations: np.ndarray,
    dt: float,
    *,
    warmup_seconds: float = 2.0,
    action_deadband: float = 0.08,
    reversal_rate_threshold_hz: float = 0.4,
    band_power_threshold: float = 0.25,
    acceleration_peak_to_peak_threshold_mps2: float = 0.8,
    band_min_frequency_hz: float = 0.2,
    band_max_frequency_hz: float = 2.5,
) -> OscillationMetrics:
    if not np.isfinite(dt) or dt <= 0.0:
        raise ValueError("dt must be finite and positive")
    commands = np.asarray(command_actions, dtype=np.float64)
    accelerations = np.asarray(actual_accelerations, dtype=np.float64)
    if commands.ndim == 2:
        commands = commands[:, None, :]
    if accelerations.ndim == 2:
        accelerations = accelerations[:, None, :]
    if commands.ndim != 3 or accelerations.shape != commands.shape:
        raise ValueError("control histories must have shape [time, aircraft, axis]")
    if commands.shape[2] != 3:
        raise ValueError("control histories must contain three axes")

    start = min(
        max(len(commands) - 1, 0),
        max(0, int(round(warmup_seconds / max(dt, 1e-8)))),
    )
    commands = commands[start:]
    accelerations = accelerations[start:]
    duration = max(len(commands) * dt, dt)
    aircraft_metrics = []
    for aircraft in range(commands.shape[1]):
        aircraft_command = commands[:, aircraft]
        aircraft_acceleration = accelerations[:, aircraft]
        reversal_rates = [
            _count_reversals(aircraft_command[:, axis], action_deadband) / duration
            for axis in range(3)
        ]
        axis_rms = np.sqrt(np.mean(np.square(aircraft_acceleration), axis=0))
        spectral_axis = int(np.argmax(axis_rms))
        dominant_frequency, band_power_ratio = _spectral_metrics(
            aircraft_acceleration[:, spectral_axis],
            dt,
            band_min_frequency_hz,
            band_max_frequency_hz,
        )
        peak_to_peak = float(np.max(np.ptp(aircraft_acceleration, axis=0)))
        if len(aircraft_acceleration) > 1:
            jerk = np.diff(aircraft_acceleration, axis=0) / dt
            jerk_rms = float(np.sqrt(np.mean(np.sum(np.square(jerk), axis=1))))
            command_slew = np.diff(aircraft_command, axis=0) / dt
            command_slew_rms = float(
                np.sqrt(np.mean(np.sum(np.square(command_slew), axis=1)))
            )
        else:
            jerk_rms = 0.0
            command_slew_rms = 0.0
        oscillatory = bool(
            duration >= 4.0
            and max(reversal_rates) >= reversal_rate_threshold_hz
            and band_power_ratio >= band_power_threshold
            and peak_to_peak >= acceleration_peak_to_peak_threshold_mps2
        )
        aircraft_metrics.append(
            {
                "oscillatory": oscillatory,
                "reversal": float(max(reversal_rates)),
                "frequency": dominant_frequency,
                "band_ratio": band_power_ratio,
                "peak_to_peak": peak_to_peak,
                "jerk": jerk_rms,
                "slew": command_slew_rms,
                "saturation": float(np.mean(np.abs(aircraft_command) >= 0.999)),
            }
        )

    strongest = max(aircraft_metrics, key=lambda row: row["band_ratio"])
    return OscillationMetrics(
        oscillatory=bool(any(row["oscillatory"] for row in aircraft_metrics)),
        oscillatory_aircraft_count=int(
            sum(row["oscillatory"] for row in aircraft_metrics)
        ),
        max_command_reversal_rate_hz=float(
            max(row["reversal"] for row in aircraft_metrics)
        ),
        dominant_frequency_hz=float(strongest["frequency"]),
        oscillation_band_power_ratio=float(
            max(row["band_ratio"] for row in aircraft_metrics)
        ),
        acceleration_peak_to_peak_mps2=float(
            max(row["peak_to_peak"] for row in aircraft_metrics)
        ),
        actual_jerk_rms_mps3=float(
            np.mean([row["jerk"] for row in aircraft_metrics])
        ),
        command_slew_rms_per_s=float(
            np.mean([row["slew"] for row in aircraft_metrics])
        ),
        command_saturation_fraction=float(
            np.mean([row["saturation"] for row in aircraft_metrics])
        ),
    )
