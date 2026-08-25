import argparse
import csv
import os

import numpy as np

from los_uav_interception import (
    LOSGuidanceConfig,
    SimulationConfig,
    analyze_oscillation,
    run_multi,
    run_single,
)
from los_uav_interception.simulation import EVALUATION_MOTION_MODES


PROFILES = {
    "legacy": LOSGuidanceConfig.legacy(),
    "stable": LOSGuidanceConfig(),
    "stable_v02": LOSGuidanceConfig.stable_v02(),
    "conservative": LOSGuidanceConfig.conservative(),
    "flight_test": LOSGuidanceConfig.flight_test(),
}


def episode_metrics(result, dt: float):
    return analyze_oscillation(
        result.command_actions,
        result.actual_accelerations,
        dt,
    )


def summarize(rows: list[dict], profiles: list[str]) -> list[dict]:
    summaries = []
    for profile_name in profiles:
        for motion in (*EVALUATION_MOTION_MODES, "overall"):
            selected = [
                row
                for row in rows
                if row["profile"] == profile_name
                and (motion == "overall" or row["motion"] == motion)
            ]
            summaries.append(
                {
                    "scope": selected[0]["scope"],
                    "profile": profile_name,
                    "los_rate_source": selected[0]["los_rate_source"],
                    "motion": motion,
                    "episodes": len(selected),
                    "success_rate": float(np.mean([row["success"] for row in selected])),
                    "oscillation_rate": float(
                        np.mean([row["oscillatory"] for row in selected])
                    ),
                    "mean_minimum_distance_m": float(
                        np.mean([row["minimum_distance_m"] for row in selected])
                    ),
                    "mean_elapsed_time_s": float(
                        np.mean([row["elapsed_time_s"] for row in selected])
                    ),
                    "mean_max_command_reversal_rate_hz": float(
                        np.mean(
                            [row["max_command_reversal_rate_hz"] for row in selected]
                        )
                    ),
                    "mean_actual_jerk_rms_mps3": float(
                        np.mean(
                            [row["actual_jerk_rms_mps3"] for row in selected]
                        )
                    ),
                    "mean_oscillation_band_power_ratio": float(
                        np.mean([row["oscillation_band_power_ratio"] for row in selected])
                    ),
                    "mean_acceleration_peak_to_peak_mps2": float(
                        np.mean([row["acceleration_peak_to_peak_mps2"] for row in selected])
                    ),
                    "reversal_threshold_exceedance_rate": float(
                        np.mean(
                            [row["max_command_reversal_rate_hz"] >= 0.4 for row in selected]
                        )
                    ),
                    "band_power_threshold_exceedance_rate": float(
                        np.mean(
                            [row["oscillation_band_power_ratio"] >= 0.25 for row in selected]
                        )
                    ),
                    "acceleration_peak_to_peak_threshold_exceedance_rate": float(
                        np.mean(
                            [row["acceleration_peak_to_peak_mps2"] >= 0.8 for row in selected]
                        )
                    ),
                    "mean_agent_visible_fraction": float(
                        np.mean([row["agent_visible_fraction"] for row in selected])
                    ),
                    "mean_all_agents_lost_fraction": float(
                        np.mean([row["all_agents_lost_fraction"] for row in selected])
                    ),
                    "mean_absolute_range_error_fraction": float(
                        np.mean(
                            [row["mean_absolute_range_error_fraction"] for row in selected]
                        )
                    ),
                    "range_error_over_20_percent_fraction": float(
                        np.mean(
                            [
                                row["range_error_over_20_percent_fraction"]
                                for row in selected
                            ]
                        )
                    ),
                }
            )
    return summaries


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare LOS guidance profiles")
    parser.add_argument("--scope", choices=("single", "multi"), default="single")
    parser.add_argument("--episodes", type=int, default=100)
    parser.add_argument("--seed", type=int, default=2026081801)
    parser.add_argument("--angle-noise-deg", type=float, default=0.5)
    parser.add_argument("--camera-constant-pixel-m", type=float, default=1200.0)
    parser.add_argument("--pixel-error-max", type=int, default=1)
    parser.add_argument(
        "--los-rate-source",
        choices=("accurate_delayed", "bearing_history"),
        default="accurate_delayed",
    )
    parser.add_argument("--no-fov", action="store_true")
    parser.add_argument("--fov-horizontal-deg", type=float, default=24.0)
    parser.add_argument("--fov-vertical-deg", type=float, default=16.0)
    parser.add_argument(
        "--profiles",
        nargs="+",
        choices=tuple(PROFILES),
        default=list(PROFILES),
    )
    parser.add_argument("--output", default="outputs/stability_benchmark.csv")
    arguments = parser.parse_args()
    runner = run_single if arguments.scope == "single" else run_multi
    rows = []
    for profile_name in arguments.profiles:
        guidance_config = PROFILES[profile_name]
        for motion_index, motion in enumerate(EVALUATION_MOTION_MODES):
            for episode in range(arguments.episodes):
                seed = arguments.seed + motion_index * 100_000 + episode
                config = SimulationConfig(
                    target_motion=motion,
                    angle_noise_std_deg=arguments.angle_noise_deg,
                    camera_constant_pixel_m=arguments.camera_constant_pixel_m,
                    pixel_error_max=arguments.pixel_error_max,
                    los_rate_source=arguments.los_rate_source,
                    fov_enabled=not arguments.no_fov,
                    fov_horizontal_deg=arguments.fov_horizontal_deg,
                    fov_vertical_deg=arguments.fov_vertical_deg,
                    guidance_config=guidance_config,
                )
                result = runner(config, seed=seed)
                stability = episode_metrics(result, config.dt)
                finite_range_errors = np.abs(
                    result.range_error_fraction[
                        np.isfinite(result.range_error_fraction)
                    ]
                )
                rows.append(
                    {
                        "scope": arguments.scope,
                        "profile": profile_name,
                        "los_rate_source": arguments.los_rate_source,
                        "motion": motion,
                        "seed": seed,
                        "success": int(result.success),
                        "minimum_distance_m": result.minimum_distance,
                        "elapsed_time_s": result.elapsed_time,
                        "oscillatory": int(stability.oscillatory),
                        "oscillatory_aircraft_count": stability.oscillatory_aircraft_count,
                        "max_command_reversal_rate_hz": stability.max_command_reversal_rate_hz,
                        "dominant_frequency_hz": stability.dominant_frequency_hz,
                        "oscillation_band_power_ratio": stability.oscillation_band_power_ratio,
                        "acceleration_peak_to_peak_mps2": stability.acceleration_peak_to_peak_mps2,
                        "actual_jerk_rms_mps3": stability.actual_jerk_rms_mps3,
                        "command_slew_rms_per_s": stability.command_slew_rms_per_s,
                        "command_saturation_fraction": stability.command_saturation_fraction,
                        "agent_visible_fraction": float(np.mean(result.target_visible)),
                        "all_agents_lost_fraction": float(
                            np.mean(~np.any(result.target_visible, axis=1))
                        ),
                        "mean_absolute_range_error_fraction": float(
                            np.mean(finite_range_errors)
                        ),
                        "range_error_over_20_percent_fraction": float(
                            np.mean(finite_range_errors > 0.20)
                        ),
                    }
                )
    os.makedirs(os.path.dirname(arguments.output) or ".", exist_ok=True)
    with open(arguments.output, "w", encoding="utf-8-sig", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    summary_rows = summarize(rows, arguments.profiles)
    summary_path = f"{os.path.splitext(arguments.output)[0]}_summary.csv"
    with open(summary_path, "w", encoding="utf-8-sig", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(summary_rows[0]))
        writer.writeheader()
        writer.writerows(summary_rows)
    for profile_name in arguments.profiles:
        selected = [row for row in rows if row["profile"] == profile_name]
        print(
            f"{profile_name:6s} success={np.mean([row['success'] for row in selected]):7.2%} "
            f"oscillation={np.mean([row['oscillatory'] for row in selected]):7.2%} "
            f"reversal={np.mean([row['max_command_reversal_rate_hz'] for row in selected]):.3f} Hz "
            f"jerk={np.mean([row['actual_jerk_rms_mps3'] for row in selected]):.3f} m/s^3"
        )
    print(os.path.abspath(arguments.output))
    print(os.path.abspath(summary_path))


if __name__ == "__main__":
    main()
