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
from los_uav_interception.simulation import MOTION_MODES


PROFILES = {
    "legacy": LOSGuidanceConfig.legacy(),
    "stable": LOSGuidanceConfig(),
    "conservative": LOSGuidanceConfig.conservative(),
    "flight_test": LOSGuidanceConfig.flight_test(),
}


def episode_metrics(result, dt: float) -> list:
    metrics = []
    for interceptor_index in range(result.interceptor_velocities.shape[1]):
        metrics.append(
            analyze_oscillation(
                result.desired_velocities[:, interceptor_index, :],
                result.interceptor_velocities[:, interceptor_index, :],
                result.guidance_accelerations[:, interceptor_index, :],
                dt,
            )
        )
    return metrics


def summarize(rows: list[dict], profiles: list[str]) -> list[dict]:
    summaries = []
    for profile_name in profiles:
        for motion in (*MOTION_MODES, "overall"):
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
                    "motion": motion,
                    "episodes": len(selected),
                    "success_rate": float(np.mean([row["success"] for row in selected])),
                    "oscillation_rate": float(
                        np.mean([row["oscillatory"] for row in selected])
                    ),
                    "mean_minimum_distance_m": float(
                        np.mean([row["minimum_distance_m"] for row in selected])
                    ),
                    "mean_maximum_command_reversal_rate_hz": float(
                        np.mean(
                            [row["maximum_command_reversal_rate_hz"] for row in selected]
                        )
                    ),
                    "mean_acceleration_jerk_rms_mps3": float(
                        np.mean(
                            [row["mean_acceleration_jerk_rms_mps3"] for row in selected]
                        )
                    ),
                    "mean_agent_visible_fraction": float(
                        np.mean([row["agent_visible_fraction"] for row in selected])
                    ),
                    "mean_all_agents_lost_fraction": float(
                        np.mean([row["all_agents_lost_fraction"] for row in selected])
                    ),
                }
            )
    return summaries


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare legacy and stable old LOS")
    parser.add_argument("--scope", choices=("single", "multi"), default="single")
    parser.add_argument("--episodes", type=int, default=100)
    parser.add_argument("--seed", type=int, default=2026081801)
    parser.add_argument("--angle-noise-deg", type=float, default=0.5)
    parser.add_argument("--range-bias", type=float, default=0.075)
    parser.add_argument("--range-jitter", type=float, default=0.005)
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
        for motion_index, motion in enumerate(MOTION_MODES):
            for episode in range(arguments.episodes):
                seed = arguments.seed + motion_index * 100_000 + episode
                config = SimulationConfig(
                    target_motion=motion,
                    angle_noise_std_deg=arguments.angle_noise_deg,
                    range_bias_fraction=arguments.range_bias,
                    range_jitter_std=arguments.range_jitter,
                    fov_enabled=not arguments.no_fov,
                    fov_horizontal_deg=arguments.fov_horizontal_deg,
                    fov_vertical_deg=arguments.fov_vertical_deg,
                    guidance_config=guidance_config,
                )
                result = runner(config, seed=seed)
                stability = episode_metrics(result, config.dt)
                rows.append(
                    {
                        "scope": arguments.scope,
                        "profile": profile_name,
                        "motion": motion,
                        "seed": seed,
                        "success": int(result.success),
                        "minimum_distance_m": result.minimum_distance,
                        "oscillatory": int(any(item.oscillatory for item in stability)),
                        "maximum_command_reversal_rate_hz": max(
                            item.maximum_command_reversal_rate_hz for item in stability
                        ),
                        "mean_acceleration_jerk_rms_mps3": float(
                            np.mean([item.acceleration_jerk_rms_mps3 for item in stability])
                        ),
                        "agent_visible_fraction": float(np.mean(result.target_visible)),
                        "all_agents_lost_fraction": float(
                            np.mean(~np.any(result.target_visible, axis=1))
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
            f"reversal={np.mean([row['maximum_command_reversal_rate_hz'] for row in selected]):.3f} Hz "
            f"jerk={np.mean([row['mean_acceleration_jerk_rms_mps3'] for row in selected]):.3f} m/s^3"
        )
    print(os.path.abspath(arguments.output))
    print(os.path.abspath(summary_path))


if __name__ == "__main__":
    main()
