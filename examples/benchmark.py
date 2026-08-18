import argparse

import numpy as np

from los_uav_interception import SimulationConfig, run_multi, run_single
from los_uav_interception.simulation import MOTION_MODES


def main() -> None:
    parser = argparse.ArgumentParser(description="Benchmark LOS interception")
    parser.add_argument("scope", choices=("single", "multi"))
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--angle-noise-deg", type=float, default=0.5)
    parser.add_argument("--range-bias", type=float, default=0.075)
    parser.add_argument("--range-jitter", type=float, default=0.005)
    parser.add_argument("--no-fov", action="store_true")
    parser.add_argument("--fov-horizontal-deg", type=float, default=24.0)
    parser.add_argument("--fov-vertical-deg", type=float, default=16.0)
    parser.add_argument("--seed", type=int, default=1000)
    arguments = parser.parse_args()
    runner = run_single if arguments.scope == "single" else run_multi

    for motion in MOTION_MODES:
        results = [
            runner(
                SimulationConfig(
                    target_motion=motion,
                    angle_noise_std_deg=arguments.angle_noise_deg,
                    range_bias_fraction=arguments.range_bias,
                    range_jitter_std=arguments.range_jitter,
                    fov_enabled=not arguments.no_fov,
                    fov_horizontal_deg=arguments.fov_horizontal_deg,
                    fov_vertical_deg=arguments.fov_vertical_deg,
                ),
                seed=arguments.seed + episode,
            )
            for episode in range(arguments.episodes)
        ]
        success_rate = np.mean([result.success for result in results])
        mean_minimum_distance = np.mean(
            [result.minimum_distance for result in results]
        )
        print(
            f"{motion:10s} success={success_rate:7.2%} "
            f"mean_minimum_distance={mean_minimum_distance:.3f} m"
        )


if __name__ == "__main__":
    main()
