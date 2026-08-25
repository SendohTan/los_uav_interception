from __future__ import annotations

import argparse

from .guidance import LOSGuidanceConfig
from .plotting import plot_result, save_csv
from .simulation import MOTION_MODES, SimulationConfig, run_multi, run_single


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run standalone LOS UAV interception")
    parser.add_argument("scope", choices=("single", "multi"))
    parser.add_argument("--motion", choices=MOTION_MODES, default="line")
    parser.add_argument("--interceptor-type", choices=("A", "B"), default="A")
    parser.add_argument("--target-type", choices=("C", "D"), default="C")
    parser.add_argument("--success-radius", type=float, default=0.6)
    parser.add_argument("--angle-noise-deg", type=float, default=0.5)
    parser.add_argument("--camera-constant-pixel-m", type=float, default=1200.0)
    parser.add_argument("--pixel-error-max", type=int, default=1)
    parser.add_argument("--no-fov", action="store_true")
    parser.add_argument("--fov-horizontal-deg", type=float, default=24.0)
    parser.add_argument("--fov-vertical-deg", type=float, default=16.0)
    parser.add_argument("--camera-attitude-response-tau-s", type=float, default=0.25)
    parser.add_argument("--camera-gimbal-pitch-deg", type=float, default=0.0)
    parser.add_argument("--camera-frame-rate-hz", type=float, default=30.0)
    parser.add_argument("--sensor-latency-s", type=float, default=None)
    parser.add_argument("--detection-dropout-probability", type=float, default=0.01)
    parser.add_argument(
        "--los-rate-source",
        choices=("accurate_delayed", "bearing_history"),
        default="accurate_delayed",
    )
    parser.add_argument("--command-delay-steps", type=int, default=1)
    parser.add_argument("--acceleration-response-tau-s", type=float, default=0.2)
    parser.add_argument(
        "--guidance-profile",
        choices=("legacy", "stable", "stable_v02", "conservative", "flight_test"),
        default="stable",
    )
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--plot", default="outputs/trajectory.png")
    parser.add_argument("--csv", default="outputs/trajectory.csv")
    return parser


def main() -> None:
    arguments = build_parser().parse_args()
    if arguments.camera_frame_rate_hz <= 0.0:
        raise ValueError("camera frame rate must be positive")
    sensor_latency_s = (
        1.0 / arguments.camera_frame_rate_hz
        if arguments.sensor_latency_s is None
        else arguments.sensor_latency_s
    )
    guidance_profiles = {
        "legacy": LOSGuidanceConfig.legacy,
        "stable": LOSGuidanceConfig,
        "stable_v02": LOSGuidanceConfig.stable_v02,
        "conservative": LOSGuidanceConfig.conservative,
        "flight_test": LOSGuidanceConfig.flight_test,
    }
    config = SimulationConfig(
        success_radius=arguments.success_radius,
        interceptor_type=arguments.interceptor_type,
        target_type=arguments.target_type,
        target_motion=arguments.motion,
        angle_noise_std_deg=arguments.angle_noise_deg,
        camera_constant_pixel_m=arguments.camera_constant_pixel_m,
        pixel_error_max=arguments.pixel_error_max,
        fov_enabled=not arguments.no_fov,
        fov_horizontal_deg=arguments.fov_horizontal_deg,
        fov_vertical_deg=arguments.fov_vertical_deg,
        camera_attitude_response_tau_s=arguments.camera_attitude_response_tau_s,
        camera_gimbal_pitch_deg=arguments.camera_gimbal_pitch_deg,
        camera_frame_rate_hz=arguments.camera_frame_rate_hz,
        sensor_latency_s=sensor_latency_s,
        detection_dropout_probability=arguments.detection_dropout_probability,
        los_rate_source=arguments.los_rate_source,
        command_delay_steps=arguments.command_delay_steps,
        acceleration_response_tau_s=arguments.acceleration_response_tau_s,
        guidance_config=guidance_profiles[arguments.guidance_profile](),
    )
    runner = run_single if arguments.scope == "single" else run_multi
    result = runner(config, seed=arguments.seed)
    plot_path = plot_result(result, arguments.plot)
    csv_path = save_csv(result, arguments.csv, config.dt)
    print(f"success={result.success}")
    print(f"minimum_distance_m={result.minimum_distance:.6f}")
    print(f"elapsed_time_s={result.elapsed_time:.2f}")
    print(f"hit_probability_F={result.hit_metrics['P']:.6f}")
    print(f"plot={plot_path}")
    print(f"csv={csv_path}")
