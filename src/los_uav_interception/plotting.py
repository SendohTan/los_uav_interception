from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from .simulation import SimulationResult


def plot_result(result: SimulationResult, output_path: str | Path) -> Path:
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure = plt.figure(figsize=(9, 7), constrained_layout=True)
    axis = figure.add_subplot(111, projection="3d")
    target = result.target_positions
    axis.plot(target[:, 0], target[:, 1], target[:, 2], label="Target", linewidth=2.2)
    for index in range(result.interceptor_positions.shape[1]):
        trajectory = result.interceptor_positions[:, index, :]
        axis.plot(
            trajectory[:, 0],
            trajectory[:, 1],
            trajectory[:, 2],
            label=f"Interceptor {index + 1}",
            linewidth=1.8,
        )
    axis.scatter(*target[0], marker="o", s=45, color="black", label="Target start")
    axis.set_xlabel("North / m")
    axis.set_ylabel("East / m")
    axis.set_zlabel("Down / m")
    axis.set_title(
        f"LOS interception | success={result.success} | "
        f"min={result.minimum_distance:.3f} m | "
        f"visible={np.mean(result.target_visible):.1%} | "
        f"range MAE={np.nanmean(np.abs(result.range_error_fraction)):.1%}"
    )
    axis.legend(loc="best")
    figure.savefig(output_path, dpi=220)
    plt.close(figure)
    return output_path


def save_csv(result: SimulationResult, output_path: str | Path, dt: float) -> Path:
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    for step in range(len(result.target_positions)):
        target = result.target_positions[step]
        for index in range(result.interceptor_positions.shape[1]):
            interceptor = result.interceptor_positions[step, index]
            rows.append(
                [
                    step * dt,
                    index,
                    *interceptor,
                    *target,
                    result.distances[step, index],
                    int(result.target_visible[step, index]),
                    result.range_error_fraction[step, index],
                ]
            )
    np.savetxt(
        output_path,
        np.asarray(rows),
        delimiter=",",
        header=(
            "time_s,interceptor_id,interceptor_north_m,interceptor_east_m,"
            "interceptor_down_m,target_north_m,target_east_m,target_down_m,distance_m,"
            "target_visible,range_error_fraction"
        ),
        comments="",
    )
    return output_path
