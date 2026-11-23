"""Read a single aircraft's recorded telemetry and report oscillation offline."""

import argparse
import csv
from dataclasses import asdict
import json
from pathlib import Path

import numpy as np

from .stability import analyze_oscillation


COMMAND_COLUMNS = tuple(f"command_action_{axis}" for axis in ("north", "east", "down"))
ACCELERATION_COLUMNS = tuple(
    f"actual_acceleration_{axis}_mps2" for axis in ("north", "east", "down")
)
REQUIRED_COLUMNS = ("time_s", *COMMAND_COLUMNS, *ACCELERATION_COLUMNS)
DELAY_COLUMNS = ("image_sample_time_s", "applied_command_source_time_s")


def audit_csv(path: Path, reference_hz: float = 10.0) -> dict:
    """Apply the package's stability rule to timestamped NED telemetry."""
    if not np.isfinite(reference_hz) or reference_hz <= 0:
        raise ValueError("reference frequency must be positive and finite")
    with path.open(encoding="utf-8-sig", newline="") as file:
        reader = csv.DictReader(file)
        columns = reader.fieldnames or ()
        missing = set(REQUIRED_COLUMNS).difference(columns)
        if missing:
            raise ValueError(f"missing columns: {', '.join(sorted(missing))}")
        rows = list(reader)

    if len(rows) < 2:
        raise ValueError("at least two telemetry samples are required")
    if "interceptor_id" in columns and len({row["interceptor_id"] for row in rows}) != 1:
        raise ValueError("provide records for exactly one aircraft")
    try:
        samples = np.array([[float(row[col]) for col in REQUIRED_COLUMNS] for row in rows])
    except (TypeError, ValueError) as error:
        raise ValueError("timestamps, commands and accelerations must be numeric") from error
    if not np.isfinite(samples).all():
        raise ValueError("telemetry contains non-finite samples")
    time, command, acceleration = samples[:, 0], samples[:, 1:4], samples[:, 4:7]
    intervals = np.diff(time)
    dt = float(np.median(intervals))
    if np.any(intervals <= 0):
        raise ValueError("timestamps must be strictly increasing")
    if not np.allclose(intervals, dt, rtol=0.02, atol=1e-6):
        raise ValueError("timestamps must be uniformly sampled within 2%; review gaps before scoring")
    if np.any(np.abs(command) > 1.000001):
        raise ValueError("command_action columns must be normalized to [-1, 1]")

    report = {
        "sample_count": len(rows),
        "time_span_s": float(time[-1] - time[0]),
        "sample_rate_hz": 1.0 / dt,
        "native": asdict(analyze_oscillation(command, acceleration, dt)),
        "reference": None,
        "recorded_delays_ms": {},
    }
    stride = max(1, round(1.0 / (reference_hz * dt)))
    if np.isclose(1.0 / (stride * dt), reference_hz, rtol=0.02):
        report["reference"] = {
            "sample_rate_hz": 1.0 / (stride * dt),
            "stride": stride,
            "metrics": asdict(analyze_oscillation(command[::stride], acceleration[::stride], stride * dt)),
        }

    for column in DELAY_COLUMNS:
        if column not in columns:
            continue
        try:
            delays = np.array([stamp - float(row[column]) for stamp, row in zip(time, rows)
                               if row[column] is not None and row[column].strip()])
        except ValueError as error:
            raise ValueError(f"{column} must contain numeric times or blank cells") from error
        if not len(delays):
            continue
        if not np.isfinite(delays).all() or np.any(delays < -1e-6):
            raise ValueError(f"{column} must contain finite times at or before time_s")
        delays = np.maximum(delays, 0.0)
        report["recorded_delays_ms"][column] = {
            "samples": len(delays),
            "median": float(np.median(delays) * 1000),
            "p95": float(np.percentile(delays, 95) * 1000),
        }
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, help="single-aircraft telemetry CSV")
    parser.add_argument("--reference-hz", type=float, default=10.0)
    args = parser.parse_args()
    try:
        report = audit_csv(args.csv, args.reference_hz)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
