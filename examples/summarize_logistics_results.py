"""Summarize recorded logistics simulation outcomes, without rerunning flight models."""

import argparse
from collections import defaultdict
import csv
from pathlib import Path
from statistics import mean
import sys


DATA = Path(__file__).resolve().parents[1] / "docs/logistics_rendezvous_60hz/episodes.csv"
DATASETS = (
    "clock_pilot", "sequential_matrix", "synchronous", "without_near_termination",
    "rate_regression", "rate_factors",
)


def percent(numerator, denominator):
    return f"{100 * numerator / denominator:.2f}" if denominator else ""


def summarize(rows):
    groups = defaultdict(list)
    for row in rows:
        groups[row["variant"], row["group"]].append(row)
    for (variant, group), trials in sorted(groups.items()):
        count = len(trials)
        successes = sum(int(row["success"]) for row in trials)
        oscillations = sum(int(float(row["oscillatory"])) for row in trials)
        visible = mean(float(row["visible_fraction"]) for row in trials)
        swing_10hz = [int(row["oscillatory_common10"]) for row in trials if row["oscillatory_common10"]]
        result = [variant, group, count, successes, percent(successes, count),
                  oscillations, percent(oscillations, count), sum(swing_10hz) if swing_10hz else "",
                  percent(visible, 1)]
        for rank in (2, 3):
            prefix = f"departure_{rank}_"
            frames = [sum(int(row[prefix + field] or 0) for row in trials) for field in (
                "direct_frames", "sampled_frames", "occluded_by_earlier_frames", "eligible_frames"
            )]
            result.extend((percent(frames[0], frames[1]), percent(frames[2], frames[3])))
        yield result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", choices=DATASETS)
    parser.add_argument("--cohort", help="Filter by a cohort such as original60 or added120")
    parser.add_argument("--data", type=Path, default=DATA)
    args = parser.parse_args()
    with args.data.open(encoding="utf-8-sig", newline="") as source:
        rows = [row for row in csv.DictReader(source) if row["dataset"] == args.dataset
                and (args.cohort is None or row["cohort"] == args.cohort)]
    if not rows:
        parser.error("no trials matched the requested dataset and cohort")
    writer = csv.writer(sys.stdout, lineterminator="\n")
    writer.writerow(("variant", "group", "episodes", "successes", "success_pct", "oscillations",
                     "oscillation_pct", "oscillations_reference10", "mean_visible_pct",
                     "second_direct_pct", "second_occluded_by_earlier_pct", "third_direct_pct",
                     "third_occluded_by_earlier_pct"))
    writer.writerows(summarize(rows))


if __name__ == "__main__":
    main()
