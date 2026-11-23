import csv
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import numpy as np

from los_uav_interception.single_oscillation_audit import REQUIRED_COLUMNS, audit_csv


class SingleOscillationAuditTest(unittest.TestCase):
    def write_log(self, path, *, varying=True, gap=False, multiple_ids=False):
        times = np.arange(601) / 60
        if gap:
            times[350:] += 1 / 60
        with path.open("w", encoding="utf-8", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([*REQUIRED_COLUMNS, "interceptor_id", "image_sample_time_s"])
            for i, time in enumerate(times):
                command = 0.5 * np.sin(2 * np.pi * time) if varying else 0.2
                writer.writerow([f"{time:.6f}", command, 0, 0, 4 * command, 0, 0,
                                 int(multiple_ids and i > 0), f"{time - 0.1:.6f}"])

    def test_60_hz_reversals_and_recorded_delay(self):
        with TemporaryDirectory() as folder:
            path = Path(folder) / "single.csv"
            self.write_log(path)
            result = audit_csv(path)
        self.assertAlmostEqual(result["sample_rate_hz"], 60, places=2)
        self.assertTrue(result["native"]["oscillatory"])
        self.assertEqual(result["reference"]["stride"], 6)
        self.assertTrue(result["reference"]["metrics"]["oscillatory"])
        self.assertAlmostEqual(result["recorded_delays_ms"]["image_sample_time_s"]["median"], 100)

    def test_constant_command_does_not_count_as_reversal(self):
        with TemporaryDirectory() as folder:
            path = Path(folder) / "single.csv"
            self.write_log(path, varying=False)
            self.assertFalse(audit_csv(path)["native"]["oscillatory"])

    def test_missing_tick_is_rejected(self):
        with TemporaryDirectory() as folder:
            path = Path(folder) / "single.csv"
            self.write_log(path, gap=True)
            with self.assertRaisesRegex(ValueError, "uniformly sampled"):
                audit_csv(path)

    def test_mixed_aircraft_are_rejected(self):
        with TemporaryDirectory() as folder:
            path = Path(folder) / "multi.csv"
            self.write_log(path, multiple_ids=True)
            with self.assertRaisesRegex(ValueError, "exactly one aircraft"):
                audit_csv(path)
