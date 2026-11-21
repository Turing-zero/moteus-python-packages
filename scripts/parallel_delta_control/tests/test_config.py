import dataclasses
import json
import tempfile
import unittest
from pathlib import Path

from params import (AppConfig, ConfigError, MechanismParams, load_config,
                    save_config)


class ConfigTests(unittest.TestCase):
    def test_defaults(self):
        config = AppConfig()
        config.validate()
        self.assertEqual(config.transmission.total_reduction, 17.76)
        self.assertEqual(config.transmission.directions, (-1, 1, 1))
        self.assertEqual(config.transmission.belt_reduction, 2.22)
        self.assertEqual(config.mechanism.H, 85.0)
        self.assertEqual(config.mechanism.D, 80.0)
        self.assertEqual(config.mechanism.alpha1_deg, 45.0)
        self.assertEqual(config.mechanism.alpha2_deg, 60.0)
        self.assertEqual(config.mechanism.beta_deg, 45.0)
        self.assertEqual(config.mechanism.motor_heights, (8.0, 15.0, 22.0))
        self.assertEqual(config.calibration.drive_current_a, 0.5)
        self.assertEqual(config.calibration.max_q_current_a, 6.0)
        self.assertEqual(config.calibration.motor_kv, 127.2)
        self.assertEqual(config.calibration.firmware_max_current_a, 2.0)
        self.assertEqual(config.calibration.current_limit_scale, 0.25)
        self.assertEqual(config.safety.velocity_rpy_max_deg_s, 10.0)
        self.assertEqual(config.safety.velocity_motor_max_rev_s, 0.5)

    def test_round_trip_and_atomic_replace(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "nested" / "config.json"
            config = AppConfig()
            config.transmission = dataclasses.replace(
                config.transmission, motor_reduction=7.5, directions=(1, -1, 1))
            save_config(config, path)
            loaded = load_config(path)
            self.assertEqual(loaded.to_dict(), config.to_dict())
            save_config(AppConfig(), path)
            self.assertEqual(load_config(path).transmission.motor_reduction, 8.0)

    def test_missing_file_uses_defaults(self):
        with tempfile.TemporaryDirectory() as directory:
            config = load_config(Path(directory) / "missing.json")
            self.assertEqual(config, AppConfig())

    def test_bad_json_and_unknown_key(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.json"
            path.write_text("{bad", encoding="utf-8")
            with self.assertRaises(ConfigError):
                load_config(path)
            path.write_text(json.dumps({"mechanism": {"H": 150, "typo": 1}}), encoding="utf-8")
            with self.assertRaises(ConfigError):
                load_config(path)

    def test_invalid_ratios_ids_limits(self):
        with self.assertRaises(ConfigError):
            MechanismParams(H=10).validate()
        config = AppConfig()
        config.transmission.motor_reduction = 0
        with self.assertRaises(ConfigError):
            config.validate()
        config = AppConfig()
        config.transmission.motor_ids = (1, 1, 2)
        with self.assertRaises(ConfigError):
            config.validate()
        config = AppConfig()
        config.safety.motor_min_deg = (1, -1, -1)
        with self.assertRaises(ConfigError):
            config.validate()
        config = AppConfig()
        config.calibration.max_q_current_a = 0.4
        with self.assertRaises(ConfigError):
            config.validate()


if __name__ == "__main__":
    unittest.main()
