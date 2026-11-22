import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
REPO_MOTEUS = PROJECT.parent.parent / "moteus"
sys.path.insert(0, str(PROJECT))
sys.path.insert(0, str(REPO_MOTEUS))

from fastapi.testclient import TestClient

from _support import fake_controller
from server import create_app, telemetry_period_s


class ApiTests(unittest.TestCase):
    def setUp(self):
        self.controller = fake_controller()
        self.temp = tempfile.TemporaryDirectory()
        self.client = TestClient(create_app(self.controller, Path(self.temp.name) / "config.json"))

    def tearDown(self):
        self.client.close()
        self.temp.cleanup()

    def test_state_and_calibration(self):
        response = self.client.get("/api/state")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()["state"], "uncalibrated")
        response = self.client.post("/api/calibrate")
        self.assertEqual(response.status_code, 200)
        self.assertTrue(response.json()["calibrated"])
        self.assertTrue(response.json()["target"]["valid"])

    def test_forward_and_inverse(self):
        self.client.post("/api/calibrate")
        response = self.client.post("/api/command/rpy", json={"rpy_deg": [0, 0, 10]})
        self.assertEqual(response.status_code, 200)
        np.testing.assert_allclose(response.json()["target"]["motor_deg"], [160, 160, 160], atol=1e-7)
        response = self.client.post("/api/command/motors", json={"motor_deg": [16, 32, -16]})
        self.assertEqual(response.status_code, 200)
        self.assertTrue(response.json()["target"]["valid"])

    def test_command_duration(self):
        self.client.post("/api/calibrate")
        response = self.client.post(
            "/api/command/rpy", json={"rpy_deg": [0, 0, 10], "duration_s": 4.0})
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()["motion"]["duration_s"], 4.0)
        response = self.client.post(
            "/api/command/motors",
            json={"motor_deg": [16, 32, -16], "duration_s": 2.5})
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()["motion"]["duration_s"], 2.5)
        response = self.client.post(
            "/api/command/motors",
            json={"motor_deg": [160, 160, 160], "duration_s": 0.01})
        self.assertEqual(response.status_code, 409)
        self.assertIn("至少需要", response.json()["detail"])
        response = self.client.post(
            "/api/command/rpy", json={"rpy_deg": [0, 0, 10], "duration_s": 0})
        self.assertEqual(response.status_code, 409)

    def test_reject_uncalibrated_and_invalid(self):
        response = self.client.post("/api/command/rpy", json={"rpy_deg": [0, 0, 1]})
        self.assertEqual(response.status_code, 409)
        self.client.post("/api/calibrate")
        response = self.client.post("/api/command/rpy", json={"rpy_deg": [0, 0, 61]})
        self.assertEqual(response.status_code, 400)

    def test_stop_hold_clear(self):
        self.client.post("/api/calibrate")
        self.assertEqual(self.client.post("/api/stop").json()["state"], "stopped")
        self.assertTrue(self.client.post("/api/hold").json()["enabled"])
        response = self.client.post("/api/clear")
        self.assertEqual(response.status_code, 200)
        self.assertFalse(response.json()["enabled"])

    def test_dynamic_config_and_save(self):
        config = self.client.get("/api/state").json()["config"]
        config["transmission"]["motor_reduction"] = 4.0
        response = self.client.post("/api/config?save=true", json=config)
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()["config"]["transmission"]["motor_reduction"], 4.0)
        self.assertTrue((Path(self.temp.name) / "config.json").is_file())
        self.assertFalse(response.json()["calibrated"])

    def test_control_assets_require_revalidation(self):
        for path in ("/", "/static/app.js", "/static/style.css"):
            response = self.client.get(path)
            self.assertEqual(response.status_code, 200)
            self.assertIn("no-cache", response.headers["Cache-Control"])
        page = self.client.get("/")
        self.assertIn("motorCommandDuration", page.text)
        self.assertIn("rpyCommandDuration", page.text)
        for element_id in ("motorCommandDuration", "rpyCommandDuration"):
            self.assertRegex(
                page.text, rf'id="{element_id}"[^>]*min="0\.05"[^>]*step="any"')
        for element_id in ("rpySpeed0", "rpySpeed1", "rpySpeed2", "rpySpeedZero"):
            self.assertIn(element_id, page.text)
        self.assertIn('min="-10" max="10"', page.text)
        script = self.client.get("/static/app.js")
        self.assertIn("duration_s: duration", script.text)

    def test_extreme_calibration_endpoint(self):
        response = self.client.post("/api/calibrate/extreme")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()["state"], "calibrating")
        self.assertTrue(response.json()["extreme_calibration"]["active"])
        self.assertEqual(self.client.post("/api/calibrate/extreme").status_code, 409)
        response = self.client.post("/api/stop")
        self.assertEqual(response.status_code, 200)
        self.assertFalse(response.json()["extreme_calibration"]["active"])

    def test_velocity_endpoints_and_stop(self):
        self.client.post("/api/calibrate")
        response = self.client.post(
            "/api/command/velocity/motors", json={"motor_rev_s": [0.1, 0.2, 0.3]})
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()["velocity"]["kind"], "motor")
        self.assertEqual(response.json()["state"], "moving")
        response = self.client.post(
            "/api/command/velocity/rpy", json={"rpy_deg_s": [5, 0, 0]})
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()["velocity"]["kind"], "rpy")
        response = self.client.post("/api/stop")
        self.assertFalse(response.json()["velocity"]["active"])

    def test_telemetry_rate_is_capped(self):
        self.assertEqual(telemetry_period_s(100.0), 0.05)
        self.assertEqual(telemetry_period_s(20.0), 0.05)
        self.assertEqual(telemetry_period_s(10.0), 0.1)

    def test_websocket(self):
        with self.client.websocket_connect("/api/ws") as socket:
            state = socket.receive_json()
            self.assertIn("target", state)
            self.assertIn("actual", state)


if __name__ == "__main__":
    unittest.main()
