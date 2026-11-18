import dataclasses
import unittest

import numpy as np

from kinematics import (KinematicsError, forward_kinematics, home_result,
                        inverse_from_rpy, motor_deg_to_q, motor_deg_to_raw_rev,
                        q_to_motor_deg, raw_rev_to_motor_deg, verify_inverse)
from mechanism import Q_HOME, Q_HOME_DEG
from _support import controlled_config


class KinematicsTests(unittest.TestCase):
    def setUp(self):
        self.config = controlled_config()

    def test_home(self):
        result = home_result(self.config)
        np.testing.assert_allclose(result.rotation, np.eye(3), atol=1e-12)
        np.testing.assert_allclose(np.degrees(result.q), Q_HOME_DEG, atol=1e-12)
        np.testing.assert_allclose(result.motor_deg, np.zeros(3), atol=1e-12)
        np.testing.assert_allclose(result.rpy_deg, np.zeros(3), atol=1e-10)

    def test_clockwise_outputs_and_counterclockwise_links(self):
        geometry = home_result(self.config).geometry
        expected_axis = [30.0, 270.0, 150.0]
        expected_link = [210.0, 90.0, 330.0]
        expected_platform = [300.0, 180.0, 60.0]

        def azimuth(vector):
            return np.degrees(np.arctan2(vector[1], vector[0])) % 360.0

        def assert_azimuth(vector, expected):
            error = (azimuth(vector) - expected + 180.0) % 360.0 - 180.0
            self.assertAlmostEqual(error, 0.0, places=10)

        for index in range(3):
            assert_azimuth(geometry["w"][index], expected_axis[index])
            for point in (geometry["roots"][index], geometry["Bw"][index]):
                assert_azimuth(point, expected_link[index])
                signed_step = (azimuth(geometry["Bv"][index]) - azimuth(point) + 180.0) % 360.0 - 180.0
                self.assertAlmostEqual(signed_step, 90.0, places=10)
            assert_azimuth(geometry["Bv"][index], expected_platform[index])
        output_angles = [azimuth(point) for point in geometry["roots"]]
        for index in range(3):
            output_step = (output_angles[(index + 1) % 3] - output_angles[index] + 180.0) % 360.0 - 180.0
            self.assertAlmostEqual(output_step, -120.0, places=10)
        np.testing.assert_allclose(geometry["heights"], [8.0, 15.0, 22.0])
        np.testing.assert_allclose(geometry["roots"][:, 2], [8.0, 15.0, 22.0])

    def test_motor_and_transmission_mapping(self):
        config = controlled_config()
        np.testing.assert_allclose(motor_deg_to_q([160, -160, 320], config.transmission),
                                   Q_HOME + np.radians([10, -10, 20]))
        config.transmission = dataclasses.replace(
            config.transmission, motor_reduction=4.0, belt_reduction=3.0)
        np.testing.assert_allclose(
            q_to_motor_deg(motor_deg_to_q([120, -240, 360], config.transmission),
                           config.transmission), [120, -240, 360])

    def test_raw_zero_direction_and_multiturn(self):
        zero = np.array([10.25, -3.5, 0.75])
        directions = np.array([1.0, -1.0, 1.0])
        motor = np.array([1080.0, -720.0, 3600.0])
        raw = motor_deg_to_raw_rev(motor, zero, directions)
        np.testing.assert_allclose(raw_rev_to_motor_deg(raw, zero, directions), motor)

    def test_inverse_forward_round_trip(self):
        for rpy in ([0, 0, 0], [0, 0, 60], [0, 0, -60],
                    [15, 15, 60], [-15, -15, -60], [15, -15, 60], [5, 10, -30]):
            with self.subTest(rpy=rpy):
                inverse = inverse_from_rpy(self.config, rpy)
                forward = verify_inverse(self.config, inverse, [0, 0, 0], np.eye(3))
                np.testing.assert_allclose(forward.rotation, inverse.rotation, atol=1e-9)
                np.testing.assert_allclose(forward.rpy_deg, rpy, atol=1e-8)

    def test_large_yaw_uses_path_continuation(self):
        forward = forward_kinematics(self.config, [960, 960, 960])
        np.testing.assert_allclose(forward.rpy_deg, [0, 0, 60], atol=1e-8)
        self.assertGreater(forward.path_steps, 20)

    def test_workspace_grid(self):
        worst = 0.0
        count = 0
        for roll in np.linspace(-15, 15, 7):
            for pitch in np.linspace(-15, 15, 7):
                for yaw in np.linspace(-60, 60, 13):
                    inverse = inverse_from_rpy(self.config, [roll, pitch, yaw])
                    forward = verify_inverse(self.config, inverse, [0, 0, 0], np.eye(3))
                    worst = max(worst, float(np.max(np.abs(forward.rotation - inverse.rotation))))
                    self.assertGreaterEqual(forward.condition, self.config.safety.min_condition)
                    count += 1
        self.assertEqual(count, 637)
        self.assertLess(worst, 1e-8)

    def test_invalid_and_out_of_range(self):
        with self.assertRaises(KinematicsError):
            forward_kinematics(self.config, [0, np.nan, 0])
        with self.assertRaises(KinematicsError):
            inverse_from_rpy(self.config, [16, 0, 0])
        with self.assertRaises(KinematicsError):
            inverse_from_rpy(self.config, [0, 0, 61])

    def test_feedback_boundary_margin(self):
        inverse = inverse_from_rpy(
            self.config, [10, 10, 60.173115], enforce_limits=False)
        with self.assertRaises(KinematicsError):
            forward_kinematics(self.config, inverse.motor_deg)
        feedback = forward_kinematics(
            self.config, inverse.motor_deg,
            rpy_margin_deg=self.config.safety.feedback_rpy_margin_deg)
        self.assertAlmostEqual(feedback.rpy_deg[2], 60.173115, places=6)
        with self.assertRaises(KinematicsError):
            inverse_from_rpy(self.config, [10, 10, 60.01])

    def test_boundary_move_allows_numerical_path_overshoot(self):
        # Regression: nonlinear coupling makes the continuation path reach
        # pitch=15.602916 even though both requested endpoints are legal.
        start = inverse_from_rpy(self.config, [15, 15, 0])
        end = inverse_from_rpy(self.config, [0, 15, 0], start.motor_deg)
        result = verify_inverse(self.config, end, start.motor_deg, start.rotation)
        np.testing.assert_allclose(result.rpy_deg, [0, 15, 0], atol=1e-8)

    def test_pure_yaw_motor_travel(self):
        inverse = inverse_from_rpy(self.config, [0, 0, 60])
        np.testing.assert_allclose(inverse.motor_deg, [960, 960, 960], atol=1e-8)


if __name__ == "__main__":
    unittest.main()
