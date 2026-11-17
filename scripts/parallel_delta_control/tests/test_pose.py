import math
import unittest

import numpy as np

from pose import (SphericalPose, matrix_to_rpy, matrix_to_spherical,
                  rpy_to_matrix, spherical_to_matrix)


class PoseTests(unittest.TestCase):
    def test_identity(self):
        np.testing.assert_allclose(rpy_to_matrix([0, 0, 0]), np.eye(3), atol=1e-12)
        np.testing.assert_allclose(matrix_to_rpy(np.eye(3)), np.zeros(3), atol=1e-12)
        pose = matrix_to_spherical(np.eye(3))
        self.assertAlmostEqual(pose.theta, 0.0)
        self.assertAlmostEqual(pose.phi, 0.0)
        self.assertAlmostEqual(pose.psi, 0.0)

    def test_known_yaw(self):
        R = rpy_to_matrix([0, 0, 90])
        np.testing.assert_allclose(R @ np.array([1, 0, 0]), [0, 1, 0], atol=1e-12)

    def test_rpy_spherical_round_trip(self):
        for roll in (-15, 0, 15):
            for pitch in (-15, 0, 15):
                for yaw in (-60, 0, 60):
                    with self.subTest(rpy=(roll, pitch, yaw)):
                        R = rpy_to_matrix([roll, pitch, yaw])
                        pose = matrix_to_spherical(R)
                        reconstructed = spherical_to_matrix(pose)
                        np.testing.assert_allclose(reconstructed, R, atol=1e-12)
                        np.testing.assert_allclose(matrix_to_rpy(reconstructed),
                                                   [roll, pitch, yaw], atol=1e-10)

    def test_invalid_input(self):
        with self.assertRaises(ValueError):
            rpy_to_matrix([0, math.nan, 0])
        with self.assertRaises(ValueError):
            rpy_to_matrix([0, 0])


if __name__ == "__main__":
    unittest.main()
