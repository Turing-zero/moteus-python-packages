import unittest

import numpy as np

from trajectory import SynchronizedTrajectory, WaypointTrajectory


class TrajectoryTests(unittest.TestCase):
    def test_quintic_endpoints_and_limits(self):
        start = np.array([1.0, -2.0, 0.5])
        goal = np.array([2.0, -2.5, 1.0])
        trajectory = SynchronizedTrajectory.between(
            start, goal, 10.0, velocity_limit_rev_s=0.5,
            accel_limit_rev_s2=2.0)
        positions, velocities, done, progress = trajectory.sample(10.0)
        np.testing.assert_allclose(positions, start)
        np.testing.assert_allclose(velocities, np.zeros(3))
        self.assertFalse(done)
        self.assertEqual(progress, 0.0)
        positions, velocities, done, progress = trajectory.sample(10.0 + trajectory.duration_s)
        np.testing.assert_allclose(positions, goal)
        np.testing.assert_allclose(velocities, np.zeros(3), atol=1e-12)
        self.assertTrue(done)
        self.assertEqual(progress, 1.0)
        previous = -1.0
        for u in np.linspace(0, 1, 101):
            positions, velocities, _, progress = trajectory.sample(10 + u * trajectory.duration_s)
            self.assertGreaterEqual(progress, previous)
            previous = progress
            normalized = (positions - start) / (goal - start)
            np.testing.assert_allclose(normalized, np.full(3, normalized[0]), atol=1e-12)
        self.assertLessEqual(1.875 / trajectory.duration_s * np.max(np.abs(goal - start)), 0.5 + 1e-9)
        self.assertLessEqual(10 / np.sqrt(3) / trajectory.duration_s**2 * np.max(np.abs(goal - start)), 2.0 + 1e-9)

    def test_requested_duration(self):
        trajectory = SynchronizedTrajectory.between(
            [0, 0, 0], [1, 0.5, -0.25], 10.0,
            velocity_limit_rev_s=0.5, accel_limit_rev_s2=2.0,
            duration_s=6.0)
        self.assertEqual(trajectory.duration_s, 6.0)
        positions, _, done, _ = trajectory.sample(13.0)
        np.testing.assert_allclose(positions, [0.5, 0.25, -0.125])
        self.assertFalse(done)
        _, velocities, done, _ = trajectory.sample(16.0)
        np.testing.assert_allclose(velocities, np.zeros(3), atol=1e-12)
        self.assertTrue(done)
        with self.assertRaisesRegex(ValueError, "至少需要"):
            SynchronizedTrajectory.between(
                [0, 0, 0], [1, 0, 0], 0.0, 0.5, 2.0, duration_s=1.0)
        for invalid in (0.0, -1.0, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                SynchronizedTrajectory.between(
                    [0, 0, 0], [0, 0, 0], 0.0, 0.5, 2.0,
                    duration_s=invalid)

    def test_exact_minimum_duration_is_accepted(self):
        single = SynchronizedTrajectory.between(
            [0, 0, 0], [1, 0, 0], 0.0, 1.0, 2.0)
        exact_single = SynchronizedTrajectory.between(
            [0, 0, 0], [1, 0, 0], 0.0, 1.0, 2.0,
            duration_s=single.duration_s)
        self.assertEqual(exact_single.duration_s, single.duration_s)
        waypoint = WaypointTrajectory.between(
            [[0, 0, 0], [1, 0, 0], [0, 0, 0]], 0.0, 1.0, 2.0)
        exact_waypoint = WaypointTrajectory.between(
            [[0, 0, 0], [1, 0, 0], [0, 0, 0]], 0.0, 1.0, 2.0,
            duration_s=waypoint.duration_s)
        self.assertEqual(exact_waypoint.duration_s, waypoint.duration_s)

    def test_zero_distance(self):
        trajectory = SynchronizedTrajectory.between([0, 0, 0], [0, 0, 0], 0, 1, 1)
        position, velocity, done, _ = trajectory.sample(trajectory.duration_s)
        np.testing.assert_allclose(position, np.zeros(3))
        np.testing.assert_allclose(velocity, np.zeros(3))
        self.assertTrue(done)

    def test_waypoint_total_duration_and_middle_point(self):
        trajectory = WaypointTrajectory.between(
            [[0, 0, 0], [1, 1, 1], [0, 0, 0]], 10.0,
            velocity_limit_rev_s=1.0, accel_limit_rev_s2=2.0,
            duration_s=10.0)
        self.assertEqual(trajectory.duration_s, 10.0)
        first = trajectory.segments[0]
        positions, velocities, done, _ = trajectory.sample(
            10.0 + first.duration_s)
        np.testing.assert_allclose(positions, [1, 1, 1])
        np.testing.assert_allclose(velocities, np.zeros(3), atol=1e-12)
        self.assertFalse(done)
        positions, velocities, done, progress = trajectory.sample(20.0)
        np.testing.assert_allclose(positions, np.zeros(3))
        np.testing.assert_allclose(velocities, np.zeros(3), atol=1e-12)
        self.assertTrue(done)
        self.assertEqual(progress, 1.0)
        with self.assertRaisesRegex(ValueError, "过短"):
            WaypointTrajectory.between(
                [[0, 0, 0], [1, 1, 1], [0, 0, 0]], 0.0,
                1.0, 2.0, duration_s=0.1)

    def test_waypoint_is_smooth_without_middle_stop(self):
        trajectory = WaypointTrajectory.between(
            [[1, 0, 0], [0, 0, 0], [-1, 0, 0]], 10.0,
            velocity_limit_rev_s=0.5, accel_limit_rev_s2=2.0)
        first = trajectory.segments[0]
        midpoint = first.start_time_s + first.duration_s
        _, before, _, _ = trajectory.sample(midpoint - 1e-5)
        _, middle, _, _ = trajectory.sample(midpoint)
        _, after, _, _ = trajectory.sample(midpoint + 1e-5)
        self.assertLess(middle[0], -1e-6)  # Moving through the waypoint.
        np.testing.assert_allclose(before, after, atol=1e-6)
        _, initial_velocity, _, _ = trajectory.sample(10.0)
        _, final_velocity, done, _ = trajectory.sample(10.0 + trajectory.duration_s)
        np.testing.assert_allclose(initial_velocity, np.zeros(3), atol=1e-12)
        np.testing.assert_allclose(final_velocity, np.zeros(3), atol=1e-12)
        self.assertTrue(done)
        for segment in trajectory.segments:
            a, b, _, _ = segment.coefficients()
            for u in np.linspace(0.0, 1.0, 201):
                _, velocity, _, _ = segment.sample(
                    segment.start_time_s + segment.duration_s * u)
                acceleration = (6.0 * a * u + 2.0 * b) / segment.duration_s**2
                self.assertLessEqual(np.max(np.abs(velocity)), 0.5 + 1e-9)
                self.assertLessEqual(np.max(np.abs(acceleration)), 2.0 + 1e-9)

    def test_invalid(self):
        with self.assertRaises(ValueError):
            SynchronizedTrajectory.between([0, 0, 0], [1, 1, 1], 0, 0, 1)


if __name__ == "__main__":
    unittest.main()
