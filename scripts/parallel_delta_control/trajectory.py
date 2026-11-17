"""Synchronized multi-axis quintic trajectories in raw moteus revolutions."""

from __future__ import annotations

import math
import dataclasses
from dataclasses import dataclass
from typing import Iterable

import numpy as np


# Maximum derivative and second derivative of 10u^3-15u^4+6u^5.
_QUINTIC_MAX_D1 = 1.875
_QUINTIC_MAX_D2 = 10.0 / math.sqrt(3.0)


@dataclass
class SynchronizedTrajectory:
    start: np.ndarray
    goal: np.ndarray
    start_time_s: float
    duration_s: float

    @classmethod
    def between(cls, start: Iterable[float], goal: Iterable[float], now_s: float,
                velocity_limit_rev_s: float, accel_limit_rev_s2: float,
                min_duration_s: float = 0.05,
                duration_s: float | None = None) -> "SynchronizedTrajectory":
        start_array = np.asarray(list(start), dtype=float).reshape(3)
        goal_array = np.asarray(list(goal), dtype=float).reshape(3)
        if not np.all(np.isfinite(start_array)) or not np.all(np.isfinite(goal_array)):
            raise ValueError("轨迹起点和终点必须有限")
        if velocity_limit_rev_s <= 0 or accel_limit_rev_s2 <= 0:
            raise ValueError("轨迹速度和加速度上限必须为正")
        distance = float(np.max(np.abs(goal_array - start_array)))
        velocity_duration = _QUINTIC_MAX_D1 * distance / velocity_limit_rev_s
        accel_duration = math.sqrt(_QUINTIC_MAX_D2 * distance / accel_limit_rev_s2)
        minimum_duration = max(min_duration_s, velocity_duration, accel_duration)
        if duration_s is None:
            duration = minimum_duration
        else:
            duration = float(duration_s)
            if not math.isfinite(duration) or duration <= 0:
                raise ValueError("目标到达时间 duration_s 必须是有限正数（秒）")
            if duration + 1e-9 < minimum_duration:
                raise ValueError(
                    f"目标到达时间 {duration:.9f} s 过短：当前位移在速度/加速度上限下"
                    f"至少需要 {minimum_duration:.9f} s")
        return cls(start_array, goal_array, float(now_s), duration)

    def shift_start(self, now_s: float) -> "SynchronizedTrajectory":
        return dataclasses.replace(self, start_time_s=float(now_s))

    def sample(self, now_s: float) -> tuple[np.ndarray, np.ndarray, bool, float]:
        """Return position, velocity, done, and normalized time in [0,1]."""
        if self.duration_s <= 0:
            return self.goal.copy(), np.zeros(3), True, 1.0
        u = float(np.clip((now_s - self.start_time_s) / self.duration_s, 0.0, 1.0))
        progress = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
        derivative = (30.0 * u**2 - 60.0 * u**3 + 30.0 * u**4) / self.duration_s
        delta = self.goal - self.start
        return (self.start + progress * delta, derivative * delta, u >= 1.0, u)


@dataclass
class SplineSegment:
    """One cubic Hermite segment in physical units."""

    start: np.ndarray
    goal: np.ndarray
    start_time_s: float
    duration_s: float
    start_velocity: np.ndarray
    goal_velocity: np.ndarray

    def coefficients(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        h = self.duration_s
        a = 2.0 * self.start - 2.0 * self.goal + h * (self.start_velocity + self.goal_velocity)
        b = -3.0 * self.start + 3.0 * self.goal - h * (2.0 * self.start_velocity + self.goal_velocity)
        c = h * self.start_velocity
        return a, b, c, self.start.copy()

    def sample(self, now_s: float) -> tuple[np.ndarray, np.ndarray, bool, float]:
        u = float(np.clip((now_s - self.start_time_s) / self.duration_s, 0.0, 1.0))
        a, b, c, d = self.coefficients()
        position = ((a * u + b) * u + c) * u + d
        velocity = (3.0 * a * u**2 + 2.0 * b * u + c) / self.duration_s
        return position, velocity, u >= 1.0, u

    def maxima(self) -> tuple[float, float]:
        """Exact per-axis maxima: |v| is quadratic, |a| is linear."""
        a, b, c, _ = self.coefficients()
        candidates_u = [np.zeros(3), np.ones(3)]
        for index in range(3):
            if abs(a[index]) > 1e-15:
                u = -b[index] / (3.0 * a[index])
                if 0.0 < u < 1.0:
                    candidates_u.append(np.array([u, u, u]))
        max_velocity = 0.0
        max_accel = 0.0
        for u in candidates_u:
            velocity = (3.0 * a * u**2 + 2.0 * b * u + c) / self.duration_s
            acceleration = (6.0 * a * u + 2.0 * b) / self.duration_s**2
            max_velocity = max(max_velocity, float(np.max(np.abs(velocity))))
            max_accel = max(max_accel, float(np.max(np.abs(acceleration))))
        return max_velocity, max_accel


def _spline_derivatives(points: np.ndarray, durations: np.ndarray) -> np.ndarray:
    """Solve clamped cubic-spline first derivatives for all three axes."""
    count = len(points)
    matrix = np.zeros((count, count))
    rhs = np.zeros((count, 3))
    matrix[0, 0] = 1.0
    matrix[-1, -1] = 1.0
    for index in range(1, count - 1):
        h0 = float(durations[index - 1])
        h1 = float(durations[index])
        matrix[index, index - 1] = h1
        matrix[index, index] = 2.0 * (h0 + h1)
        matrix[index, index + 1] = h0
        rhs[index] = 3.0 * (
            h1 / h0 * (points[index] - points[index - 1]) +
            h0 / h1 * (points[index + 1] - points[index]))
    return np.linalg.solve(matrix, rhs)


@dataclass
class WaypointTrajectory:
    """Continuous clamped cubic spline through all waypoints."""

    segments: list[SplineSegment]
    duration_s: float

    @classmethod
    def between(cls, waypoints: Iterable[Iterable[float]], now_s: float,
                velocity_limit_rev_s: float, accel_limit_rev_s2: float,
                duration_s: float | None = None) -> "WaypointTrajectory":
        points = [np.asarray(list(point), dtype=float).reshape(3)
                  for point in waypoints]
        if len(points) < 2 or not all(np.all(np.isfinite(point)) for point in points):
            raise ValueError("waypoint 轨迹至少需要两个有限的三轴点")
        # Remove consecutive duplicates; they add no geometric segment.
        points = [points[0]] + [point for previous, point in zip(points[:-1], points[1:])
                                if np.max(np.abs(point - previous)) > 1e-12]
        if len(points) == 1:
            total = 0.05 if duration_s is None else float(duration_s)
            if not math.isfinite(total) or total <= 0:
                raise ValueError("目标到达时间 duration_s 必须是有限正数（秒）")
            segment = SplineSegment(points[0], points[0], float(now_s), total,
                                    np.zeros(3), np.zeros(3))
            return cls([segment], total)
        points_array = np.stack(points)
        distances = np.max(np.abs(np.diff(points_array, axis=0)), axis=1)
        durations = distances / float(np.sum(distances))
        derivatives = _spline_derivatives(points_array, durations)

        def make_segments(scale: float) -> list[SplineSegment]:
            result = []
            started = float(now_s)
            for index in range(len(points_array) - 1):
                duration = float(durations[index] * scale)
                result.append(SplineSegment(
                    start=points_array[index].copy(),
                    goal=points_array[index + 1].copy(),
                    start_time_s=started, duration_s=duration,
                    start_velocity=derivatives[index] / scale,
                    goal_velocity=derivatives[index + 1] / scale))
                started += duration
            return result

        base_segments = make_segments(1.0)
        max_velocity = max(segment.maxima()[0] for segment in base_segments)
        max_accel = max(segment.maxima()[1] for segment in base_segments)
        scale = max(max_velocity / velocity_limit_rev_s,
                    math.sqrt(max_accel / accel_limit_rev_s2),
                    0.05 / float(np.sum(durations)))
        minimum_total = float(np.sum(durations)) * scale
        if duration_s is None:
            total = minimum_total
        else:
            total = float(duration_s)
            if not math.isfinite(total) or total <= 0:
                raise ValueError("目标到达时间 duration_s 必须是有限正数（秒）")
            if total + 1e-9 < minimum_total:
                raise ValueError(
                    f"目标到达时间 {total:.9f} s 过短：连续样条轨迹在速度/加速度上限下"
                    f"至少需要 {minimum_total:.9f} s")
        final_scale = total / float(np.sum(durations))
        return cls(make_segments(final_scale), total)

    def shift_start(self, now_s: float) -> "WaypointTrajectory":
        offset = float(now_s) - self.segments[0].start_time_s
        return WaypointTrajectory(
            [dataclasses.replace(segment, start_time_s=segment.start_time_s + offset)
             for segment in self.segments], self.duration_s)

    def sample(self, now_s: float) -> tuple[np.ndarray, np.ndarray, bool, float]:
        for index, segment in enumerate(self.segments):
            if now_s <= segment.start_time_s + segment.duration_s:
                positions, velocities, segment_done, progress = segment.sample(now_s)
                done = segment_done and index == len(self.segments) - 1
                return (positions, velocities, done,
                        (index + progress) / len(self.segments))
        return (self.segments[-1].goal.copy(), np.zeros(3), True, 1.0)
