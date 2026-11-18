import asyncio
import dataclasses
import sys
import threading
import time
import unittest
from unittest import mock
from pathlib import Path

import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
REPO_MOTEUS = PROJECT.parent.parent / "moteus"
sys.path.insert(0, str(PROJECT))
sys.path.insert(0, str(REPO_MOTEUS))

from moteus.protocol import Mode, Register

from _support import fake_controller, feed
from controller import ControlError, MechanismController
from kinematics import (inverse_from_rpy, motor_deg_to_raw_rev, KinematicsError,
                        forward_kinematics)
from params import AppConfig
from trajectory import WaypointTrajectory


def extreme_sample(controller, positions, stage, current=1.0):
    sign = 1.0 if stage == "positive" else -1.0
    currents = sign * abs(current) * controller.directions
    feed(controller, positions, mode=Mode.POSITION, q_currents=currents)
    controller._service_extreme_calibration(time.monotonic())


class ControllerTests(unittest.TestCase):
    def test_uncalibrated_motion_rejected(self):
        controller = fake_controller()
        with self.assertRaises(ControlError):
            controller.command_rpy([0, 0, 0])
        with self.assertRaises(ControlError):
            controller.hold()

    def test_calibration_and_inverse_command(self):
        controller = fake_controller()
        controller.calibrate()
        self.assertTrue(controller.calibrated)
        self.assertTrue(controller.enabled)
        np.testing.assert_allclose(controller.snapshot()["actual"]["rpy_deg"], [0, 0, 0], atol=1e-10)
        result = controller.command_rpy([0, 0, 10])
        np.testing.assert_allclose(result.motor_deg, [160, 160, 160], atol=1e-8)
        expected = motor_deg_to_raw_rev(result.motor_deg, controller.zero_rev, controller.directions)
        np.testing.assert_allclose([controller.motors[i].raw_target_rev for i in controller.ids], expected)

    def test_forward_command(self):
        controller = fake_controller()
        controller.calibrate()
        result = controller.command_motors([16, 32, -16])
        np.testing.assert_allclose(np.degrees(result.q), [31, 272, 149], atol=1e-8)
        self.assertTrue(np.all(np.isfinite(result.rpy_deg)))

    def test_requested_duration_and_atomic_rejection(self):
        controller = fake_controller()
        controller.calibrate()
        controller.command_motors([16, 32, -16], duration_s=4.0)
        self.assertEqual(controller._trajectory.duration_s, 4.0)
        self.assertEqual(controller.snapshot()["motion"]["duration_s"], 4.0)
        before_targets = [controller.motors[i].raw_target_rev for i in controller.ids]
        before_trajectory = controller._trajectory
        before_target_result = controller.target_result
        with self.assertRaisesRegex(ControlError, "至少需要"):
            controller.command_motors([160, 160, 160], duration_s=0.01)
        self.assertEqual(
            before_targets, [controller.motors[i].raw_target_rev for i in controller.ids])
        self.assertIs(controller._trajectory, before_trajectory)
        self.assertIs(controller.target_result, before_target_result)

    def test_invalid_command_is_atomic(self):
        controller = fake_controller()
        controller.calibrate()
        controller.command_rpy([0, 0, 2])
        before = [controller.motors[i].raw_target_rev for i in controller.ids]
        with self.assertRaises(ControlError):
            controller.command_motors([5000, 0, 0])
        after = [controller.motors[i].raw_target_rev for i in controller.ids]
        self.assertEqual(before, after)

    def test_stop_then_hold(self):
        controller = fake_controller()
        controller.calibrate()
        controller.stop()
        self.assertEqual(controller.snapshot()["state"], "stopped")
        with self.assertRaises(ControlError):
            controller.command_rpy([0, 0, 1])
        controller.hold()
        self.assertTrue(controller.enabled)

    def test_feedback_timeout_stops_group(self):
        controller = fake_controller()
        controller.calibrate()
        for motor in controller.motors.values():
            motor.last_update = time.monotonic() - 10.0
        controller._apply_results({}, time.monotonic())
        self.assertFalse(controller.enabled)
        self.assertIsNotNone(controller.fault)
        self.assertEqual(controller.snapshot()["state"], "fault")

    def test_feedback_jump_invalidates_zero(self):
        controller = fake_controller()
        controller.calibrate()
        controller._previous_feedback_time = time.monotonic() - 0.125
        feed(controller, [20.0, -3.5, 0.75], mode=Mode.POSITION)
        self.assertFalse(controller.calibrated)
        self.assertFalse(controller.enabled)
        self.assertIsNotNone(controller.fault)
        self.assertIn("电机 1", controller.fault)
        self.assertIn("反馈间隔", controller.fault)
        diagnostic = controller.snapshot()["last_feedback_jump"]
        self.assertEqual(diagnostic["motor_id"], 1)
        self.assertAlmostEqual(diagnostic["step_deg"], 3510.0, places=6)
        self.assertEqual(diagnostic["threshold_deg"], 30.0)
        self.assertGreaterEqual(diagnostic["elapsed_ms"], 120.0)

    def test_fault_102_is_global_warning(self):
        controller = fake_controller()
        controller.calibrate()
        results = {}
        for motor_id in controller.ids:
            results[motor_id] = {
                Register.POSITION: controller.motors[motor_id].raw_position_rev,
                Register.MODE: int(Mode.POSITION), Register.FAULT: 102,
            }
        with self.assertLogs("parallel_delta_control", level="WARNING") as logs:
            controller._apply_results(results, time.monotonic())
            controller._apply_results(results, time.monotonic())
        self.assertIsNone(controller.fault)
        self.assertTrue(controller.enabled)
        self.assertEqual(len(logs.output), 3)  # One warning per motor, no 100 Hz spam.

    def test_feedback_fault_stops_group(self):
        controller = fake_controller()
        controller.calibrate()
        results = {}
        for motor_id in controller.ids:
            results[motor_id] = {
                Register.POSITION: controller.motors[motor_id].raw_position_rev,
                Register.MODE: int(Mode.FAULT), Register.FAULT: 42,
            }
        controller._apply_results(results, time.monotonic())
        self.assertFalse(controller.enabled)
        self.assertIn("42", controller.fault)
    def test_feedback_boundary_overshoot_does_not_latch_fault(self):
        controller = fake_controller()
        controller.calibrate()
        overshoot = inverse_from_rpy(
            controller.config, [10, 10, 60.173115], enforce_limits=False)
        for alpha in np.linspace(0.0, 1.0, 101)[1:]:
            motor_deg = overshoot.motor_deg * alpha
            raw = motor_deg_to_raw_rev(
                motor_deg, controller.zero_rev, controller.directions)
            feed(controller, raw, mode=Mode.POSITION)
        self.assertIsNone(controller.fault)
        self.assertTrue(controller.calibrated)
        self.assertAlmostEqual(controller.actual_result.rpy_deg[2], 60.173115, places=5)
        controller.stop()
        controller.hold()
        self.assertTrue(controller.enabled)
    def test_inverse_singular_transition_uses_validated_home_waypoint(self):
        controller = fake_controller()
        controller.config.safety.max_feedback_step_deg = 5000.0
        controller.calibrate()
        controller.command_rpy([3, 3, 45])
        feed(controller,
             [controller.motors[i].raw_target_rev for i in controller.ids],
             mode=Mode.POSITION)
        result = controller.command_rpy([-3, -3, -45])
        self.assertIsNone(controller.fault)
        self.assertIsInstance(controller._trajectory, WaypointTrajectory)
        self.assertEqual(len(controller._trajectory.segments), 2)
        np.testing.assert_allclose(result.rpy_deg, [-3, -3, -45], atol=1e-8)
        self.assertIn("直立点", controller.message)

    def test_spline_validation_has_bounded_outer_samples(self):
        controller = fake_controller()
        controller.calibrate()
        target = inverse_from_rpy(controller.config, [-3, -3, -45])
        raw_target = motor_deg_to_raw_rev(
            target.motor_deg, controller.zero_rev, controller.directions)
        trajectory = WaypointTrajectory.between(
            [controller._raw_positions(), raw_target], time.monotonic(),
            controller.config.safety.velocity_limit_rev_s,
            controller.config.safety.accel_limit_rev_s2)
        with mock.patch("controller.forward_kinematics", wraps=forward_kinematics) as fk:
            controller._validate_waypoint_path(trajectory)
        self.assertEqual(fk.call_count, 20)

    def test_initial_inverse_branch_failure_also_uses_home(self):
        controller = fake_controller()
        controller.calibrate()
        real_inverse = inverse_from_rpy
        calls = {"count": 0}

        def fail_first_branch(*args, **kwargs):
            calls["count"] += 1
            if calls["count"] == 1:
                raise KinematicsError("目标接近奇异：条件指标 0.0291 < 0.0500")
            return real_inverse(*args, **kwargs)

        with mock.patch("controller.inverse_from_rpy", side_effect=fail_first_branch):
            result = controller.command_rpy([-3, -3, -45])
        self.assertIsNone(controller.fault)
        self.assertIsInstance(controller._trajectory, WaypointTrajectory)
        np.testing.assert_allclose(result.rpy_deg, [-3, -3, -45], atol=1e-8)
        self.assertGreaterEqual(calls["count"], 3)

    def test_extreme_calibration_success_and_return(self):
        controller = fake_controller()
        controller.config.safety.max_feedback_step_deg = 5000.0
        clock = [time.monotonic()]
        positive = np.array([10.25, -3.5, 0.75]) + 8.0 / 3.0
        negative = np.array([10.25, -3.5, 0.75]) - 8.0 / 3.0
        with mock.patch("controller.time.monotonic", side_effect=lambda: clock[0]):
            controller.start_extreme_calibration()
            with self.assertRaises(ControlError):
                controller.command_motors([0, 0, 0])
            clock[0] += 0.1
            extreme_sample(controller, positive, "positive")
            clock[0] += 1.2
            extreme_sample(controller, positive, "positive")
            clock[0] += 1.2
            extreme_sample(controller, positive, "positive")
            self.assertEqual(controller._extreme_calibration.stage, "negative")
            clock[0] += 0.1
            extreme_sample(controller, negative, "negative")
            clock[0] += 1.2
            extreme_sample(controller, negative, "negative")
            clock[0] += 1.2
            extreme_sample(controller, negative, "negative")
            self.assertEqual(controller._extreme_calibration.stage, "return")
            np.testing.assert_allclose(
                controller.zero_rev, [10.25, -3.5, 0.75])
            clock[0] += controller._trajectory.duration_s + 0.1
            controller._motion_command()
            feed(controller, [10.25, -3.5, 0.75], mode=Mode.POSITION)
            controller._service_extreme_calibration(clock[0])
            self.assertIsNone(controller._extreme_calibration)
            self.assertIsNone(controller.fault)
            self.assertTrue(controller.calibrated)
            self.assertTrue(controller.enabled)
            self.assertEqual(controller.snapshot()["state"], "ready")
            self.assertAlmostEqual(
                controller.snapshot()["last_extreme_calibration"]["imbalance"], 0.0)

    def test_extreme_calibration_rejects_span_imbalance(self):
        controller = fake_controller()
        clock = [time.monotonic()]
        positive = np.array([13.25, -0.5, 3.75])
        negative = np.array([7.25, -6.5, 1.35])
        with mock.patch("controller.time.monotonic", side_effect=lambda: clock[0]):
            controller.start_extreme_calibration()
            for stage, positions in (("positive", positive), ("negative", negative)):
                clock[0] += 0.1
                extreme_sample(controller, positions, stage)
                clock[0] += 1.2
                extreme_sample(controller, positions, stage)
                clock[0] += 1.2
                extreme_sample(controller, positions, stage)
            self.assertFalse(controller.calibrated)
            self.assertFalse(controller.enabled)
            self.assertIsNone(controller._extreme_calibration)
            self.assertIn("不一致", controller.fault)
            # A later POSITION+102 must not overwrite the real calibration error.
            feed(controller, negative, mode=Mode.POSITION, fault=102)
            controller._apply_results(
                {motor_id: motor.values for motor_id, motor in controller.motors.items()},
                time.monotonic())
            self.assertIn("不一致", controller.fault)

    def test_firmware_velocity_commands_and_expected_limit_102(self):
        controller = fake_controller()
        controller.start_extreme_calibration()
        _, velocities = controller._extreme_drive_commands()
        np.testing.assert_allclose(list(velocities.values()), [0.5, 0.5, 0.5])
        controller._reset_extreme_stage("negative", time.monotonic())
        _, velocities = controller._extreme_drive_commands()
        np.testing.assert_allclose(list(velocities.values()), [-0.5, -0.5, -0.5])

        feed(controller, [10.25, -3.5, 0.75], mode=Mode.POSITION, fault=102)
        controller._apply_results(
            {motor_id: motor.values for motor_id, motor in controller.motors.items()},
            time.monotonic())
        self.assertIsNone(controller.fault)
        self.assertIsNotNone(controller._extreme_calibration)
        feed(controller, [10.25, -3.5, 0.75], mode=Mode.POSITION, fault=99)
        controller._apply_results(
            {motor_id: motor.values for motor_id, motor in controller.motors.items()},
            time.monotonic())
        self.assertIn("99", controller.fault)
        self.assertIsNone(controller._extreme_calibration)

    def test_extreme_calibration_overcurrent_timeout_and_stop(self):
        controller = fake_controller()
        clock = [time.monotonic()]
        with mock.patch("controller.time.monotonic", side_effect=lambda: clock[0]):
            controller.start_extreme_calibration()
            extreme_sample(controller, [10.25, -3.5, 0.75], "positive", current=4.0)
            self.assertIn("电流", controller.fault)
            self.assertIsNone(controller._extreme_calibration)

        controller = fake_controller()
        clock = [time.monotonic()]
        with mock.patch("controller.time.monotonic", side_effect=lambda: clock[0]):
            controller.start_extreme_calibration()
            clock[0] += 21.0
            extreme_sample(controller, [10.25, -3.5, 0.75], "positive")
            self.assertIn("超时", controller.fault)

        controller = fake_controller()
        controller.start_extreme_calibration()
        controller.stop()
        self.assertIsNone(controller._extreme_calibration)
        self.assertFalse(controller.enabled)
        controller.config.calibration.firmware_max_current_a = None
        with self.assertRaisesRegex(ControlError, "servo.max_current_A"):
            controller.start_extreme_calibration()
        controller.config.calibration.firmware_max_current_a = 10.0
        controller.config.calibration.motor_kv = None
        with self.assertRaisesRegex(ControlError, "Kv"):
            controller.start_extreme_calibration()


class ControllerAsyncTests(unittest.IsolatedAsyncioTestCase):
    async def test_async_inverse_planning_does_not_block_event_loop(self):
        controller = fake_controller()
        controller.calibrate()
        entered = threading.Event()
        release = threading.Event()
        real_inverse = inverse_from_rpy

        def blocking_inverse(*args, **kwargs):
            entered.set()
            if not release.wait(2.0):
                raise TimeoutError("test planning release timeout")
            return real_inverse(*args, **kwargs)

        with mock.patch("controller.inverse_from_rpy", side_effect=blocking_inverse):
            task = asyncio.create_task(controller.command_rpy_async([0, 0, 10]))
            try:
                while not entered.is_set():
                    await asyncio.sleep(0.001)
                self.assertFalse(task.done())
                await asyncio.sleep(0.02)  # The event loop remains responsive.
                self.assertFalse(task.done())
            finally:
                release.set()
            result = await task
        np.testing.assert_allclose(result.rpy_deg, [0, 0, 10], atol=1e-8)

    async def test_async_inverse_rejects_feedback_changed_during_planning(self):
        controller = fake_controller()
        controller.calibrate()
        entered = threading.Event()
        release = threading.Event()
        real_inverse = inverse_from_rpy

        def blocking_inverse(*args, **kwargs):
            entered.set()
            if not release.wait(2.0):
                raise TimeoutError("test planning release timeout")
            return real_inverse(*args, **kwargs)

        with mock.patch("controller.inverse_from_rpy", side_effect=blocking_inverse):
            task = asyncio.create_task(controller.command_rpy_async([0, 0, 10]))
            try:
                while not entered.is_set():
                    await asyncio.sleep(0.001)
                feed(controller, [10.255, -3.495, 0.755], mode=Mode.POSITION)
            finally:
                release.set()
            with self.assertRaisesRegex(ControlError, "规划期间反馈已移动"):
                await task
        self.assertIsNone(controller._trajectory)

    async def test_velocity_only_calibration_command(self):
        from backends import SimulatedBackend
        config = AppConfig()
        backend = SimulatedBackend(config.transmission.motor_ids)
        await backend.open()
        try:
            results = await backend.cycle(
                None, config.safety,
                velocities_rev_s={motor_id: 0.5 for motor_id in config.transmission.motor_ids},
                current_limit_scale=0.1, velocity_only=True)
            self.assertEqual(set(results), set(config.transmission.motor_ids))
        finally:
            await backend.close(send_stop=True)

    async def test_config_update_requires_stop_and_recalibration(self):
        controller = fake_controller()
        controller.calibrate()
        changed = AppConfig()
        changed.transmission = dataclasses.replace(changed.transmission, motor_reduction=4.0)
        with self.assertRaises(ControlError):
            await controller.update_config(changed)
        controller.stop()
        await controller.update_config(changed)
        self.assertFalse(controller.calibrated)
        self.assertEqual(controller.config.transmission.total_reduction, 8.0)
        with self.assertRaises(ControlError):
            controller.command_rpy([0, 0, 0])

    async def test_moteus_simulator_end_to_end(self):
        from backends import SimulatedBackend
        config = AppConfig()
        backend = SimulatedBackend(config.transmission.motor_ids)
        controller = MechanismController(config, backend, auto_calibrate_simulation=False)
        await backend.open()
        try:
            results = await backend.cycle(None, config.safety, command_stop=True)
            controller._apply_results(results, time.monotonic())
            with self.assertRaises(ControlError):
                controller.command_rpy([0, 0, 1])
            controller.calibrate()
            controller.command_rpy([2, -3, 5])
            for _ in range(300):
                targets, velocities = controller._motion_command()
                results = await backend.cycle(
                    targets, config.safety, velocities_rev_s=velocities)
                controller._apply_results(results, time.monotonic())
                if controller.actual_result is not None and np.max(np.abs(
                        controller.actual_result.motor_deg - controller.target_result.motor_deg)) < 0.5:
                    break
                await asyncio.sleep(0.005)
            self.assertIsNotNone(controller.actual_result)
            np.testing.assert_allclose(controller.actual_result.rpy_deg, [2, -3, 5], atol=0.2)
        finally:
            await controller.shutdown()
    async def test_boundary_transition_full_feedback(self):
        from backends import SimulatedBackend
        config = AppConfig()
        backend = SimulatedBackend(config.transmission.motor_ids)
        controller = MechanismController(config, backend, auto_calibrate_simulation=True)
        await controller.start()
        try:
            for _ in range(100):
                if controller.calibrated:
                    break
                await asyncio.sleep(0.02)
            self.assertTrue(controller.calibrated)
            max_pitch = 0.0
            for rpy in ([15, 15, 0], [0, 15, 0]):
                controller.command_rpy(rpy)
                deadline = time.monotonic() + 15.0
                while time.monotonic() < deadline:
                    self.assertIsNone(controller.fault)
                    if controller.actual_result is not None:
                        max_pitch = max(max_pitch, abs(controller.actual_result.rpy_deg[1]))
                        if controller._trajectory is None and np.max(np.abs(
                                controller.actual_result.rpy_deg - np.asarray(rpy))) < 0.05:
                            break
                    await asyncio.sleep(0.02)
                else:
                    self.fail(f"模拟器未能到达 {rpy}")
            self.assertIsNone(controller.fault)
            self.assertLess(max_pitch, 16.1)
            self.assertGreater(controller._path_rpy_allowance[1], 0.0)
        finally:
            await controller.shutdown()

    async def test_simulator_reconnect_autocalibrates(self):
        from backends import SimulatedBackend
        config = AppConfig()
        backend = SimulatedBackend(config.transmission.motor_ids)
        controller = MechanismController(config, backend, auto_calibrate_simulation=True)
        await controller.start()
        try:
            for _ in range(100):
                if controller.calibrated:
                    break
                await asyncio.sleep(0.02)
            self.assertTrue(controller.calibrated)
            reconnects = controller._reconnects
            await backend.close(send_stop=False)
            for _ in range(150):
                if controller.calibrated and controller._reconnects > reconnects:
                    break
                await asyncio.sleep(0.02)
            self.assertTrue(controller.calibrated)
            self.assertTrue(backend.is_open)
            self.assertGreater(controller._reconnects, reconnects)
        finally:
            await controller.shutdown()


if __name__ == "__main__":
    unittest.main()
