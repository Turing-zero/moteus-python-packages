"""Async coordinated control, calibration, safety interlocks, and telemetry."""

from __future__ import annotations

import asyncio
import copy
import dataclasses
import logging
import math
import time
from dataclasses import dataclass, field
from typing import Any, Iterable

import numpy as np
from moteus.protocol import Mode, Register

from backends import MoteusBackend, fault_description, feedback_mode_name
from kinematics import (ForwardResult, KinematicsError, forward_kinematics,
                        home_result, inverse_from_rpy, motor_deg_to_raw_rev,
                        raw_rev_to_motor_deg, verify_inverse)
from params import AppConfig
from trajectory import SynchronizedTrajectory, WaypointTrajectory


log = logging.getLogger("parallel_delta_control")


class ControlError(RuntimeError):
    pass


@dataclass
class MotorState:
    id: int
    online: bool = False
    values: dict[int, Any] = field(default_factory=dict)
    last_update: float = 0.0
    raw_target_rev: float | None = None
    ever_online: bool = False

    @property
    def raw_position_rev(self) -> float:
        return float(self.values.get(Register.POSITION, math.nan))

    @property
    def controller_mode(self) -> int | None:
        value = self.values.get(Register.MODE)
        return None if value is None else int(value)

    @property
    def controller_fault(self) -> int:
        return int(self.values.get(Register.FAULT, 0) or 0)


@dataclass
class ExtremeCalibrationProcess:
    stage: str
    deadline: float
    stable_since: dict[int, float | None] = field(default_factory=dict)
    reference_rev: dict[int, float] = field(default_factory=dict)
    stable_sum_rev: dict[int, float] = field(default_factory=dict)
    stable_count: dict[int, int] = field(default_factory=dict)
    positive_rev: np.ndarray | None = None


@dataclass
class InverseCommandPlan:
    preview: ForwardResult
    trajectory: SynchronizedTrajectory | WaypointTrajectory
    config: AppConfig
    zero_rev: np.ndarray
    start_motor: np.ndarray


def _json_safe(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return _json_safe(value.tolist())
    if isinstance(value, np.generic):
        return _json_safe(value.item())
    if isinstance(value, dict):
        return {str(k): _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    return value


def _pose_snapshot(result: ForwardResult | None) -> dict[str, Any]:
    if result is None:
        return {"valid": False, "motor_deg": None, "q_deg": None,
                "rpy_deg": None, "path_rpy_max_deg": None, "condition": None, "residual": None,
                "geometry": None}
    return _json_safe({
        "valid": True,
        "motor_deg": result.motor_deg,
        "q_deg": np.degrees(result.q),
        "rpy_deg": result.rpy_deg,
        "path_rpy_max_deg": result.path_rpy_max_deg,
        "condition": result.condition,
        "residual": result.residual,
        "iterations": result.iterations,
        "path_steps": result.path_steps,
        "geometry": result.geometry,
    })


class MechanismController:
    """All mutating operations run on the event loop that owns the backend."""

    def __init__(self, config: AppConfig, backend: MoteusBackend,
                 *, auto_calibrate_simulation: bool = True):
        config.validate()
        self.config = config
        self.backend = backend
        self.motors = {motor_id: MotorState(motor_id)
                       for motor_id in config.transmission.motor_ids}
        self.auto_calibrate_simulation = auto_calibrate_simulation
        self._sim_auto_calibrated = False
        self.calibrated = False
        self.enabled = False
        self.fault: str | None = None
        self.message = "模拟器等待启动" if backend.simulation else "实物已失能，请人工直立后标定"
        self.zero_rev = np.zeros(3)
        self.directions = np.asarray(config.transmission.directions, dtype=float)
        self.target_result = home_result(config)
        self.actual_result: ForwardResult | None = None
        self._previous_actual_motor: np.ndarray | None = None
        self._path_rpy_allowance = np.zeros(3)
        self._clear_and_hold = False
        self._clear_deadline = 0.0
        self._trajectory: SynchronizedTrajectory | WaypointTrajectory | None = None
        self._trajectory_progress = 0.0
        self._running = False
        self._task: asyncio.Task | None = None
        self._cycle_ms = 0.0
        self._loop_hz = 0.0
        self._timeouts = 0
        self._reconnects = 0
        self._last_loop: float | None = None
        self._previous_feedback_time: float | None = None
        self._last_feedback_jump: dict[str, Any] | None = None
        self._extreme_calibration: ExtremeCalibrationProcess | None = None
        self._last_extreme_calibration: dict[str, Any] | None = None

    @property
    def ids(self) -> tuple[int, int, int]:
        return self.config.transmission.motor_ids

    def _fresh(self, motor: MotorState, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else now
        return (motor.online and motor.last_update > 0 and
                now - motor.last_update <= self.config.safety.feedback_timeout_s)

    def _all_fresh(self) -> bool:
        now = time.monotonic()
        return all(self._fresh(motor, now) for motor in self.motors.values())

    def _raw_positions(self) -> np.ndarray:
        return np.array([self.motors[motor_id].raw_position_rev for motor_id in self.ids])

    def _raw_targets(self) -> dict[int, float]:
        return {motor_id: float(self.motors[motor_id].raw_target_rev)
                for motor_id in self.ids}

    def _motion_command(self) -> tuple[dict[int, float], dict[int, float]]:
        """Sample one common trajectory, rather than three independent moves."""
        if self._trajectory is None:
            return self._raw_targets(), {motor_id: 0.0 for motor_id in self.ids}
        positions, velocities, done, progress = self._trajectory.sample(time.monotonic())
        self._trajectory_progress = progress
        if done:
            self._trajectory = None
        return ({motor_id: float(value) for motor_id, value in zip(self.ids, positions)},
                {motor_id: float(value) for motor_id, value in zip(self.ids, velocities)})

    async def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._task = asyncio.create_task(self.run(), name="parallel-delta-control")

    async def run(self) -> None:
        period = 1.0 / self.config.safety.control_rate_hz
        while self._running:
            started = time.perf_counter()
            now = time.monotonic()
            if self._last_loop is not None:
                elapsed = started - self._last_loop
                if elapsed > 0:
                    self._loop_hz += (1.0 / elapsed - self._loop_hz) * 0.05
            self._last_loop = started
            try:
                if not self.backend.is_open:
                    await self.backend.open()
                    self._reconnects += 1
                    if self.backend.simulation:
                        self._sim_auto_calibrated = False
                    self._invalidate_calibration("后端已重新连接，必须重新直立标定")
                process = self._extreme_calibration
                command_stop = not self.enabled and process is None
                velocity_only = (process is not None and
                                 process.stage in ("positive", "negative"))
                if velocity_only:
                    targets, velocities = self._extreme_drive_commands()
                elif self.enabled:
                    targets, velocities = self._motion_command()
                else:
                    targets, velocities = None, None
                current_limit_scale = None if process is None else \
                    self.config.calibration.current_limit_scale
                results = await self.backend.cycle(
                    targets, self._cycle_safety(), command_stop=command_stop,
                    velocities_rev_s=velocities,
                    current_limit_scale=current_limit_scale,
                    velocity_only=velocity_only)
                self._apply_results(results, time.monotonic())
                if (self.backend.simulation and self.auto_calibrate_simulation and
                        self._extreme_calibration is None and
                        not self._sim_auto_calibrated and not self.calibrated and
                        self._all_fresh()):
                    self.calibrate()
                    self._sim_auto_calibrated = True
                    self.message = "模拟器已在已知直立零位标定"
                if self._extreme_calibration is not None:
                    self._service_extreme_calibration(time.monotonic())
                else:
                    self._service_clear_and_hold(time.monotonic())
            except Exception as exc:
                self._timeouts += 1
                log.warning("控制/通信周期异常：%s", exc)
                self.enabled = False
                self._trajectory = None
                self._clear_and_hold = False
                self._invalidate_calibration(f"通信/控制异常：{exc}")
                for motor in self.motors.values():
                    motor.online = False
                    motor.raw_target_rev = None
                await self.backend.close(send_stop=False)
                await asyncio.sleep(0.5)
            elapsed = time.perf_counter() - started
            period = 1.0 / self.config.safety.control_rate_hz
            self._cycle_ms += (elapsed * 1000.0 - self._cycle_ms) * 0.05
            await asyncio.sleep(max(0.0, period - elapsed))

    def _apply_results(self, results: dict[int, dict[int, Any]], now: float) -> None:
        for motor_id, motor in self.motors.items():
            if motor_id not in results:
                if self.calibrated and self._fresh(motor, now) is False and motor.ever_online:
                    self._trip(f"电机 {motor_id} 反馈过期/离线")
                continue
            values = results[motor_id]
            position = float(values.get(Register.POSITION, math.nan))
            if not math.isfinite(position):
                self._trip(f"电机 {motor_id} 返回无效位置")
                continue
            motor.values = values
            motor.last_update = now
            motor.online = True
            motor.ever_online = True
            mode = motor.controller_mode
            fault = motor.controller_fault
            expected_torque_limit = (self._extreme_calibration is not None and
                                     mode == Mode.POSITION and fault == 102)
            if (fault and not expected_torque_limit) or \
                    ((self.enabled or self._extreme_calibration is not None)
                     and mode in (Mode.FAULT, Mode.TIMEOUT)):
                self._trip(f"电机 {motor_id} 控制器故障/超时："
                           f"{feedback_mode_name(values)} "
                           f"fault={fault_description(motor.controller_fault)}")
        if not self._all_fresh():
            if self.calibrated and self.enabled:
                self._trip("三电机反馈不完整，协调运动已停止")
            return
        if self.calibrated:
            self._update_actual_pose(now)

    def _update_actual_pose(self, now: float | None = None) -> None:
        now = time.monotonic() if now is None else now
        raw = self._raw_positions()
        motor_deg = raw_rev_to_motor_deg(raw, self.zero_rev, self.directions)
        previous = self._previous_actual_motor
        if previous is not None:
            deltas = motor_deg - previous
            worst = int(np.argmax(np.abs(deltas)))
            step_deg = float(deltas[worst])
            threshold = self.config.safety.max_feedback_step_deg
            if abs(step_deg) > threshold:
                elapsed_s = None
                if self._previous_feedback_time is not None:
                    elapsed_s = max(0.0, now - self._previous_feedback_time)
                motor_id = self.ids[worst]
                timing = "反馈间隔未知" if elapsed_s is None else \
                    f"反馈间隔 {elapsed_s * 1000.0:.1f} ms"
                message = (f"位置反馈不连续：电机 {motor_id} 变化 {step_deg:+.3f}° "
                           f"（阈值 {threshold:g}°，{timing}）")
                self._last_feedback_jump = {
                    "motor_id": motor_id,
                    "step_deg": step_deg,
                    "step_rev": float(self.directions[worst] * step_deg / 360.0),
                    "threshold_deg": threshold,
                    "elapsed_ms": None if elapsed_s is None else elapsed_s * 1000.0,
                    "raw_position_rev": raw.copy(),
                    "delta_deg": deltas.copy(),
                    "previous_motor_deg": previous.copy(),
                    "loop_hz": self._loop_hz,
                    "cycle_ms": self._cycle_ms,
                }
                log.warning("%s；raw_rev=%s；delta_deg=%s；loop_hz=%.1f；cycle_ms=%.1f",
                            message, np.round(raw, 9).tolist(),
                            np.round(deltas, 6).tolist(), self._loop_hz, self._cycle_ms)
                self._invalidate_calibration(message)
                self._trip(message)
                return
        initial_R = np.eye(3) if self.actual_result is None else self.actual_result.rotation
        initial_motor = np.zeros(3) if previous is None else previous
        try:
            feedback_margin = (self.config.safety.feedback_rpy_margin_deg +
                               self._path_rpy_allowance)
            result = forward_kinematics(
                self.config, motor_deg, initial_motor, initial_R,
                rpy_margin_deg=feedback_margin)
        except KinematicsError as exc:
            self._trip(f"反馈正解失败：{exc}")
            return
        self.actual_result = result
        self._previous_actual_motor = motor_deg.copy()
        self._previous_feedback_time = now

    def _cycle_safety(self):
        if self._extreme_calibration is None:
            return self.config.safety
        calibration = self.config.calibration
        return dataclasses.replace(
            self.config.safety,
            velocity_limit_rev_s=calibration.velocity_rev_s,
            accel_limit_rev_s2=calibration.accel_rev_s2,
            maximum_torque_nm=calibration.drive_torque_nm)

    def _reset_extreme_stage(self, stage: str, now: float) -> None:
        process = self._extreme_calibration
        assert process is not None
        process.stage = stage
        process.deadline = now + self.config.calibration.stage_timeout_s
        process.stable_since = {motor_id: None for motor_id in self.ids}
        process.reference_rev = {
            motor_id: self.motors[motor_id].raw_position_rev for motor_id in self.ids}
        process.stable_sum_rev = {motor_id: 0.0 for motor_id in self.ids}
        process.stable_count = {motor_id: 0 for motor_id in self.ids}

    def _extreme_drive_commands(self) -> tuple[None, dict[int, float]]:
        process = self._extreme_calibration
        assert process is not None and process.stage in ("positive", "negative")
        sign = 1.0 if process.stage == "positive" else -1.0
        calibration = self.config.calibration
        velocities: dict[int, float] = {}
        for index, motor_id in enumerate(self.ids):
            directed_sign = sign * float(self.directions[index])
            velocities[motor_id] = directed_sign * calibration.velocity_rev_s
        return None, velocities

    def _fail_extreme_calibration(self, reason: str) -> None:
        message = f"极限标定失败：{reason}"
        log.error(message)
        self._extreme_calibration = None
        self._invalidate_calibration(message)
        self._trip(message)

    def _service_extreme_calibration(self, now: float) -> None:
        process = self._extreme_calibration
        if process is None:
            return
        if self.fault:
            self._extreme_calibration = None
            return
        if not self._all_fresh():
            self._fail_extreme_calibration("三轴反馈不完整或过期")
            return
        if now > process.deadline:
            self._fail_extreme_calibration(f"阶段 {process.stage} 超时，可能存在卡滞或工作电流不足")
            return
        calibration = self.config.calibration
        for motor_id, motor in self.motors.items():
            current = float(motor.values.get(Register.Q_CURRENT, math.nan))
            if not math.isfinite(current):
                self._fail_extreme_calibration(f"电机 {motor_id} Q 轴电流反馈无效")
                return
            if abs(current) > calibration.max_q_current_a:
                self._fail_extreme_calibration(
                    f"电机 {motor_id} Q 轴电流 {current:+.3f} A 超过标定限值")
                return
        if process.stage == "return":
            if self._trajectory is not None:
                return
            if self._previous_actual_motor is None:
                self._fail_extreme_calibration("归零阶段缺少有效反馈正解")
                return
            error = float(np.max(np.abs(self._previous_actual_motor)))
            self._last_extreme_calibration = dict(
                self._last_extreme_calibration or {}, return_error_deg=error)
            if error > calibration.return_tol_deg:
                self.message = f"极限标定：等待归零，当前误差 {error:.3f}°"
                return
            zero = self.zero_rev.copy()
            for motor_id, position in zip(self.ids, zero):
                self.motors[motor_id].raw_target_rev = float(position)
            self._extreme_calibration = None
            self.enabled = True
            self.calibrated = True
            self.fault = None
            self.target_result = home_result(self.config)
            self.message = "极限标定完成：三轴区间一致，已归零并保持"
            log.info("极限标定完成，zero_rev=%s", zero.tolist())
            return

        sign = 1.0 if process.stage == "positive" else -1.0
        for index, motor_id in enumerate(self.ids):
            motor = self.motors[motor_id]
            position = motor.raw_position_rev
            velocity = float(motor.values.get(Register.VELOCITY, math.nan))
            current = float(motor.values.get(Register.Q_CURRENT, math.nan))
            stable = (math.isfinite(velocity) and math.isfinite(current) and
                      abs(velocity) <= calibration.stall_velocity_rev_s and
                      sign * float(self.directions[index]) * current >=
                      0.5 * calibration.drive_current_a)
            reference = process.reference_rev[motor_id]
            if stable and abs((position - reference) * 360.0) <= \
                    calibration.stall_position_tol_deg:
                if process.stable_since[motor_id] is None:
                    process.stable_since[motor_id] = now
                process.stable_sum_rev[motor_id] += position
                process.stable_count[motor_id] += 1
            else:
                process.stable_since[motor_id] = None
                process.reference_rev[motor_id] = position
                process.stable_sum_rev[motor_id] = 0.0
                process.stable_count[motor_id] = 0
        if not all(process.stable_since[motor_id] is not None and
                   now - process.stable_since[motor_id] >= calibration.stable_s
                   for motor_id in self.ids):
            return
        extreme = np.array([
            process.stable_sum_rev[motor_id] / process.stable_count[motor_id]
            for motor_id in self.ids])
        if process.stage == "positive":
            process.positive_rev = extreme
            log.info("正转极限平均位置 rev=%s", extreme.tolist())
            self._reset_extreme_stage("negative", now)
            self.message = "极限标定：正转极限已记录，三轴限速限流反转"
            return
        assert process.positive_rev is not None
        self._accept_extreme_midpoint(process.positive_rev, extreme, now)

    def _accept_extreme_midpoint(self, positive: np.ndarray,
                                 negative: np.ndarray, now: float) -> None:
        span = self.directions * (positive - negative)
        span_deg = span * 360.0
        mean_span = float(np.mean(span))
        imbalance = (float(np.max(span) - np.min(span)) / mean_span
                     if mean_span > 0 else math.inf)
        zero = 0.5 * (positive + negative)
        self._last_extreme_calibration = {
            "positive_rev": positive.copy(), "negative_rev": negative.copy(),
            "span_rev": span.copy(), "span_deg": span_deg.copy(),
            "imbalance": imbalance, "zero_rev": zero.copy(),
        }
        if np.any(span < self.config.calibration.min_span_rev):
            self._fail_extreme_calibration(
                f"有效行程过小：{np.round(span_deg, 3).tolist()}°，可能某轴未运动")
            return
        if imbalance > self.config.calibration.max_span_imbalance:
            self._fail_extreme_calibration(
                f"三轴行程 {np.round(span_deg, 3).tolist()}° 不一致，"
                f"相对差异 {imbalance * 100.0:.2f}% 超过限制，可能存在卡滞")
            return
        self.zero_rev = zero
        self.calibrated = True
        self.enabled = False
        self.fault = None
        self.actual_result = None
        self._previous_actual_motor = None
        self._previous_feedback_time = None
        self._path_rpy_allowance = np.zeros(3)
        self._update_actual_pose(now)
        if self.fault or self.actual_result is None:
            reason = self.fault or "反馈正解无有效结果"
            self._fail_extreme_calibration(reason)
            return
        self.target_result = home_result(self.config)
        self._trajectory = SynchronizedTrajectory.between(
            self._raw_positions(), zero, now,
            self.config.calibration.velocity_rev_s,
            self.config.calibration.accel_rev_s2)
        self._trajectory_progress = 0.0
        for motor_id, position in zip(self.ids, zero):
            self.motors[motor_id].raw_target_rev = float(position)
        process = self._extreme_calibration
        assert process is not None
        process.stage = "return"
        process.deadline = (now + self._trajectory.duration_s +
                            self.config.calibration.stage_timeout_s)
        self.enabled = True
        self.message = (f"极限标定：三轴行程 {np.round(span_deg, 3).tolist()}°，"
                        "校验通过，正在归零")

    def _service_clear_and_hold(self, now: float) -> None:
        if not self._clear_and_hold or now < self._clear_deadline:
            return
        self._clear_and_hold = False
        if not self._all_fresh():
            self._trip("清故障后反馈不完整")
            return
        if any(motor.controller_fault for motor in self.motors.values()):
            self.fault = "控制器仍存在 FAULT，请检查硬件"
            return
        self.fault = None
        if self.calibrated:
            self._capture_hold_targets()
            self.enabled = True
            self.message = "故障已清除，在当前位置保持"

    def _invalidate_calibration(self, message: str) -> None:
        self._extreme_calibration = None
        self.calibrated = False
        self.enabled = False
        self._trajectory = None
        self.actual_result = None
        self._previous_actual_motor = None
        self._previous_feedback_time = None
        self._path_rpy_allowance = np.zeros(3)
        self.zero_rev = np.zeros(3)
        self.message = message
        for motor in self.motors.values():
            motor.raw_target_rev = None

    def _trip(self, message: str) -> None:
        self._extreme_calibration = None
        self.fault = message
        self.message = message
        self.enabled = False
        self._trajectory = None
        self._clear_and_hold = False
        for motor in self.motors.values():
            motor.raw_target_rev = None

    def _check_fresh_no_fault(self) -> None:
        if not self.backend.is_open or not self._all_fresh():
            raise ControlError("三个电机必须全部在线且反馈新鲜")
        if self.fault:
            raise ControlError(f"故障锁存：{self.fault}")
        for motor in self.motors.values():
            if motor.controller_fault or motor.controller_mode == Mode.FAULT:
                raise ControlError(f"电机 {motor.id} 处于 FAULT，请先检查和清除")

    def calibrate(self) -> None:
        """Capture software zero only; never write or rezero controller config."""
        if self._extreme_calibration is not None:
            raise ControlError("极限标定期间不能人工标定")
        if self.enabled:
            raise ControlError("标定前必须先停止/失能")
        self._check_fresh_no_fault()
        self._trajectory = None
        self.zero_rev = self._raw_positions()
        self.calibrated = True
        self.fault = None
        self.target_result = home_result(self.config)
        self.actual_result = home_result(self.config)
        self._previous_actual_motor = np.zeros(3)
        self._previous_feedback_time = max(
            motor.last_update for motor in self.motors.values())
        self._path_rpy_allowance = np.zeros(3)
        for motor_id, raw in zip(self.ids, self.zero_rev):
            self.motors[motor_id].raw_target_rev = float(raw)
        self.enabled = True
        self.message = "已捕获直立软件零位并保持当前位置；未修改控制器持久配置"

    def start_extreme_calibration(self) -> None:
        """Start the non-blocking positive/negative mechanical-limit procedure."""
        if self._extreme_calibration is not None:
            raise ControlError("极限标定已在进行")
        if self.enabled:
            raise ControlError("极限标定前必须先 stop；流程会主动低速驱动三轴")
        if self.config.calibration.drive_torque_nm is None:
            raise ControlError("请先在动态参数中填写真实的电机 Kv，用于电流/扭矩换算")
        if self.config.calibration.current_limit_scale is None:
            raise ControlError("请先在动态参数中填写真实的固件 servo.max_current_A")
        self._check_fresh_no_fault()
        self._invalidate_calibration("准备极限标定；原零位已作废")
        self.fault = None
        self._last_extreme_calibration = None
        now = time.monotonic()
        self._extreme_calibration = ExtremeCalibrationProcess(
            stage="positive", deadline=now + self.config.calibration.stage_timeout_s)
        self._reset_extreme_stage("positive", now)
        self.message = "极限标定：三轴同时低速、限流正转，寻找正转极限"

    def hold(self) -> None:
        if self._extreme_calibration is not None:
            raise ControlError("极限标定期间不能手动保持")
        self._check_fresh_no_fault()
        if not self.calibrated:
            raise ControlError("未标定：请在失能状态人工直立后确认零位")
        if self.actual_result is None:
            raise ControlError("尚无有效反馈正解，无法安全保持")
        self._capture_hold_targets()
        self.enabled = True
        self.message = "在当前位置保持"

    def _capture_hold_targets(self) -> None:
        self._trajectory = None
        raw = self._raw_positions()
        for motor_id, position in zip(self.ids, raw):
            self.motors[motor_id].raw_target_rev = float(position)
        if self.actual_result is not None:
            self.target_result = copy.deepcopy(self.actual_result)

    def stop(self) -> None:
        self._extreme_calibration = None
        self.enabled = False
        self._trajectory = None
        self._clear_and_hold = False
        self.message = "已请求整组停止/失能；软件 stop 不替代硬件急停"
        for motor in self.motors.values():
            motor.raw_target_rev = None

    def clear(self) -> None:
        if self._extreme_calibration is not None:
            raise ControlError("极限标定期间不能清故障；请先 stop")
        if not self.backend.is_open or not self._all_fresh():
            raise ControlError("清故障需要三个电机在线")
        self.enabled = False
        self._trajectory = None
        self._clear_and_hold = True
        self._clear_deadline = time.monotonic() + 0.2
        self.message = "正在清除 FAULT/TIMEOUT；成功后在当前位置保持"

    def _check_ready(self) -> None:
        if self._extreme_calibration is not None:
            raise ControlError("极限标定进行中，禁止发送其他运动目标")
        self._check_fresh_no_fault()
        if not self.calibrated:
            raise ControlError("未标定：请在失能状态人工直立后确认零位")
        if not self.enabled:
            raise ControlError("当前为 stopped，请先执行 hold")
        if self.actual_result is None or self._previous_actual_motor is None:
            raise ControlError("尚无有效反馈正解")

    def _check_motor_limits(self, motor_deg: np.ndarray) -> None:
        safety = self.config.safety
        if np.any(motor_deg < np.asarray(safety.motor_min_deg) - 1e-9) or \
                np.any(motor_deg > np.asarray(safety.motor_max_deg) + 1e-9):
            raise ControlError(
                f"电机目标 {np.round(motor_deg, 6).tolist()}° 超出软件限位")
        assert self._previous_actual_motor is not None
        distance = float(np.max(np.abs(motor_deg - self._previous_actual_motor)))
        if distance > safety.max_move_deg + 1e-9:
            raise ControlError(
                f"单次目标变化 {distance:.3f}° 超过 max_move={safety.max_move_deg}°")

    def _validate_waypoint_path(self, trajectory: WaypointTrajectory) -> None:
        assert self._previous_actual_motor is not None and self.actual_result is not None
        self._validate_waypoint_path_data(
            self.config, trajectory, self.zero_rev, self.directions,
            self._previous_actual_motor, self.actual_result.rotation)

    @staticmethod
    def _validate_waypoint_path_data(config: AppConfig, trajectory: WaypointTrajectory,
                                     zero_rev: np.ndarray, directions: np.ndarray,
                                     start_motor: np.ndarray,
                                     start_rotation: np.ndarray) -> None:
        """Validate the actual spline using only immutable planning inputs."""
        previous_motor = start_motor.copy()
        previous_rotation = start_rotation.copy()
        started = trajectory.segments[0].start_time_s
        minimum = np.asarray(config.safety.motor_min_deg)
        maximum = np.asarray(config.safety.motor_max_deg)
        for alpha in np.linspace(0.0, 1.0, 21)[1:]:
            raw = trajectory.sample(started + trajectory.duration_s * alpha)[0]
            motor = raw_rev_to_motor_deg(raw, zero_rev, directions)
            if np.any(motor < minimum) or np.any(motor > maximum):
                raise ControlError("连续样条绕行路径超出电机软限位")
            try:
                result = forward_kinematics(
                    config, motor, previous_motor, previous_rotation)
            except KinematicsError as exc:
                raise ControlError(f"连续样条路径校验失败：{exc}") from exc
            previous_motor = motor
            previous_rotation = result.rotation

    def _commit_with_trajectory(self, preview: ForwardResult,
                                trajectory: SynchronizedTrajectory | WaypointTrajectory) -> None:
        """Commit only after all planning and validation has completed."""
        motor_deg = np.asarray(preview.motor_deg)
        self._check_motor_limits(motor_deg)
        raw_targets = motor_deg_to_raw_rev(motor_deg, self.zero_rev, self.directions)
        command_limits = np.array([
            self.config.safety.roll_limit_deg, self.config.safety.pitch_limit_deg,
            self.config.safety.yaw_limit_deg])
        self._path_rpy_allowance = np.maximum(
            0.0, preview.path_rpy_max_deg - command_limits)
        for motor_id, target in zip(self.ids, raw_targets):
            self.motors[motor_id].raw_target_rev = float(target)
        self._trajectory = trajectory
        self._trajectory_progress = 0.0
        self.target_result = preview
        self.message = ("直接路径预检未通过，已改为经直立点平滑绕行"
                        if isinstance(trajectory, WaypointTrajectory)
                        else "已接收三轴协调目标")

    def _commit(self, preview: ForwardResult, duration_s: float | None = None,
                raw_waypoints: list[np.ndarray] | None = None) -> None:
        raw_targets = motor_deg_to_raw_rev(preview.motor_deg, self.zero_rev, self.directions)
        try:
            if raw_waypoints is None:
                trajectory = SynchronizedTrajectory.between(
                    self._raw_positions(), raw_targets, time.monotonic(),
                    self.config.safety.velocity_limit_rev_s,
                    self.config.safety.accel_limit_rev_s2,
                    duration_s=duration_s)
            else:
                trajectory = WaypointTrajectory.between(
                    raw_waypoints, time.monotonic(),
                    self.config.safety.velocity_limit_rev_s,
                    self.config.safety.accel_limit_rev_s2,
                    duration_s=duration_s)
        except (TypeError, ValueError) as exc:
            raise ControlError(str(exc)) from exc
        if raw_waypoints is not None:
            self._validate_waypoint_path(trajectory)
        self._commit_with_trajectory(preview, trajectory)

    def command_motors(self, motor_deg: Iterable[float], *,
                       duration_s: float | None = None) -> ForwardResult:
        self._check_ready()
        assert self.actual_result is not None
        requested = np.asarray(list(motor_deg), dtype=float).reshape(3)
        self._check_motor_limits(requested)
        preview = forward_kinematics(
            self.config, requested, self._previous_actual_motor,
            self.actual_result.rotation)
        self._commit(preview, duration_s)
        return preview

    def _plan_inverse(self, rpy_deg: Iterable[float],
                      duration_s: float | None = None) -> InverseCommandPlan:
        """Pure planning work; safe to run in asyncio.to_thread()."""
        assert self.actual_result is not None and self._previous_actual_motor is not None
        config = self.config
        zero = self.zero_rev.copy()
        directions = self.directions.copy()
        start_motor = self._previous_actual_motor.copy()
        start_rotation = self.actual_result.rotation.copy()
        raw_start = self._raw_positions()
        try:
            inverse = inverse_from_rpy(config, rpy_deg, start_motor)
            preview = verify_inverse(config, inverse, start_motor, start_rotation)
            raw_target = motor_deg_to_raw_rev(preview.motor_deg, zero, directions)
            trajectory = SynchronizedTrajectory.between(
                raw_start, raw_target, time.monotonic(),
                config.safety.velocity_limit_rev_s,
                config.safety.accel_limit_rev_s2,
                duration_s=duration_s)
        except KinematicsError as direct_error:
            log.warning("直接 FK 路径未通过（%s），改用直立点绕行", direct_error)
            home_inverse = inverse_from_rpy(config, [0, 0, 0], start_motor)
            first = verify_inverse(config, home_inverse, start_motor, start_rotation)
            second_inverse = inverse_from_rpy(config, rpy_deg, first.motor_deg)
            preview = verify_inverse(
                config, second_inverse, first.motor_deg, first.rotation)
            raw_waypoints = [raw_start]
            for segment in (first, preview):
                raw_waypoints.append(motor_deg_to_raw_rev(
                    segment.motor_deg, zero, directions))
            trajectory = WaypointTrajectory.between(
                raw_waypoints, time.monotonic(),
                config.safety.velocity_limit_rev_s,
                config.safety.accel_limit_rev_s2,
                duration_s=duration_s)
            self._validate_waypoint_path_data(
                config, trajectory, zero, directions, start_motor, start_rotation)
        return InverseCommandPlan(
            preview=preview, trajectory=trajectory, config=config,
            zero_rev=zero, start_motor=start_motor)

    def _commit_inverse_plan(self, plan: InverseCommandPlan) -> ForwardResult:
        """Run on the owning event loop; reject stale or invalidated plans."""
        self._check_ready()
        if self.config is not plan.config or not np.array_equal(self.zero_rev, plan.zero_rev):
            raise ControlError("规划期间配置或零位已变化，请重新发送目标")
        assert self._previous_actual_motor is not None
        displacement = float(np.max(np.abs(
            self._previous_actual_motor - plan.start_motor)))
        if displacement > 0.25:
            raise ControlError(
                f"规划期间反馈已移动 {displacement:.3f}°，为保护通信与路径安全，请重试")
        trajectory = plan.trajectory.shift_start(time.monotonic())
        self._commit_with_trajectory(plan.preview, trajectory)
        return plan.preview

    def command_rpy(self, rpy_deg: Iterable[float], *,
                    duration_s: float | None = None) -> ForwardResult:
        self._check_ready()
        try:
            plan = self._plan_inverse(rpy_deg, duration_s)
        except KinematicsError:
            raise
        except (TypeError, ValueError) as exc:
            raise ControlError(str(exc)) from exc
        return self._commit_inverse_plan(plan)

    async def command_rpy_async(self, rpy_deg: Iterable[float], *,
                                duration_s: float | None = None) -> ForwardResult:
        self._check_ready()
        values = list(rpy_deg)
        try:
            plan = await asyncio.to_thread(self._plan_inverse, values, duration_s)
        except KinematicsError:
            raise
        except (TypeError, ValueError) as exc:
            raise ControlError(str(exc)) from exc
        return self._commit_inverse_plan(plan)

    async def update_config(self, config: AppConfig) -> None:
        config.validate()
        if self.enabled or self._extreme_calibration is not None:
            raise ControlError("修改配置前必须先 stop；修改后需要重新标定")
        old_ids = self.ids
        self.stop()
        self._invalidate_calibration("配置已修改，必须重新直立标定")
        self.config = config
        self.directions = np.asarray(config.transmission.directions, dtype=float)
        self.target_result = home_result(config)
        self.fault = None
        if old_ids != config.transmission.motor_ids:
            await self.backend.close(send_stop=True)
            self.backend.reconfigure_ids(config.transmission.motor_ids)
            self.motors = {motor_id: MotorState(motor_id)
                           for motor_id in config.transmission.motor_ids}
            self._sim_auto_calibrated = False
        else:
            for motor in self.motors.values():
                motor.raw_target_rev = None
        if self.backend.simulation:
            self._sim_auto_calibrated = False

    def _state_name(self) -> str:
        if self.fault:
            return "fault"
        if not self.backend.is_open or not any(m.online for m in self.motors.values()):
            return "disconnected"
        if self._extreme_calibration is not None:
            return "calibrating"
        if not self.calibrated:
            return "uncalibrated"
        if not self.enabled:
            return "stopped"
        if self._trajectory is not None:
            return "moving"
        if self.actual_result is not None:
            distance = np.max(np.abs(
                self.actual_result.motor_deg - self.target_result.motor_deg))
            if distance > self.config.safety.command_complete_tol_deg:
                return "moving"
        return "ready"

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        actual_motor = None if self.actual_result is None else self.actual_result.motor_deg
        target_motor = self.target_result.motor_deg
        motors = []
        for index, motor_id in enumerate(self.ids):
            motor = self.motors[motor_id]
            values = motor.values

            def number(register: int) -> float | None:
                value = values.get(register)
                if value is None:
                    return None
                value = float(value)
                return value if math.isfinite(value) else None

            motors.append({
                "id": motor_id,
                "online": self._fresh(motor, now),
                "mode": motor.controller_mode,
                "mode_name": feedback_mode_name(values),
                "raw_position_rev": number(Register.POSITION),
                "raw_target_rev": motor.raw_target_rev,
                "abs_position_rev": number(Register.ABS_POSITION),
                "motor_position_deg": None if actual_motor is None else float(actual_motor[index]),
                "motor_target_deg": float(target_motor[index]),
                "shaft_position_deg": None if self.actual_result is None else
                    float(np.degrees(self.actual_result.q[index])),
                "shaft_target_deg": float(np.degrees(self.target_result.q[index])),
                "velocity_rev_s": number(Register.VELOCITY),
                "torque_nm": number(Register.TORQUE),
                "q_current_a": number(Register.Q_CURRENT),
                "d_current_a": number(Register.D_CURRENT),
                "voltage": number(Register.VOLTAGE),
                "temperature": number(Register.TEMPERATURE),
                "fault": motor.controller_fault,
                "trajectory_complete": int(values.get(Register.TRAJECTORY_COMPLETE, 0) or 0),
                "age_ms": (now - motor.last_update) * 1000.0 if motor.last_update else None,
            })
        process = self._extreme_calibration
        stable_values = [now - since for since in process.stable_since.values()
                         if since is not None] if process is not None else []
        stable_s = min(stable_values) if len(stable_values) == 3 else 0.0
        return _json_safe({
            "mode": self.backend.mode_name,
            "state": self._state_name(),
            "calibrated": self.calibrated,
            "enabled": self.enabled,
            "fault": self.fault,
            "message": self.message,
            "last_feedback_jump": self._last_feedback_jump,
            "extreme_calibration": {
                "active": process is not None,
                "stage": None if process is None else process.stage,
                "stable_s": stable_s,
                "positive_rev": None if process is None else process.positive_rev,
            },
            "last_extreme_calibration": self._last_extreme_calibration,
            "config": self.config.to_dict(),
            "motors": motors,
            "target": _pose_snapshot(self.target_result),
            "actual": _pose_snapshot(self.actual_result),
            "motion": {
                "active": self._trajectory is not None,
                "progress": self._trajectory_progress,
                "duration_s": None if self._trajectory is None else self._trajectory.duration_s,
                "path_rpy_allowance_deg": self._path_rpy_allowance,
            },
            "backend": {
                "open": self.backend.is_open,
                "loop_hz": self._loop_hz,
                "cycle_ms": self._cycle_ms,
                "timeouts": self._timeouts,
                "reconnects": self._reconnects,
            },
            "timestamp": now,
        })

    async def shutdown(self) -> None:
        self._running = False
        if self._task is not None:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self._task = None
        self.stop()
        await self.backend.close(send_stop=True)
