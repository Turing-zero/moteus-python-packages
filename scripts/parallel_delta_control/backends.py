"""Shared-transport moteus hardware and simulator backends."""

from __future__ import annotations

import asyncio
import dataclasses
import math
import os
from typing import Any, Iterable

import moteus
from moteus import multiplex as mp
from moteus.protocol import Mode, Register

from params import SafetyParams


class BackendError(RuntimeError):
    pass


def make_query_resolution() -> moteus.QueryResolution:
    query = moteus.QueryResolution()
    query.q_current = mp.F32
    query.d_current = mp.F32
    query.trajectory_complete = mp.INT8
    query.abs_position = mp.F32
    return query


class MoteusBackend:
    """One asyncio Transport and one Controller per configured motor ID."""

    def __init__(self, ids: Iterable[int], *, simulation: bool = False,
                 channel: str = "can1", restart_ms: int = 100,
                 disable_brs: bool = False, bitrate: int | None = None,
                 dbitrate: int | None = None):
        self.ids = tuple(int(v) for v in ids)
        self.simulation = simulation
        self.channel = channel
        self.restart_ms = restart_ms
        self.disable_brs = disable_brs
        self.bitrate = bitrate
        self.dbitrate = dbitrate
        self.transport: moteus.Transport | None = None
        self.controllers: dict[int, moteus.Controller] = {}
        self.query_resolution = make_query_resolution()
        self.open_count = 0

    @property
    def mode_name(self) -> str:
        return "simulation" if self.simulation else "hardware"

    @property
    def is_open(self) -> bool:
        return self.transport is not None

    async def open(self) -> None:
        if self.transport is not None:
            return
        if self.simulation:
            from moteus.simulator import SimulatedTransport
            self.transport = SimulatedTransport(self.ids)
        else:
            if os.name != "nt" and not os.path.exists(f"/sys/class/net/{self.channel}"):
                raise BackendError(f"CAN 接口 {self.channel} 不存在")
            kwargs: dict[str, Any] = {
                "channel": self.channel,
                "interface": "socketcan",
                "restart_ms": self.restart_ms,
            }
            if self.disable_brs:
                kwargs["disable_brs"] = True
            if self.bitrate is not None:
                kwargs["bitrate"] = self.bitrate
            if self.dbitrate is not None:
                kwargs["data_bitrate"] = self.dbitrate
            device = moteus.PythonCanDevice(**kwargs)
            self.transport = moteus.Transport([device])
        self.controllers = {
            motor_id: moteus.Controller(
                id=motor_id, transport=self.transport,
                query_resolution=self.query_resolution)
            for motor_id in self.ids
        }
        self.open_count += 1

    def reconfigure_ids(self, ids: Iterable[int]) -> None:
        if self.transport is not None:
            raise BackendError("修改电机 ID 前必须关闭后端")
        self.ids = tuple(int(v) for v in ids)

    async def cycle(self, targets_rev: dict[int, float] | None,
                    safety: SafetyParams, *, command_stop: bool = False,
                    velocities_rev_s: dict[int, float] | None = None,
                    current_limit_scale: float | None = None,
                    velocity_only: bool = False,
                    ) -> dict[int, dict[int, Any]]:
        if self.transport is None:
            raise BackendError("后端尚未打开")
        commands = []
        for motor_id in self.ids:
            controller = self.controllers[motor_id]
            if command_stop:
                commands.append(controller.make_stop(query=True))
            elif targets_rev is None and not velocity_only:
                commands.append(controller.make_query())
            else:
                kwargs: dict[str, Any] = {
                    "position": math.nan if velocity_only else
                        float(targets_rev[motor_id]),
                    "velocity": 0.0 if velocities_rev_s is None else
                        float(velocities_rev_s[motor_id]),
                    "velocity_limit": safety.velocity_limit_rev_s,
                    "accel_limit": safety.accel_limit_rev_s2,
                    "watchdog_timeout": safety.watchdog_timeout_s,
                    "query": True,
                }
                if safety.maximum_torque_nm is not None:
                    kwargs["maximum_torque"] = safety.maximum_torque_nm
                if safety.kp_scale is not None:
                    kwargs["kp_scale"] = safety.kp_scale
                if safety.kd_scale is not None:
                    kwargs["kd_scale"] = safety.kd_scale
                if current_limit_scale is not None:
                    kwargs["ilimit_scale"] = float(current_limit_scale)
                commands.append(controller.make_position(**kwargs))
        try:
            results = await asyncio.wait_for(
                self.transport.cycle(commands), timeout=safety.cycle_timeout_s)
        except (asyncio.TimeoutError, TimeoutError) as exc:
            raise BackendError("moteus 控制周期超时") from exc
        except Exception as exc:
            raise BackendError(f"moteus 通信失败: {exc}") from exc
        return {
            int(result.id): dict(result.values)
            for result in results
            if result is not None and hasattr(result, "values")
        }

    async def stop_all(self) -> None:
        if self.transport is None:
            return
        try:
            commands = [self.controllers[motor_id].make_stop(query=False)
                        for motor_id in self.ids]
            await asyncio.wait_for(self.transport.cycle(commands), timeout=0.5)
        except Exception:
            # Shutdown is best-effort; the per-command watchdog remains active.
            pass

    async def close(self, send_stop: bool = True) -> None:
        if self.transport is None:
            return
        if send_stop:
            await self.stop_all()
        transport, self.transport = self.transport, None
        self.controllers = {}
        try:
            transport.close()
        except Exception:
            pass


class SimulatedBackend(MoteusBackend):
    """moteus's own servo simulator; no mechanism dynamics are implied."""

    def __init__(self, ids: Iterable[int]):
        super().__init__(ids, simulation=True)


class CandleApiBackend(MoteusBackend):
    """Persistent Windows candleLightFD backend using candle_api directly."""

    def __init__(self, ids: Iterable[int], serial_number: str | None = None,
                 channel_index: int = 0):
        super().__init__(ids, simulation=False, channel="candle_api")
        self.serial_number = serial_number
        self.channel_index = channel_index

    async def open(self) -> None:
        if self.transport is not None:
            return
        from canfd_query import DirectCandleTransport
        device = DirectCandleTransport(
            serial_number=self.serial_number, channel_index=self.channel_index,
            disable_brs=True)
        self.transport = moteus.Transport([device])
        self.controllers = {
            motor_id: moteus.Controller(
                id=motor_id, transport=self.transport,
                query_resolution=self.query_resolution)
            for motor_id in self.ids
        }
        self.open_count += 1

    async def cycle(self, targets_rev, safety, **kwargs):
        # USB bulk round-trips for three FD frames can exceed the 50 ms
        # SocketCAN-oriented default.  Keep this below the feedback timeout.
        safety = dataclasses.replace(
            safety, cycle_timeout_s=max(0.2, safety.cycle_timeout_s))
        return await super().cycle(targets_rev, safety, **kwargs)


class SocketCanBackend(MoteusBackend):
    def __init__(self, ids: Iterable[int], channel: str = "can1", **kwargs):
        super().__init__(ids, simulation=False, channel=channel, **kwargs)


def feedback_position(values: dict[int, Any]) -> float:
    value = float(values.get(Register.POSITION, math.nan))
    return value


def feedback_mode_name(values: dict[int, Any]) -> str:
    value = values.get(Register.MODE)
    if value is None:
        return "---"
    try:
        return Mode(int(value)).name
    except ValueError:
        return str(value)


_FAULT_DESCRIPTIONS = {
    32: "calibration fault", 33: "motor driver fault", 34: "over voltage",
    35: "encoder fault", 36: "motor not configured", 37: "pwm cycle overrun",
    38: "over temperature", 39: "outside limit", 40: "under voltage",
    41: "config changed", 42: "theta invalid", 43: "position invalid",
    44: "driver enable fault", 45: "stop position deprecated",
    46: "timing violation", 47: "bemf feedforward without accel",
    48: "invalid limits", 96: "limit: servo.max_velocity",
    97: "limit: servo.max_power_W", 98: "limit: max system voltage",
    99: "limit: servo.max_current_A", 100: "limit: servo.fault_temperature",
    101: "limit: servo.motor_fault_temperature",
    102: "limit: commanded max torque", 103: "limit: servopos limit",
    104: "limit: flux braking",
}


def fault_description(value: Any) -> str:
    code = int(value or 0)
    description = _FAULT_DESCRIPTIONS.get(code)
    return str(code) if description is None else f"{code} ({description})"
