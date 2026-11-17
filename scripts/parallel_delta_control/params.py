"""Configuration models and persistence for the parallel mechanism controller."""

from __future__ import annotations

import dataclasses
import json
import math
import os
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping


AGILE_WRIST_BETA_DEG = math.degrees(math.acos(1.0 / math.sqrt(3.0)))


class ConfigError(ValueError):
    """A configuration file or update is invalid."""


def _finite(name: str, value: float, lo: float | None = None,
            hi: float | None = None, positive: bool = False) -> None:
    value = float(value)
    if not math.isfinite(value):
        raise ConfigError(f"{name} 必须是有限数")
    if positive and value <= 0:
        raise ConfigError(f"{name} 必须大于 0")
    if lo is not None and value < lo:
        raise ConfigError(f"{name} 不能小于 {lo}")
    if hi is not None and value > hi:
        raise ConfigError(f"{name} 不能大于 {hi}")


def _keys(data: Mapping[str, Any], cls: type) -> dict[str, Any]:
    if not isinstance(data, Mapping):
        raise ConfigError(f"{cls.__name__} 必须是对象")
    allowed = {f.name for f in dataclasses.fields(cls)}
    unknown = sorted(set(data) - allowed)
    if unknown:
        raise ConfigError(f"{cls.__name__} 含未知字段: {', '.join(unknown)}")
    return dict(data)


def _triple(name: str, values: Any) -> tuple[float, float, float]:
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        raise ConfigError(f"{name} 必须包含三个数")
    result = tuple(float(v) for v in values)
    for i, value in enumerate(result):
        _finite(f"{name}[{i}]", value)
    return result  # type: ignore[return-value]


@dataclass
class MechanismParams:
    """Ideal coaxial 3-RRR spherical mechanism; lengths are millimetres."""

    H: float = 85.0
    D: float = 80.0
    e: float = 0.0
    alpha1_deg: float = 45.0
    alpha2_deg: float = 60.0
    beta_deg: float = 45.0
    h_mid: float = 15.0
    d_stack: float = 7.0
    column_radius: float = 14.0
    gear_radius: float = 28.0
    pinion_radius: float = 8.0
    rod_diameter: float = 8.0
    motor_radius: float = 14.0

    @property
    def R(self) -> float:
        return 0.5 * self.D

    @property
    def sphere_center(self) -> tuple[float, float, float]:
        return (self.e, 0.0, self.H)

    @property
    def motor_heights(self) -> tuple[float, float, float]:
        # Hardware drawing: M1 bottom, M2 middle, M3 top.
        return (self.h_mid - self.d_stack, self.h_mid, self.h_mid + self.d_stack)

    @property
    def r_platform(self) -> float:
        return self.R * math.tan(math.radians(self.beta_deg))

    def validate(self) -> None:
        for name in ("H", "D", "h_mid", "d_stack", "column_radius",
                     "gear_radius", "pinion_radius", "rod_diameter", "motor_radius"):
            _finite(name, getattr(self, name), positive=True)
        _finite("e", self.e, lo=0.0)
        _finite("alpha1_deg", self.alpha1_deg, 20.0, 160.0)
        _finite("alpha2_deg", self.alpha2_deg, 20.0, 160.0)
        _finite("beta_deg", self.beta_deg, 15.0, 80.0)
        if self.e > self.R:
            raise ConfigError("e 不能大于任务球半径 D/2")
        if self.H - self.R <= self.h_mid + self.d_stack:
            raise ConfigError("平台球面最低点必须高于同轴齿轮顶层")
        if self.column_radius >= self.gear_radius:
            raise ConfigError("column_radius 必须小于 gear_radius")
        if self.d_stack >= self.h_mid:
            raise ConfigError("d_stack 必须小于 h_mid")

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "MechanismParams":
        result = cls(**_keys(data, cls))
        result.validate()
        return result


@dataclass
class TransmissionParams:
    """Mapping between moteus motor-shaft revolutions and coaxial shaft angles."""

    motor_ids: tuple[int, int, int] = (1, 2, 3)
    directions: tuple[int, int, int] = (1, 1, 1)
    motor_reduction: float = 8.0
    belt_reduction: float = 2.0

    @property
    def total_reduction(self) -> float:
        return self.motor_reduction * self.belt_reduction

    def validate(self) -> None:
        if len(self.motor_ids) != 3 or len(set(self.motor_ids)) != 3:
            raise ConfigError("motor_ids 必须是三个互不相同的 ID")
        if any(isinstance(v, bool) or not isinstance(v, int) or not 1 <= v <= 126
               for v in self.motor_ids):
            raise ConfigError("motor_ids 必须是 1…126 的整数")
        if len(self.directions) != 3 or any(v not in (-1, 1) for v in self.directions):
            raise ConfigError("directions 的三项只能是 +1 或 -1")
        _finite("motor_reduction", self.motor_reduction, positive=True)
        _finite("belt_reduction", self.belt_reduction, positive=True)

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "TransmissionParams":
        values = _keys(data, cls)
        if "motor_ids" in values:
            values["motor_ids"] = tuple(values["motor_ids"])
        if "directions" in values:
            values["directions"] = tuple(values["directions"])
        result = cls(**values)
        result.validate()
        return result


@dataclass
class SafetyParams:
    """Software limits. These do not replace mechanical stops or an E-stop."""

    roll_limit_deg: float = 15.0
    pitch_limit_deg: float = 15.0
    yaw_limit_deg: float = 60.0
    # Feedback may slightly overshoot a command boundary; targets remain strict.
    feedback_rpy_margin_deg: float = 1.0
    motor_min_deg: tuple[float, float, float] = (-1300.0, -1300.0, -1300.0)
    motor_max_deg: tuple[float, float, float] = (1300.0, 1300.0, 1300.0)
    max_move_deg: float = 2600.0
    velocity_limit_rev_s: float = 0.5
    accel_limit_rev_s2: float = 2.0
    maximum_torque_nm: float | None = None
    kp_scale: float | None = None
    kd_scale: float | None = None
    watchdog_timeout_s: float = 0.25
    feedback_timeout_s: float = 0.5
    cycle_timeout_s: float = 0.05
    control_rate_hz: float = 100.0
    state_publish_hz: float = 20.0
    min_condition: float = 0.05
    fk_tolerance: float = 1e-10
    fk_max_iterations: int = 30
    fk_path_step_deg: float = 2.0
    max_pose_step_deg: float = 30.0
    max_feedback_step_deg: float = 30.0
    command_complete_tol_deg: float = 1.0

    def validate(self) -> None:
        for name in ("roll_limit_deg", "pitch_limit_deg", "yaw_limit_deg",
                     "max_move_deg", "velocity_limit_rev_s", "accel_limit_rev_s2",
                     "watchdog_timeout_s", "feedback_timeout_s", "cycle_timeout_s",
                     "control_rate_hz", "state_publish_hz", "fk_tolerance",
                     "fk_path_step_deg", "max_pose_step_deg", "max_feedback_step_deg",
                     "command_complete_tol_deg"):
            _finite(name, getattr(self, name), positive=True)
        _finite("feedback_rpy_margin_deg", self.feedback_rpy_margin_deg, lo=0.0)
        _finite("min_condition", self.min_condition, 0.0, 1.0)
        if isinstance(self.fk_max_iterations, bool) or not isinstance(self.fk_max_iterations, int) \
                or self.fk_max_iterations < 1:
            raise ConfigError("fk_max_iterations 必须是正整数")
        self.motor_min_deg = _triple("motor_min_deg", self.motor_min_deg)
        self.motor_max_deg = _triple("motor_max_deg", self.motor_max_deg)
        for i, (lo, hi) in enumerate(zip(self.motor_min_deg, self.motor_max_deg)):
            if lo >= hi:
                raise ConfigError(f"电机 {i + 1} 的 soft limit 下限必须小于上限")
            if not lo <= 0.0 <= hi:
                raise ConfigError(f"电机 {i + 1} 软限位必须包含直立零位 0°")
        for name in ("maximum_torque_nm",):
            value = getattr(self, name)
            if value is not None:
                _finite(name, value, positive=True)
        for name in ("kp_scale", "kd_scale"):
            value = getattr(self, name)
            if value is not None:
                _finite(name, value, lo=0.0)

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "SafetyParams":
        result = cls(**_keys(data, cls))
        result.validate()
        return result


@dataclass
class CalibrationParams:
    """Absolute working current plus an independent, higher protection limit."""

    velocity_rev_s: float = 0.5
    accel_rev_s2: float = 0.5
    drive_current_a: float = 0.5
    motor_kv: float | None = None
    firmware_max_current_a: float | None = None
    max_q_current_a: float = 2.0
    stall_velocity_rev_s: float = 0.02
    stall_position_tol_deg: float = 1.0
    stable_s: float = 1.0
    stage_timeout_s: float = 20.0
    min_span_rev: float = 0.1
    max_span_imbalance: float = 0.1
    return_tol_deg: float = 2.0

    @property
    def current_limit_scale(self) -> float | None:
        if self.firmware_max_current_a is None:
            return None
        return self.drive_current_a / self.firmware_max_current_a

    @property
    def drive_torque_nm(self) -> float | None:
        """Moteus output torque for the requested current at ratio=1."""
        if self.motor_kv is None:
            return None
        torque_constant = (1.5 / math.sqrt(3.0)) * (60.0 / (2.0 * math.pi)) / self.motor_kv
        return self.drive_current_a * torque_constant

    def validate(self) -> None:
        for name in ("velocity_rev_s", "accel_rev_s2", "drive_current_a",
                     "max_q_current_a", "stall_velocity_rev_s",
                     "stall_position_tol_deg", "stable_s", "stage_timeout_s",
                     "min_span_rev", "return_tol_deg"):
            _finite(f"calibration.{name}", getattr(self, name), positive=True)
        if self.motor_kv is not None:
            _finite("calibration.motor_kv", self.motor_kv, positive=True)
        if self.firmware_max_current_a is not None:
            _finite("calibration.firmware_max_current_a",
                    self.firmware_max_current_a, positive=True)
            if self.firmware_max_current_a < self.drive_current_a:
                raise ConfigError("固件电流限值不能小于标定工作电流")
        if self.max_q_current_a <= self.drive_current_a:
            raise ConfigError("标定保护电流 max_q_current_a 必须大于工作电流 drive_current_a")
        _finite("calibration.max_span_imbalance", self.max_span_imbalance,
                lo=0.000001, hi=1.0)

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "CalibrationParams":
        result = cls(**_keys(data, cls))
        result.validate()
        return result


@dataclass
class AppConfig:
    version: int = 1
    mechanism: MechanismParams = field(default_factory=MechanismParams)
    transmission: TransmissionParams = field(default_factory=TransmissionParams)
    safety: SafetyParams = field(default_factory=SafetyParams)
    calibration: CalibrationParams = field(default_factory=CalibrationParams)

    def validate(self) -> None:
        if self.version != 1:
            raise ConfigError(f"不支持的配置版本 {self.version!r}")
        self.mechanism.validate()
        self.transmission.validate()
        self.safety.validate()
        self.calibration.validate()

    def to_dict(self) -> dict[str, Any]:
        return dataclasses.asdict(self)

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "AppConfig":
        values = _keys(data, cls)
        if "mechanism" in values:
            values["mechanism"] = MechanismParams.from_dict(values["mechanism"])
        if "transmission" in values:
            values["transmission"] = TransmissionParams.from_dict(values["transmission"])
        if "safety" in values:
            values["safety"] = SafetyParams.from_dict(values["safety"])
        if "calibration" in values:
            values["calibration"] = CalibrationParams.from_dict(values["calibration"])
        result = cls(**values)
        result.validate()
        return result


def load_config(path: str | os.PathLike[str]) -> AppConfig:
    config_path = Path(path)
    if not config_path.exists():
        return AppConfig()
    try:
        with config_path.open("r", encoding="utf-8") as stream:
            data = json.load(stream)
    except (OSError, json.JSONDecodeError) as exc:
        raise ConfigError(f"无法读取配置 {config_path}: {exc}") from exc
    return AppConfig.from_dict(data)


def save_config(config: AppConfig, path: str | os.PathLike[str]) -> None:
    config.validate()
    config_path = Path(path)
    config_path.parent.mkdir(parents=True, exist_ok=True)
    temporary: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
                mode="w", encoding="utf-8", dir=config_path.parent,
                prefix=config_path.name + ".", suffix=".tmp", delete=False) as stream:
            temporary = stream.name
            json.dump(config.to_dict(), stream, ensure_ascii=False, indent=2,
                      allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, config_path)
    finally:
        if temporary is not None and os.path.exists(temporary):
            os.unlink(temporary)
