"""Continuous FK/IK and motor-shaft/transmission conversions."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

import numpy as np

from mechanism import (Q_HOME, Q_HOME_DEG, elbow_axes, geometry_snapshot,
                       inverse_kinematics, jacobian_condition, platform_axes_local)
from params import AppConfig, MechanismParams, SafetyParams, TransmissionParams
from pose import exp_rotation, matrix_to_rpy, rpy_to_matrix


class KinematicsError(ValueError):
    """A pose is invalid, unreachable, singular, or on a discontinuous branch."""


@dataclass
class ForwardResult:
    motor_deg: np.ndarray
    q: np.ndarray
    rotation: np.ndarray
    rpy_deg: np.ndarray
    path_rpy_max_deg: np.ndarray
    condition: float
    residual: float
    iterations: int
    path_steps: int
    geometry: dict[str, np.ndarray]


@dataclass
class InverseResult:
    rpy_deg: np.ndarray
    rotation: np.ndarray
    q: np.ndarray
    motor_deg: np.ndarray
    condition: float
    branches: np.ndarray
    geometry: dict[str, np.ndarray]


def _vector3(values: Iterable[float], name: str) -> np.ndarray:
    result = np.asarray(list(values), dtype=float).reshape(3)
    if not np.all(np.isfinite(result)):
        raise KinematicsError(f"{name} 必须是三个有限数")
    return result


def check_rpy_limits(rpy_deg: np.ndarray, safety: SafetyParams,
                     margin_deg: float | np.ndarray = 0.0) -> None:
    limits = np.array([safety.roll_limit_deg, safety.pitch_limit_deg,
                       safety.yaw_limit_deg]) + np.asarray(margin_deg, dtype=float)
    if np.any(np.abs(rpy_deg) > limits + 1e-7):
        raise KinematicsError(
            f"RPY {np.round(rpy_deg, 6).tolist()}° 超出允许范围 ±{limits.tolist()}°")


def motor_deg_to_q(motor_deg: Iterable[float], transmission: TransmissionParams) -> np.ndarray:
    motor = _vector3(motor_deg, "motor_deg")
    return Q_HOME + np.radians(motor) / transmission.total_reduction


def q_to_motor_deg(q: np.ndarray, transmission: TransmissionParams) -> np.ndarray:
    q = _vector3(q, "q")
    return np.degrees(q - Q_HOME) * transmission.total_reduction


def raw_rev_to_motor_deg(raw_rev: Iterable[float], zero_rev: np.ndarray,
                         directions: np.ndarray) -> np.ndarray:
    raw = _vector3(raw_rev, "raw_rev")
    return np.asarray(directions) * (raw - np.asarray(zero_rev)) * 360.0


def motor_deg_to_raw_rev(motor_deg: Iterable[float], zero_rev: np.ndarray,
                         directions: np.ndarray) -> np.ndarray:
    motor = _vector3(motor_deg, "motor_deg")
    return np.asarray(zero_rev) + np.asarray(directions) * motor / 360.0


def _rotation_distance_deg(a: np.ndarray, b: np.ndarray) -> float:
    relative = np.asarray(a) @ np.asarray(b).T
    cosine = float(np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def _newton_fk(params: MechanismParams, q: np.ndarray, initial_rotation: np.ndarray,
               tolerance: float, max_iterations: int) -> tuple[np.ndarray, float, int]:
    """Solve f=w·v-cos(alpha2), using A*delta=f and R'=exp(delta)R."""
    R = np.asarray(initial_rotation, dtype=float).reshape(3, 3).copy()
    v_local = platform_axes_local(params)
    alpha2 = math.radians(params.alpha2_deg)
    residual = math.inf
    for iteration in range(max_iterations + 1):
        v = v_local @ R.T
        w = elbow_axes(params, q)
        f = np.sum(w * v, axis=1) - math.cos(alpha2)
        residual = float(np.linalg.norm(f, ord=np.inf))
        if not math.isfinite(residual):
            raise KinematicsError("FK 残差不是有限数")
        if residual <= tolerance:
            return R, residual, iteration
        if iteration == max_iterations:
            break
        A = np.stack([np.cross(w[i], v[i]) for i in range(3)])
        try:
            delta = np.linalg.solve(A, f)
        except np.linalg.LinAlgError as exc:
            raise KinematicsError("FK 遇到奇异 Jacobian") from exc
        if not np.all(np.isfinite(delta)):
            raise KinematicsError("FK 迭代步长不是有限数")
        angle = float(np.linalg.norm(delta))
        if angle > 0.25:
            delta *= 0.25 / angle
        R = exp_rotation(delta) @ R
    raise KinematicsError(
        f"FK 在 {max_iterations} 次迭代内未收敛，残差 {residual:.3e}")


def forward_kinematics(config: AppConfig, motor_deg: Iterable[float],
                       initial_motor_deg: Iterable[float] | None = None,
                       initial_rotation: np.ndarray | None = None,
                       enforce_limits: bool = True,
                       rpy_margin_deg: float | np.ndarray = 0.0) -> ForwardResult:
    """FK with path continuation from a known assembly branch.

    If no initial state is supplied, the upright home branch is used.  Path
    steps enforce convergence, conditioning and branch continuity.  The strict
    RPY envelope applies to the final target, not to numerical substeps.
    """
    motor = _vector3(motor_deg, "motor_deg")
    q_target = motor_deg_to_q(motor, config.transmission)
    if initial_motor_deg is None:
        initial_motor = np.zeros(3)
    else:
        initial_motor = _vector3(initial_motor_deg, "initial_motor_deg")
    q_start = motor_deg_to_q(initial_motor, config.transmission)
    R = np.eye(3) if initial_rotation is None else np.asarray(
        initial_rotation, dtype=float).reshape(3, 3).copy()
    safety = config.safety
    max_delta = float(np.max(np.abs(q_target - q_start)))
    steps = max(1, int(math.ceil(math.degrees(max_delta) / safety.fk_path_step_deg)))
    total_iterations = 0
    residual = 0.0
    condition = 0.5
    previous_R = R.copy()
    path_rpy_max = np.abs(matrix_to_rpy(R))
    for index, alpha in enumerate(np.linspace(0.0, 1.0, steps + 1)[1:], start=1):
        q = q_start + alpha * (q_target - q_start)
        R, residual, iterations = _newton_fk(
            config.mechanism, q, R, safety.fk_tolerance, safety.fk_max_iterations)
        total_iterations += iterations
        if _rotation_distance_deg(R, previous_R) > safety.max_pose_step_deg:
            raise KinematicsError(f"FK 路径第 {index} 步出现装配分支跳变")
        previous_R = R.copy()
        rpy = matrix_to_rpy(R)
        path_rpy_max = np.maximum(path_rpy_max, np.abs(rpy))
        w = elbow_axes(config.mechanism, q)
        condition = jacobian_condition(config.mechanism, R, w)
        if condition < safety.min_condition:
            raise KinematicsError(
                f"FK 路径第 {index} 步接近奇异：条件指标 {condition:.4f} < "
                f"{safety.min_condition:.4f}")
        # Intermediate poses are numerical continuation steps, not separate
        # physical target commands.  Only the final target uses the strict RPY
        # command envelope; actual feedback gets its own configurable margin.
        if enforce_limits and index == steps:
            check_rpy_limits(rpy, safety, rpy_margin_deg)
    rpy = matrix_to_rpy(R)
    return ForwardResult(
        motor_deg=motor,
        q=q_target,
        rotation=R,
        rpy_deg=rpy,
        path_rpy_max_deg=path_rpy_max,
        condition=condition,
        residual=residual,
        iterations=total_iterations,
        path_steps=steps,
        geometry=geometry_snapshot(config.mechanism, q_target, R),
    )


def inverse_from_rpy(config: AppConfig, rpy_deg: Iterable[float],
                     reference_motor_deg: Iterable[float] | None = None,
                     enforce_limits: bool = True) -> InverseResult:
    rpy = _vector3(rpy_deg, "rpy_deg")
    if enforce_limits:
        check_rpy_limits(rpy, config.safety)
    R = rpy_to_matrix(rpy)
    reference_q = None if reference_motor_deg is None else motor_deg_to_q(
        reference_motor_deg, config.transmission)
    try:
        q, w, branches = inverse_kinematics(config.mechanism, R, reference_q)
    except ValueError as exc:
        raise KinematicsError(str(exc)) from exc
    condition = jacobian_condition(config.mechanism, R, w)
    if condition < config.safety.min_condition:
        raise KinematicsError(
            f"目标接近奇异：条件指标 {condition:.4f} < {config.safety.min_condition:.4f}")
    motor = q_to_motor_deg(q, config.transmission)
    return InverseResult(
        rpy_deg=rpy,
        rotation=R,
        q=q,
        motor_deg=motor,
        condition=condition,
        branches=branches,
        geometry=geometry_snapshot(config.mechanism, q, R),
    )


def verify_inverse(config: AppConfig, inverse: InverseResult,
                   current_motor_deg: Iterable[float], current_rotation: np.ndarray
                   ) -> ForwardResult:
    """Path-continued FK verification of an IK command on the current branch."""
    result = forward_kinematics(
        config, inverse.motor_deg, current_motor_deg, current_rotation)
    error = float(np.max(np.abs(result.rotation - inverse.rotation)))
    if error > 1e-7:
        raise KinematicsError(f"IK/FK 装配分支回验失败，矩阵误差 {error:.3e}")
    return result


def home_result(config: AppConfig) -> ForwardResult:
    return forward_kinematics(config, np.zeros(3), enforce_limits=False)


def rpy_rate_to_raw_velocity(config: AppConfig, actual: ForwardResult,
                             rpy_rate_deg_s: Iterable[float]) -> np.ndarray:
    """Convert Roll/Pitch/Yaw rates to raw motor rev/s using the local Jacobian."""
    roll, pitch, yaw = np.radians(_vector3(rpy_rate_deg_s, "rpy_rate_deg_s"))
    # Standard active ZYX body-rate mapping, evaluated at the current RPY.
    current_rpy = np.radians(actual.rpy_deg)
    cr, sr = math.cos(current_rpy[0]), math.sin(current_rpy[0])
    cp, sp = math.cos(current_rpy[1]), math.sin(current_rpy[1])
    body_from_rpy = np.array([
        [1.0, 0.0, -sp],
        [0.0, cr, sr * cp],
        [0.0, -sr, cr * cp],
    ])
    body_omega = body_from_rpy @ np.array([roll, pitch, yaw])
    world_omega = actual.rotation @ body_omega
    w = np.asarray(actual.geometry["w"], dtype=float)
    v = np.asarray(actual.geometry["v"], dtype=float)
    A = np.stack([np.cross(w[i], v[i]) for i in range(3)])
    B = np.zeros((3, 3))
    for i in range(3):
        u = np.array([0.0, 0.0, 1.0])
        B[i, i] = float(np.dot(np.cross(u, w[i]), v[i]))
    try:
        q_dot = np.linalg.solve(B, A) @ world_omega
    except np.linalg.LinAlgError as exc:
        raise KinematicsError("当前姿态速度 Jacobian 奇异，无法执行 RPY 速度命令") from exc
    if not np.all(np.isfinite(q_dot)):
        raise KinematicsError("RPY 速度转换结果无效")
    motor_deg_s = q_dot * config.transmission.total_reduction * 180.0 / math.pi
    return np.asarray(config.transmission.directions, dtype=float) * motor_deg_s / 360.0


def home_shaft_deg() -> np.ndarray:
    return Q_HOME_DEG.copy()
