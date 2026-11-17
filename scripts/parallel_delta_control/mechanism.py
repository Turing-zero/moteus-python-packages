"""Geometry and analytic IK for a coaxial-drive 3-RRR spherical mechanism.

Derived from parallel_delta/coaxial_delta/spm.py and geometry.py.  The leg
ordering, negative hinge-axis convention, and home branch are intentionally
preserved.  All angles in this module are radians unless explicitly suffixed.
"""

from __future__ import annotations

import math

import numpy as np

from params import MechanismParams
from pose import SphericalPose, platform_center, spherical_to_matrix


N_LEGS = 3
# Output shafts M1 -> M2 -> M3 step clockwise by 120 deg: roots/Bw are
# 210/90/330 deg.  Each output-to-platform connection remains +90 deg CCW.
ETA_MOTOR = np.radians([0.0, 240.0, 120.0])
ETA_V = np.radians([120.0, 0.0, 240.0])
Q_HOME_DEG = np.array([30.0, 270.0, 150.0])
Q_HOME = np.radians(Q_HOME_DEG)


def _vector3(value: np.ndarray | list[float] | tuple[float, ...], name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float).reshape(3)
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} 必须是三个有限数")
    return result


def wrap_pi(angle: float | np.ndarray) -> float | np.ndarray:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def unwrap_near(angle: np.ndarray, reference: np.ndarray) -> np.ndarray:
    """Add whole turns to angle, staying closest to the continuous reference."""
    return np.asarray(angle) + 2.0 * np.pi * np.round(
        (np.asarray(reference) - np.asarray(angle)) / (2.0 * np.pi))


def rotation_center(params: MechanismParams) -> np.ndarray:
    return np.asarray(params.sphere_center, dtype=float)


def base_axes(params: MechanismParams) -> np.ndarray:
    _ = params
    return np.tile(np.array([0.0, 0.0, 1.0]), (N_LEGS, 1))


def platform_axes_local(params: MechanismParams) -> np.ndarray:
    beta = math.radians(params.beta_deg)
    sb, cb = math.sin(beta), math.cos(beta)
    return np.stack([
        np.array([sb * math.cos(eta), sb * math.sin(eta), cb]) for eta in ETA_V
    ])


def _leg_frame(u: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    z = np.array([0.0, 0.0, 1.0])
    x = z - float(np.dot(z, u)) * u
    norm = float(np.linalg.norm(x))
    if norm < 1e-9:
        x = np.array([1.0, 0.0, 0.0])
        if abs(float(np.dot(x, u))) > 0.9:
            x = np.array([0.0, 1.0, 0.0])
        x = x - float(np.dot(x, u)) * u
    x = x / np.linalg.norm(x)
    y = np.cross(u, x)
    return x, y / np.linalg.norm(y)


def intermediate_axis(u: np.ndarray, q: float, alpha1: float) -> np.ndarray:
    x, y = _leg_frame(u)
    return math.cos(alpha1) * u + math.sin(alpha1) * (math.cos(q) * x + math.sin(q) * y)


def elbow_axes(params: MechanismParams, q: np.ndarray) -> np.ndarray:
    q = _vector3(q, "q")
    u = base_axes(params)
    alpha1 = math.radians(params.alpha1_deg)
    return np.stack([intermediate_axis(u[i], q[i], alpha1) for i in range(3)])


def inverse_leg(u: np.ndarray, v: np.ndarray,
                alpha1: float, alpha2: float) -> list[float]:
    """Two working-mode solutions for one spherical RRR leg."""
    x, y = _leg_frame(u)
    a = math.sin(alpha1) * float(np.dot(x, v))
    b = math.sin(alpha1) * float(np.dot(y, v))
    c = math.cos(alpha2) - math.cos(alpha1) * float(np.dot(u, v))
    radius = math.hypot(a, b)
    if radius < 1e-12:
        return []
    if abs(c) > radius + 1e-9:
        return []
    c = float(np.clip(c, -radius, radius))
    phi = math.atan2(b, a)
    delta = math.acos(c / radius)
    return [phi + delta, phi - delta]


def inverse_kinematics(params: MechanismParams, rotation: np.ndarray,
                       reference_q: np.ndarray | None = None
                       ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return continuous q, elbow axes w, and chosen branch indices.

    The two analytic roots are first selected by wrapped distance to reference,
    then unwrapped near reference so a command never jumps by a whole turn.
    """
    reference = Q_HOME.copy() if reference_q is None else _vector3(reference_q, "reference_q")
    v_local = platform_axes_local(params)
    v = v_local @ np.asarray(rotation, dtype=float).T
    u = base_axes(params)
    alpha1 = math.radians(params.alpha1_deg)
    alpha2 = math.radians(params.alpha2_deg)
    q = np.full(3, np.nan)
    branches = np.full(3, -1, dtype=int)
    for i in range(N_LEGS):
        solutions = inverse_leg(u[i], v[i], alpha1, alpha2)
        if not solutions:
            raise ValueError(f"支链 {i + 1} 在当前姿态无解析逆解")
        wrapped = np.asarray([wrap_pi(s) for s in solutions])
        branch = int(np.argmin(np.abs(wrap_pi(wrapped - reference[i]))))
        branches[i] = branch
        q[i] = unwrap_near(wrapped[branch], reference[i])
    return q, elbow_axes(params, q), branches


def joint_radii(params: MechanismParams) -> tuple[np.ndarray, float, float]:
    heights = np.asarray(params.motor_heights, dtype=float)
    rho_u = np.maximum(params.H - heights, 1.0)
    cb = max(math.cos(math.radians(params.beta_deg)), 1e-6)
    rho_v = params.R / cb
    rho_w = 0.5 * (float(np.mean(rho_u)) + rho_v)
    return rho_u, rho_w, rho_v


def hinge_points(params: MechanismParams, rotation: np.ndarray,
                 w: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    C = rotation_center(params)
    rho_u, rho_w, rho_v = joint_radii(params)
    u = base_axes(params)
    v = platform_axes_local(params) @ np.asarray(rotation).T
    Bu = C[None, :] - rho_u[:, None] * u
    Bw = C[None, :] - rho_w * w
    Bv = C[None, :] - rho_v * v
    return Bu, Bw, Bv


def arm_root_points(params: MechanismParams, w: np.ndarray) -> np.ndarray:
    xy = np.array([params.e, 0.0])
    heights = np.asarray(params.motor_heights, dtype=float)
    wxy = -np.asarray(w, dtype=float)[:, :2]
    norm = np.maximum(np.linalg.norm(wxy, axis=1, keepdims=True), 1e-12)
    direction = wxy / norm
    points = np.zeros((3, 3))
    points[:, 0] = xy[0] + params.gear_radius * direction[:, 0]
    points[:, 1] = xy[1] + params.gear_radius * direction[:, 1]
    points[:, 2] = heights
    return points


def jacobian_condition(params: MechanismParams, rotation: np.ndarray,
                       w: np.ndarray) -> float:
    """Source model conditioning index zeta=sigma_min(J)/sigma_max(J)."""
    u = base_axes(params)
    v = platform_axes_local(params) @ np.asarray(rotation).T
    A = np.stack([np.cross(w[i], v[i]) for i in range(3)])
    B = np.zeros((3, 3))
    for i in range(3):
        B[i, i] = float(np.dot(np.cross(u[i], w[i]), v[i]))
    try:
        J = np.linalg.solve(A, B)
        singular = np.linalg.svd(J, compute_uv=False)
    except np.linalg.LinAlgError:
        return 0.0
    if not np.all(np.isfinite(singular)) or singular[0] < 1e-12:
        return 0.0
    return float(singular[-1] / singular[0])


def geometry_snapshot(params: MechanismParams, q: np.ndarray,
                      rotation: np.ndarray) -> dict[str, np.ndarray]:
    w = elbow_axes(params, q)
    Bu, Bw, Bv = hinge_points(params, rotation, w)
    return {
        "C": rotation_center(params),
        "P": platform_center(params, rotation),
        "R": np.asarray(rotation, dtype=float),
        "u": base_axes(params),
        "w": w,
        "v": platform_axes_local(params) @ np.asarray(rotation).T,
        "Bu": Bu,
        "Bw": Bw,
        "Bv": Bv,
        "roots": arm_root_points(params, w),
        "heights": np.asarray(params.motor_heights, dtype=float),
    }


def spherical_geometry(params: MechanismParams, pose: SphericalPose,
                       reference_q: np.ndarray | None = None) -> dict[str, np.ndarray]:
    """Convenience adapter retaining the source model's spherical-pose API."""
    R = spherical_to_matrix(pose)
    q, _w, _branches = inverse_kinematics(params, R, reference_q)
    return geometry_snapshot(params, q, R)
