"""Rotation conventions: active ZYX RPY and the source model's spherical pose."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class SphericalPose:
    """Source model coordinates, in radians; this is not roll/pitch/yaw."""

    theta: float = 0.0
    phi: float = 0.0
    psi: float = 0.0


def _vector3(name: str, values: np.ndarray | list[float] | tuple[float, ...]) -> np.ndarray:
    result = np.asarray(values, dtype=float).reshape(3)
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} 必须是三个有限数")
    return result


def rpy_to_matrix(rpy_deg: np.ndarray | list[float] | tuple[float, ...]) -> np.ndarray:
    """Active platform-to-world rotation Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    roll, pitch, yaw = np.radians(_vector3("rpy_deg", rpy_deg))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def matrix_to_rpy(rotation: np.ndarray) -> np.ndarray:
    """Return active ZYX roll/pitch/yaw in degrees."""
    R = np.asarray(rotation, dtype=float).reshape(3, 3)
    if not np.all(np.isfinite(R)):
        raise ValueError("旋转矩阵含非有限值")
    pitch = math.atan2(-R[2, 0], math.hypot(R[0, 0], R[1, 0]))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return np.degrees([roll, pitch, yaw])


def rotation_align_z(normal: np.ndarray) -> np.ndarray:
    """Minimum Rodrigues rotation mapping +Z onto normal."""
    n = np.asarray(normal, dtype=float).reshape(3)
    norm = float(np.linalg.norm(n))
    if norm < 1e-12 or not math.isfinite(norm):
        raise ValueError("平台法线无效")
    n = n / norm
    z = np.array([0.0, 0.0, 1.0])
    c = float(np.dot(z, n))
    if c > 1.0 - 1e-12:
        return np.eye(3)
    if c < -1.0 + 1e-12:
        return np.diag([1.0, -1.0, -1.0])
    v = np.cross(z, n)
    vx = np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
    return np.eye(3) + vx + vx @ vx * ((1.0 - c) / float(np.dot(v, v)))


def matrix_to_spherical(rotation: np.ndarray) -> SphericalPose:
    """Convert R to R0(theta,phi) @ Rz(psi), preserving standard RPY semantics."""
    R = np.asarray(rotation, dtype=float).reshape(3, 3)
    n = R[:, 2]
    theta = math.acos(float(np.clip(n[2], -1.0, 1.0)))
    # atan2(-0, -0) can otherwise produce an arbitrary +/-pi at the south pole.
    phi = 0.0 if theta < 1e-12 else math.atan2(-n[1], -n[0])
    R0 = rotation_align_z(n)
    M = R0.T @ R
    psi = math.atan2(M[1, 0], M[0, 0])
    return SphericalPose(theta, phi, psi)


def spherical_to_matrix(pose: SphericalPose) -> np.ndarray:
    n = np.array([
        -math.sin(pose.theta) * math.cos(pose.phi),
        -math.sin(pose.theta) * math.sin(pose.phi),
        math.cos(pose.theta),
    ])
    cz, sz = math.cos(pose.psi), math.sin(pose.psi)
    rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
    return rotation_align_z(n) @ rz


def platform_center(params, rotation: np.ndarray) -> np.ndarray:
    """The spherical mechanism has no independent XYZ translation."""
    C = np.asarray(params.sphere_center, dtype=float)
    return C - params.R * np.asarray(rotation, dtype=float)[:, 2]


def exp_rotation(delta: np.ndarray) -> np.ndarray:
    """Exponential map for a world-frame rotation increment."""
    d = np.asarray(delta, dtype=float).reshape(3)
    angle = float(np.linalg.norm(d))
    K = np.array([[0.0, -d[2], d[1]], [d[2], 0.0, -d[0]], [-d[1], d[0], 0.0]])
    if angle < 1e-12:
        return np.eye(3) + K
    return np.eye(3) + math.sin(angle) / angle * K + \
        (1.0 - math.cos(angle)) / (angle * angle) * (K @ K)
