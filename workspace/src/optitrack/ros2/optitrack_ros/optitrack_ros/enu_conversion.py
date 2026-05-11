"""NatNet / Motive world -> ENU (East, North, Up), then optional NED for PX4 wire format.

Motive often uses a Y-up right-handed world. ``z_up`` assumes the stream is already Z-up
(vertical +Z); only ``world_yaw_offset_deg`` aligns horizontal axes to ENU.

Quaternion convention:
- Input ``quaternion_xyzw`` matches geometry_msgs (x, y, z, w), passive body->mocap world.
- Internal ops use Hamilton (w, x, y, z).
- PX4 ``VehicleOdometry.q`` is Hamilton (w, x, y, z), passive body->reference frame.
"""

from __future__ import annotations

import math
from typing import Literal, Tuple

import numpy as np

VerticalAxisMode = Literal["z_up", "y_up"]

# Passive: v_world = R(q) @ v_body. Hamilton wxyz composition: R(q1@q2)=R(q1)R(q2).


def _quat_wxyz_from_xyzw(q_xyzw: Tuple[float, float, float, float]) -> np.ndarray:
    x, y, z, w = q_xyzw
    return np.array([w, x, y, z], dtype=np.float64)


def _quat_xyzw_from_wxyz(q: np.ndarray) -> Tuple[float, float, float, float]:
    w, x, y, z = q.tolist()
    return (x, y, z, w)


def _quat_normalize_wxyz(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return q / n


def _quat_mult_wxyz(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product q1 * q2 (same as R(q1) @ R(q2) for column vectors)."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=np.float64,
    )


def _quat_wxyz_from_matrix(R: np.ndarray) -> np.ndarray:
    """Rotation matrix (3x3) -> unit quaternion (w,x,y,z), passive."""
    m = R
    t = np.trace(m)
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return _quat_normalize_wxyz(np.array([w, x, y, z], dtype=np.float64))


def _rot_z_wxyz(yaw_rad: float) -> np.ndarray:
    c = math.cos(0.5 * yaw_rad)
    s = math.sin(0.5 * yaw_rad)
    return np.array([c, 0.0, 0.0, s], dtype=np.float64)


# Y-up (Motive-style) -> Z-up intermediate: p_out = R @ p_in with
# p_out = (x, -z, y) so old +Y (up) -> new +Z (up) for vector (0,1,0)->(0,0,1).
_R_YUP_TO_ZUP = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]], dtype=np.float64)
_Q_YUP_TO_ZUP = _quat_wxyz_from_matrix(_R_YUP_TO_ZUP)

# ENU (x=E,y=N,z=U) -> NED (x=N,y=E,z=D) at same origin: [n,e,d] = [e_y, e_x, -e_z]
_R_ENU_TO_NED = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]], dtype=np.float64)
_Q_ENU_TO_NED = _quat_wxyz_from_matrix(_R_ENU_TO_NED)


def moticap_to_enu(
    position_m: Tuple[float, float, float],
    quaternion_xyzw: Tuple[float, float, float, float],
    *,
    vertical_axis_mode: VerticalAxisMode,
    world_yaw_offset_deg: float,
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
    """Return (position_enu, quaternion_xyzw_enu) with passive body->ENU world."""
    p = np.array(position_m, dtype=np.float64)
    q_body_m = _quat_normalize_wxyz(_quat_wxyz_from_xyzw(quaternion_xyzw))

    if vertical_axis_mode == "y_up":
        p_z = _R_YUP_TO_ZUP @ p
        q_body_z = _quat_mult_wxyz(_Q_YUP_TO_ZUP, q_body_m)
    else:
        p_z = p.copy()
        q_body_z = q_body_m.copy()

    yaw_rad = math.radians(world_yaw_offset_deg)
    q_yaw = _rot_z_wxyz(yaw_rad)
    R_yaw = np.array(
        [
            [math.cos(yaw_rad), -math.sin(yaw_rad), 0.0],
            [math.sin(yaw_rad), math.cos(yaw_rad), 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    p_enu = R_yaw @ p_z
    q_body_enu = _quat_mult_wxyz(q_yaw, q_body_z)

    return (
        (float(p_enu[0]), float(p_enu[1]), float(p_enu[2])),
        _quat_xyzw_from_wxyz(_quat_normalize_wxyz(q_body_enu)),
    )


def enu_to_ned_px4(
    position_enu: Tuple[float, float, float],
    quaternion_xyzw_enu: Tuple[float, float, float, float],
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
    """Map ENU pose to PX4 NED local frame (POSE_FRAME_NED). Returns (pos_ned, q_xyzw_ned)."""
    p = np.array(position_enu, dtype=np.float64)
    p_ned = _R_ENU_TO_NED @ p
    q_enu = _quat_normalize_wxyz(_quat_wxyz_from_xyzw(quaternion_xyzw_enu))
    q_ned = _quat_mult_wxyz(_Q_ENU_TO_NED, q_enu)
    return (
        (float(p_ned[0]), float(p_ned[1]), float(p_ned[2])),
        _quat_xyzw_from_wxyz(_quat_normalize_wxyz(q_ned)),
    )


def quat_geometry_xyzw_to_px4_wxyz(
    quaternion_xyzw: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    """Return Hamilton (w, x, y, z) tuple for px4_msgs from geometry_msgs (x, y, z, w)."""
    q = _quat_normalize_wxyz(_quat_wxyz_from_xyzw(quaternion_xyzw))
    w, x, y, z = q.tolist()
    return (w, x, y, z)
