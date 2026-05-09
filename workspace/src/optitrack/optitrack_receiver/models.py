from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple


@dataclass(frozen=True)
class RigidBodySample:
    """Single rigid body sample from a MoCap frame (NatNet stream)."""

    body_id: int
    position_m: Tuple[float, float, float]
    quaternion_xyzw: Tuple[float, float, float, float]
    marker_error: float
    tracking_valid: bool


@dataclass(frozen=True)
class MoCapFrame:
    """Decoded motion capture frame (rigid bodies only are surfaced for now)."""

    frame_number: int
    rigid_bodies: List[RigidBodySample]
