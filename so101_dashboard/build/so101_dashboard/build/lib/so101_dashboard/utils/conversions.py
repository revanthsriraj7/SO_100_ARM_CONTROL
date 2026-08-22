"""Small, dependency-free conversion and validation helpers.

Kept separate from any ROS or Qt imports so it can be unit tested in
isolation (see test/test_conversions.py).
"""
from __future__ import annotations

import math
from typing import Dict, Iterable, List, Optional, Sequence


def rad_to_deg(value: float) -> float:
    return value * 180.0 / math.pi


def deg_to_rad(value: float) -> float:
    return value * math.pi / 180.0


def is_finite(value: float) -> bool:
    """True if value is a real, finite number (not NaN, not +/-inf)."""
    return isinstance(value, (int, float)) and math.isfinite(value)


def all_finite(values: Iterable[float]) -> bool:
    return all(is_finite(v) for v in values)


def joint_state_to_dict(
    joint_names: Sequence[str],
    msg_names: Sequence[str],
    msg_values: Sequence[float],
) -> Dict[str, float]:
    """Map a sensor_msgs/JointState-style (names, values) pair onto the
    subset of joints this application cares about (joint_names), by name
    -- not by index, since a JointState message is not guaranteed to list
    joints in the same order as our config, or to include every joint we
    care about (e.g. still starting up).
    """
    lookup = dict(zip(msg_names, msg_values))
    return {name: lookup[name] for name in joint_names if name in lookup}


def has_all_joints(data: Dict[str, float], joint_names: Sequence[str]) -> bool:
    return all(name in data for name in joint_names)


def missing_joints(data: Dict[str, float], joint_names: Sequence[str]) -> List[str]:
    return [name for name in joint_names if name not in data]


def within_limits(
    position: float, min_position: Optional[float], max_position: Optional[float]
) -> bool:
    if min_position is not None and position < min_position:
        return False
    if max_position is not None and position > max_position:
        return False
    return True


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def trapezoidal_duration(distance: float, max_velocity: float, max_acceleration: float) -> float:
    """Minimum time (s) to cover `distance` under a symmetric trapezoidal
    velocity profile (ramp up at max_acceleration, cruise at max_velocity,
    ramp down), or a triangular profile (never reaches max_velocity) if
    the distance is too short to complete a full ramp.

    Standard motion-profile formulas:
      ramp_distance = v_max^2 / a_max        (distance covered by ramp up + ramp down)
      trapezoidal:  t = d/v_max + v_max/a_max         (d >= ramp_distance)
      triangular:   t = 2 * sqrt(d / a_max)           (d <  ramp_distance)
    """
    distance = abs(distance)
    if distance <= 0.0 or max_velocity <= 0.0 or max_acceleration <= 0.0:
        return 0.0
    ramp_distance = (max_velocity ** 2) / max_acceleration
    if distance >= ramp_distance:
        return distance / max_velocity + max_velocity / max_acceleration
    return 2.0 * math.sqrt(distance / max_acceleration)
