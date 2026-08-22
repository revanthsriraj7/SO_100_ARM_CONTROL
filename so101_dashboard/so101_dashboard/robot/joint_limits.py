"""Joint limit loading and validation.

Loaded once from config/joint_limits.yaml (see that file for where the
numbers came from) and used to validate any position before it's sent to
the robot -- HOLD, HOME, waypoint playback, trajectory playback, and
manual joint targets all go through here first.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

import yaml

from ..utils.conversions import all_finite, trapezoidal_duration, within_limits


@dataclass(frozen=True)
class JointLimit:
    min_position: Optional[float]
    max_position: Optional[float]
    max_velocity: Optional[float] = None
    max_acceleration: Optional[float] = None
    max_effort: Optional[float] = None


class JointLimits:
    def __init__(self, limits: Dict[str, JointLimit]) -> None:
        self._limits: Dict[str, JointLimit] = limits

    @classmethod
    def from_yaml(cls, path: str) -> 'JointLimits':
        with open(path, 'r') as f:
            data = yaml.safe_load(f) or {}
        raw = data.get('joint_limits', {})
        limits = {}
        for name, entry in raw.items():
            limits[name] = JointLimit(
                min_position=entry.get('min_position'),
                max_position=entry.get('max_position'),
                max_velocity=entry.get('max_velocity'),
                max_acceleration=entry.get('max_acceleration'),
                max_effort=entry.get('max_effort'),
            )
        return cls(limits)

    def get(self, joint_name: str) -> Optional[JointLimit]:
        return self._limits.get(joint_name)

    def known_joints(self) -> List[str]:
        return list(self._limits.keys())

    def min_move_duration(
        self,
        joint_names: Sequence[str],
        from_positions: Sequence[float],
        to_positions: Sequence[float],
        velocity_scaling: float = 1.0,
        acceleration_scaling: float = 1.0,
    ) -> float:
        """Slowest-joint minimum time (s) to move from from_positions to
        to_positions without exceeding any joint's (scaled) velocity/
        acceleration limit, using a symmetric trapezoidal (or triangular,
        for short moves) velocity profile. Joints with no configured
        velocity/acceleration limit are skipped (contribute 0). All joints
        move synchronously to a single time_from_start, so the slowest
        joint sets the pace for the whole move.
        """
        slowest = 0.0
        for name, start, end in zip(joint_names, from_positions, to_positions):
            limit = self._limits.get(name)
            if limit is None or not limit.max_velocity or not limit.max_acceleration:
                continue
            v_max = limit.max_velocity * max(velocity_scaling, 0.001)
            a_max = limit.max_acceleration * max(acceleration_scaling, 0.001)
            slowest = max(slowest, trapezoidal_duration(end - start, v_max, a_max))
        return slowest

    def validate_position(self, joint_name: str, position: float) -> Optional[str]:
        """Return None if valid, else a human-readable reason it's invalid."""
        if not all_finite([position]):
            return f'{joint_name}: position is not finite ({position!r})'
        limit = self._limits.get(joint_name)
        if limit is None:
            return None  # unknown joint: nothing to validate against
        if not within_limits(position, limit.min_position, limit.max_position):
            return (
                f'{joint_name}: position {position:.4f} rad outside limits '
                f'[{limit.min_position}, {limit.max_position}]'
            )
        return None

    def validate_positions(
        self, joint_names: Sequence[str], positions: Sequence[float]
    ) -> List[str]:
        """Return a list of human-readable errors (empty list == all valid)."""
        errors: List[str] = []
        if len(joint_names) != len(positions):
            errors.append(
                f'joint_names/positions length mismatch: '
                f'{len(joint_names)} != {len(positions)}'
            )
            return errors
        for name, pos in zip(joint_names, positions):
            err = self.validate_position(name, pos)
            if err:
                errors.append(err)
        return errors
