"""YAML load/save for recorded trajectories, in the format from the
project spec:

    trajectory:
      joint_names: [...]
      points:
        - time_from_start: 0.0
          positions: [...]
          velocities: [...]

No Qt here -- plain I/O, unit testable in isolation.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

import yaml


@dataclass
class TrajectoryPointData:
    time_from_start: float
    positions: List[float]
    velocities: Optional[List[float]] = None
    efforts: Optional[List[float]] = None


@dataclass
class TrajectoryData:
    name: str
    joint_names: List[str]
    points: List[TrajectoryPointData] = field(default_factory=list)
    sample_rate_hz: Optional[float] = None

    @property
    def duration(self) -> float:
        return self.points[-1].time_from_start if self.points else 0.0


def save_trajectory(path: str, trajectory: TrajectoryData) -> None:
    data = {
        'name': trajectory.name,
        'sample_rate_hz': trajectory.sample_rate_hz,
        'trajectory': {
            'joint_names': trajectory.joint_names,
            'points': [
                {
                    'time_from_start': p.time_from_start,
                    'positions': p.positions,
                    **({'velocities': p.velocities} if p.velocities is not None else {}),
                    **({'efforts': p.efforts} if p.efforts is not None else {}),
                }
                for p in trajectory.points
            ],
        },
    }
    with open(path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)


def load_trajectory(path: str) -> TrajectoryData:
    with open(path, 'r') as f:
        data = yaml.safe_load(f) or {}
    traj = data.get('trajectory', {})
    points = [
        TrajectoryPointData(
            time_from_start=p['time_from_start'],
            positions=list(p['positions']),
            velocities=list(p['velocities']) if 'velocities' in p else None,
            efforts=list(p['efforts']) if 'efforts' in p else None,
        )
        for p in traj.get('points', [])
    ]
    return TrajectoryData(
        name=data.get('name', 'trajectory'),
        joint_names=list(traj.get('joint_names', [])),
        points=points,
        sample_rate_hz=data.get('sample_rate_hz'),
    )
