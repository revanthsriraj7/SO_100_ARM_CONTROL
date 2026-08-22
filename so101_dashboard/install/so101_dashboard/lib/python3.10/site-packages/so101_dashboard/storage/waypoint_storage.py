"""YAML load/save for waypoint lists. No Qt here -- plain I/O, unit
testable in isolation.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from typing import List

import yaml


@dataclass
class Waypoint:
    name: str
    timestamp: str
    joint_names: List[str]
    joint_positions: List[float]

    @classmethod
    def create(cls, name: str, joint_names: List[str], joint_positions: List[float]) -> 'Waypoint':
        return cls(
            name=name,
            timestamp=datetime.now().isoformat(),
            joint_names=list(joint_names),
            joint_positions=list(joint_positions),
        )


def save_waypoints(path: str, waypoints: List[Waypoint]) -> None:
    data = {
        'waypoints': [
            {
                'name': wp.name,
                'timestamp': wp.timestamp,
                'joint_names': wp.joint_names,
                'joint_positions': wp.joint_positions,
            }
            for wp in waypoints
        ]
    }
    with open(path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)


def load_waypoints(path: str) -> List[Waypoint]:
    with open(path, 'r') as f:
        data = yaml.safe_load(f) or {}
    result = []
    for entry in data.get('waypoints', []):
        result.append(Waypoint(
            name=entry['name'],
            timestamp=entry.get('timestamp', ''),
            joint_names=list(entry['joint_names']),
            joint_positions=list(entry['joint_positions']),
        ))
    return result
