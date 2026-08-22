"""JSON load/save for Programs -- an ordered sequence of ProgramElements
(waypoints + gripper actions). Deliberately separate from
waypoint_storage.py: a Program *references* the same Waypoint dataclass
for its waypoint-type elements (reuse, not a parallel representation) but
is its own persisted concept, matching a saved *sequence* vs. a saved
*single target*.

No Qt here -- plain I/O, unit testable in isolation.
"""
from __future__ import annotations

import uuid
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional

import json

from .waypoint_storage import Waypoint


class ElementType(str, Enum):
    WAYPOINT = 'waypoint'
    JAW_OPEN = 'jaw_open'
    JAW_CLOSE = 'jaw_close'
    WAIT = 'wait'
    # New action types (SetSpeed, GripperPosition, DigitalOutput, ...) add
    # cleanly as more enum members + a branch in ProgramManager's executor
    # -- nothing about this structure assumes only these four exist.


@dataclass
class ProgramElement:
    element_type: ElementType
    element_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    waypoint: Optional[Waypoint] = None  # set only when element_type == WAYPOINT
    # Per-element motion scaling, meaningful only for WAYPOINT elements.
    # None means "use the app's configured default" (see ProgramManager) --
    # deliberately NOT defaulted to a number here, so a program saved before
    # this feature existed (or an element that never had it set) resolves to
    # whatever the dashboard is currently configured for, not a silently
    # different hardcoded speed.
    velocity_scaling: Optional[float] = None
    acceleration_scaling: Optional[float] = None
    # Meaningful only for WAIT elements -- how long the program pauses
    # here before advancing to the next element. Unlike the motion
    # scalings above this always has a concrete value (there's no
    # app-wide "default wait duration" to fall back to).
    wait_duration_sec: float = 1.0

    @property
    def display_name(self) -> str:
        if self.element_type == ElementType.WAYPOINT:
            return self.waypoint.name if self.waypoint else '(unnamed waypoint)'
        if self.element_type == ElementType.WAIT:
            return f'Wait {self.wait_duration_sec:g}s'
        return 'Jaw Open' if self.element_type == ElementType.JAW_OPEN else 'Jaw Close'


@dataclass
class Program:
    name: str
    elements: List[ProgramElement] = field(default_factory=list)
    version: int = 1
    # How many times RUN executes the full element sequence before
    # stopping on its own: 1 (default) = run once, same as before this
    # existed; 0 = loop forever until STOP is pressed; N>1 = run N times.
    loop_count: int = 1


def _element_to_dict(el: ProgramElement) -> dict:
    if el.element_type == ElementType.WAYPOINT:
        wp = el.waypoint
        return {
            'type': el.element_type.value,
            'name': wp.name,
            'joint_names': wp.joint_names,
            'joints': wp.joint_positions,
            'timestamp': wp.timestamp,
            'velocity_scaling': el.velocity_scaling,
            'acceleration_scaling': el.acceleration_scaling,
        }
    if el.element_type == ElementType.WAIT:
        return {'type': el.element_type.value, 'duration_sec': el.wait_duration_sec}
    return {'type': el.element_type.value}


def _element_from_dict(data: dict) -> ProgramElement:
    element_type = ElementType(data['type'])
    if element_type == ElementType.WAYPOINT:
        wp = Waypoint(
            name=data.get('name', 'Waypoint'),
            timestamp=data.get('timestamp', ''),
            joint_names=list(data.get('joint_names', [])),
            joint_positions=list(data.get('joints', [])),
        )
        return ProgramElement(
            element_type=element_type,
            waypoint=wp,
            velocity_scaling=data.get('velocity_scaling'),
            acceleration_scaling=data.get('acceleration_scaling'),
        )
    if element_type == ElementType.WAIT:
        return ProgramElement(element_type=element_type, wait_duration_sec=float(data.get('duration_sec', 1.0)))
    return ProgramElement(element_type=element_type)


def save_program(path: str, program: Program) -> None:
    data = {
        'program_name': program.name,
        'version': program.version,
        'loop_count': program.loop_count,
        'elements': [_element_to_dict(el) for el in program.elements],
    }
    with open(path, 'w') as f:
        json.dump(data, f, indent=2)


def load_program(path: str) -> Program:
    with open(path, 'r') as f:
        data = json.load(f)
    if 'elements' not in data or 'program_name' not in data:
        raise ValueError(f"'{path}' is not a valid program file (missing program_name/elements)")
    elements = [_element_from_dict(e) for e in data['elements']]
    return Program(
        name=data['program_name'], elements=elements, version=data.get('version', 1),
        loop_count=data.get('loop_count', 1),
    )
