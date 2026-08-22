"""Trajectory PLAYBACK: load, validate (joint names/count/limits/finite
values), and play via the FollowJointTrajectory action.

PAUSE/RESUME note: FollowJointTrajectory has no native pause. PAUSE cancels
the in-flight goal and estimates which point playback had reached from
wall-clock elapsed time versus each point's time_from_start; RESUME
re-sends the remaining points as a new goal. This is an approximation
(the real controller's actual progress could differ slightly from the
wall-clock estimate) -- documented here rather than presented as exact.

Arm/gripper split: a recorded TrajectoryData's joint_names is whatever the
recorder was given -- in practice arm joints AND the gripper together (see
TrajectoryRecorder). arm_controller's FollowJointTrajectory action doesn't
manage the gripper joint at all, so a goal naming it gets rejected
outright by the controller before anything plays. play() strips the goal
down to just the arm joints; the gripper (if present in the recording)
gets a single best-effort GripperCommand to its position at whichever
point playback starts from -- it doesn't support multi-point trajectories
the way the arm does, so recorded gripper motion mid-trajectory doesn't
replay smoothly, only the position at that starting point.
"""
from __future__ import annotations

import time
import uuid
from enum import Enum, auto
from typing import List, Optional

from PySide6.QtCore import QObject, Signal

from .joint_limits import JointLimits
from ..ros.robot_interface import RobotInterface
from ..storage.trajectory_storage import TrajectoryData
from ..utils.logging import get_logger
from ..utils.signals import ActionOutcome

_log = get_logger('robot.trajectory_manager')


class PlaybackState(Enum):
    IDLE = auto()
    PLAYING = auto()
    PAUSED = auto()


class TrajectoryManager(QObject):
    playback_state_changed = Signal(object)   # PlaybackState
    playback_finished = Signal(object)         # ActionOutcome
    error_occurred = Signal(str)
    trajectory_loaded = Signal(object)         # TrajectoryData

    def __init__(
        self,
        robot_interface: RobotInterface,
        joint_limits: JointLimits,
        arm_joint_names: List[str],
        gripper_joint_name: str,
        gripper_max_effort: float = 50.0,
        parent: Optional[QObject] = None,
    ) -> None:
        super().__init__(parent)
        self._ri = robot_interface
        self._joint_limits = joint_limits
        self._arm_joint_names = list(arm_joint_names)
        self._gripper_joint_name = gripper_joint_name
        self._gripper_max_effort = gripper_max_effort
        self._trajectory: Optional[TrajectoryData] = None
        self._state = PlaybackState.IDLE
        self._request_id = ''
        self._playback_start_wall = 0.0
        self._playback_start_index = 0
        self._paused_at_index = 0

        robot_interface.trajectory_result.connect(self._on_result)

    @property
    def state(self) -> PlaybackState:
        return self._state

    @property
    def trajectory(self) -> Optional[TrajectoryData]:
        return self._trajectory

    def load(self, trajectory: TrajectoryData) -> None:
        self._trajectory = trajectory
        self._set_state(PlaybackState.IDLE)
        self.trajectory_loaded.emit(trajectory)

    def validate(self) -> List[str]:
        if self._trajectory is None:
            return ['No trajectory loaded']
        errors: List[str] = []
        if not self._trajectory.points:
            errors.append('Trajectory has no points')
            return errors
        for i, p in enumerate(self._trajectory.points):
            if len(p.positions) != len(self._trajectory.joint_names):
                errors.append(f'Point {i}: {len(p.positions)} positions for {len(self._trajectory.joint_names)} joints')
                continue
            errors.extend(self._joint_limits.validate_positions(self._trajectory.joint_names, p.positions))
        return errors

    def play(self) -> None:
        if self._state == PlaybackState.PLAYING:
            return
        errors = self.validate()
        if errors:
            self.error_occurred.emit('Cannot play trajectory:\n' + '\n'.join(errors[:10]))
            return
        self._send_from_index(self._paused_at_index if self._state == PlaybackState.PAUSED else 0)

    def pause(self) -> None:
        if self._state != PlaybackState.PLAYING or self._trajectory is None:
            return
        elapsed = time.monotonic() - self._playback_start_wall
        base = self._trajectory.points[self._playback_start_index].time_from_start
        target_elapsed = elapsed + base
        self._ri.request_cancel_all()

        resume_index = len(self._trajectory.points) - 1
        for i, p in enumerate(self._trajectory.points):
            if p.time_from_start >= target_elapsed:
                resume_index = i
                break
        self._paused_at_index = resume_index
        _log.info(f'Playback paused near point {resume_index}/{len(self._trajectory.points)}')
        self._set_state(PlaybackState.PAUSED)

    def resume(self) -> None:
        if self._state != PlaybackState.PAUSED:
            return
        self._send_from_index(self._paused_at_index)

    def stop(self) -> None:
        self._ri.request_cancel_all()
        self._paused_at_index = 0
        self._set_state(PlaybackState.IDLE)

    def _send_from_index(self, index: int) -> None:
        assert self._trajectory is not None
        points = self._trajectory.points[index:]
        if not points:
            self._set_state(PlaybackState.IDLE)
            return

        joint_names = self._trajectory.joint_names
        try:
            arm_indices = [joint_names.index(n) for n in self._arm_joint_names]
        except ValueError:
            self.error_occurred.emit('Cannot play: trajectory is missing data for one or more arm joints')
            return

        base_t = points[0].time_from_start
        payload = [
            (
                p.time_from_start - base_t,
                [p.positions[i] for i in arm_indices],
                [p.velocities[i] for i in arm_indices] if p.velocities else None,
            )
            for p in points
        ]

        self._playback_start_wall = time.monotonic()
        self._playback_start_index = index
        self._request_id = str(uuid.uuid4())
        _log.info(f'Playing trajectory from point {index} ({len(payload)} arm points)')

        # See module docstring: gripper doesn't go through FollowJointTrajectory
        # at all, just a one-shot best-effort move to this segment's starting
        # gripper position, not gated on the arm goal's result.
        if self._gripper_joint_name in joint_names:
            gripper_idx = joint_names.index(self._gripper_joint_name)
            self._ri.request_gripper_goal(
                points[0].positions[gripper_idx], self._gripper_max_effort,
                request_id=f'{self._request_id}-gripper',
            )

        self._ri.request_full_trajectory_goal(
            self._arm_joint_names, payload, request_id=self._request_id
        )
        self._set_state(PlaybackState.PLAYING)

    def _set_state(self, state: PlaybackState) -> None:
        self._state = state
        self.playback_state_changed.emit(state)

    def _on_result(self, outcome: ActionOutcome) -> None:
        if outcome.request_id != self._request_id:
            return
        if self._state != PlaybackState.PLAYING:
            return  # already paused/stopped locally; ignore late result
        if not outcome.accepted:
            self.error_occurred.emit(f'Trajectory goal rejected: {outcome.message}')
            self._set_state(PlaybackState.IDLE)
            return
        if outcome.succeeded:
            self._paused_at_index = 0
            self._set_state(PlaybackState.IDLE)
        else:
            _log.warning(f'Trajectory playback finished without success: {outcome.message}')
            self._set_state(PlaybackState.IDLE)
        self.playback_finished.emit(outcome)
