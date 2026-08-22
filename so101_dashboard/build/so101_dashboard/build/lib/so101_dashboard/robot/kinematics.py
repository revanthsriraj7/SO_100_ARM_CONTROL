"""Forward/inverse kinematics for the SO-100/SO-101 5-DOF arm, used only
by Cartesian (X/Y/Z) jogging in Joint Control -- everything else in this
app commands joints directly and never needs this module.

The joint chain below (origin xyz/rpy + axis per joint, base_link frame)
is transcribed directly from
SO-100-arm/so_arm_100_description/urdf/so_arm_100_5dof.urdf
(Shoulder_Rotation -> Shoulder_Pitch -> Elbow -> Wrist_Pitch -> Wrist_Roll,
then the fixed End_Effector_Joint offset to the tool-tip frame). forward()
implements the textbook URDF joint-transform recurrence directly --
T_parent_child(q) = T_origin * Rot(axis, q) -- composed per joint, so
there's no dependency on any particular kinematics library's internal
chain-building convention.

This has been cross-checked against ROS's own robot_state_publisher/tf2
(which computes TF from this exact URDF via kdl_parser) at multiple
non-trivial joint configurations: both position and full orientation
matched to 6 decimal places. If the URDF's arm geometry ever changes,
this table must be updated to match -- nothing here reads the URDF file
at runtime.

Inverse kinematics is a damped-least-squares (Levenberg-Marquardt style)
iterative solver using a numerically-differentiated Jacobian of forward()
-- deliberately not an analytic/closed-form solution, so it doesn't
depend on any solver-library chain construction either, only on
forward() being correct (which is what was cross-checked above). It
solves position only (X/Y/Z), not orientation, matching what Cartesian
jog actually needs; each jog step seeds from the arm's current
commanded joint targets, so it's a small local correction, not a
from-scratch global reach -- exactly the regime this kind of solver
converges fastest and most reliably in.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np
import PyKDL

from .joint_limits import JointLimits

# (xyz, rpy, axis) per revolute joint, parent-to-joint-frame, in that
# joint's own local axis -- see module docstring for source.
_JOINT_CHAIN = [
    ((0.0, -0.0452, 0.0165), (1.5708, 0.0, 0.0), (0.0, -1.0, 0.0)),   # Shoulder_Rotation
    ((0.0, 0.1025, 0.0306), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),         # Shoulder_Pitch
    ((0.0, 0.11257, 0.028), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),         # Elbow
    ((0.0, 0.0052, 0.1349), (-1.57079, 0.0, 0.0), (1.0, 0.0, 0.0)),    # Wrist_Pitch
    ((0.0, -0.0601, 0.0), (0.0, 1.57079, 0.0), (0.0, 1.0, 0.0)),       # Wrist_Roll
]
# Fixed End_Effector_Joint offset from the Wrist_Roll child link (Fixed_Gripper)
# to the tool-tip frame (End_Effector) -- unaffected by the Gripper joint itself.
_END_EFFECTOR_OFFSET_XYZ = (0.0, -0.09, 0.0)
_END_EFFECTOR_OFFSET_RPY = (0.0, 0.0, 0.0)

NUM_ARM_JOINTS = len(_JOINT_CHAIN)


@dataclass
class IKResult:
    success: bool
    positions: Optional[List[float]]
    residual_m: float
    message: str


def _fk_frame(joint_positions: Sequence[float]) -> PyKDL.Frame:
    if len(joint_positions) != NUM_ARM_JOINTS:
        raise ValueError(f'expected {NUM_ARM_JOINTS} arm joint positions, got {len(joint_positions)}')
    T = PyKDL.Frame.Identity()
    for (xyz, rpy, axis), q in zip(_JOINT_CHAIN, joint_positions):
        T_origin = PyKDL.Frame(PyKDL.Rotation.RPY(*rpy), PyKDL.Vector(*xyz))
        R_joint = PyKDL.Rotation.Rot(PyKDL.Vector(*axis), q)
        T = T * T_origin * PyKDL.Frame(R_joint)
    T_ee = PyKDL.Frame(PyKDL.Rotation.RPY(*_END_EFFECTOR_OFFSET_RPY), PyKDL.Vector(*_END_EFFECTOR_OFFSET_XYZ))
    return T * T_ee


def forward(joint_positions: Sequence[float]) -> Tuple[float, float, float]:
    """(x, y, z) meters, base_link frame, of the tool-tip (End_Effector)."""
    p = _fk_frame(joint_positions).p
    return (p.x(), p.y(), p.z())


def _fk_position_vec(joint_positions: Sequence[float]) -> np.ndarray:
    x, y, z = forward(joint_positions)
    return np.array([x, y, z])


def _numerical_jacobian(joint_positions: Sequence[float], eps: float = 1e-6) -> np.ndarray:
    p0 = _fk_position_vec(joint_positions)
    J = np.zeros((3, NUM_ARM_JOINTS))
    q = list(joint_positions)
    for i in range(NUM_ARM_JOINTS):
        q[i] += eps
        J[:, i] = (_fk_position_vec(q) - p0) / eps
        q[i] -= eps
    return J


def solve_ik(
    seed_positions: Sequence[float],
    target_xyz: Tuple[float, float, float],
    joint_limits: Optional[JointLimits] = None,
    arm_joint_names: Optional[Sequence[str]] = None,
    max_iters: int = 50,
    tolerance_m: float = 1e-4,
    damping: float = 0.02,
) -> IKResult:
    """Damped-least-squares position IK. Clamps each iteration's step to
    joint_limits (if given, alongside arm_joint_names to look them up) so
    the solver can't wander through a wildly out-of-range configuration
    while converging -- but the final success/failure verdict is decided
    purely by whether forward(result) actually lands within tolerance_m
    of the target, not by iteration count. A result is only ever returned
    as success=True if it's been positionally re-verified against
    forward() -- never just "trusted" from the solver's own math."""
    if len(seed_positions) != NUM_ARM_JOINTS:
        return IKResult(False, None, float('nan'), f'expected {NUM_ARM_JOINTS} seed positions')

    limits = None
    if joint_limits is not None and arm_joint_names is not None:
        limits = []
        for name in arm_joint_names:
            lim = joint_limits.get(name)
            lo = lim.min_position if (lim is not None and lim.min_position is not None) else -np.inf
            hi = lim.max_position if (lim is not None and lim.max_position is not None) else np.inf
            limits.append((lo, hi))

    q = np.array(seed_positions, dtype=float)
    target = np.array(target_xyz, dtype=float)
    try:
        for _ in range(max_iters):
            p = _fk_position_vec(q)
            err = target - p
            if np.linalg.norm(err) < tolerance_m:
                break
            J = _numerical_jacobian(q)
            JJt = J @ J.T + (damping ** 2) * np.eye(3)
            dq = J.T @ np.linalg.solve(JJt, err)
            q = q + dq
            if limits is not None:
                for i, (lo, hi) in enumerate(limits):
                    q[i] = min(max(q[i], lo), hi)
    except np.linalg.LinAlgError as exc:
        return IKResult(False, None, float('nan'), f'IK solver numerical failure: {exc}')

    residual = float(np.linalg.norm(target - _fk_position_vec(q)))
    if residual >= tolerance_m:
        return IKResult(False, None, residual, f'IK did not converge (residual {residual * 1000:.2f} mm)')
    return IKResult(True, [float(v) for v in q], residual, 'ok')
