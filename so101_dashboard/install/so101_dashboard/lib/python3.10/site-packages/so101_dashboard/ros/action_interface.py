"""Generic action-client helper: send a goal, get accepted/succeeded/message
back through a single callback. Used for FollowJointTrajectory and
GripperCommand alike so trajectory_controller.py doesn't duplicate the
send/track/result boilerplate for each.

No Qt here -- plain rclpy.
"""
from __future__ import annotations

from typing import Any, Callable, Optional

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future

from ..utils.signals import ActionOutcome


class ActionClientWrapper:
    def __init__(self, node: Node, action_type: Any, action_name: str) -> None:
        self._node = node
        self._client = ActionClient(node, action_type, action_name)
        self._action_name = action_name
        self._active_goal_handle = None
        self._logger = node.get_logger()

    @property
    def action_name(self) -> str:
        return self._action_name

    def is_server_ready(self) -> bool:
        return self._client.server_is_ready()

    def send_goal(
        self,
        goal_msg: Any,
        on_result: Callable[[ActionOutcome], None],
        request_id: Optional[str] = None,
        feedback_callback: Optional[Callable[[Any], None]] = None,
    ) -> None:
        if not self._client.server_is_ready():
            self._logger.warning(f"Action server '{self._action_name}' not ready")
            on_result(ActionOutcome(
                accepted=False, succeeded=False,
                message=f"Action server '{self._action_name}' not available",
                request_id=request_id,
            ))
            return

        send_future = self._client.send_goal_async(
            goal_msg, feedback_callback=feedback_callback
        )

        def _on_goal_response(fut: Future) -> None:
            goal_handle = fut.result()
            if goal_handle is None or not goal_handle.accepted:
                on_result(ActionOutcome(
                    accepted=False, succeeded=False,
                    message=f"Goal rejected by '{self._action_name}'",
                    request_id=request_id,
                ))
                return

            self._active_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()

            def _on_result(rfut: Future) -> None:
                self._active_goal_handle = None
                wrapped = rfut.result()
                if wrapped is None:
                    on_result(ActionOutcome(
                        accepted=True, succeeded=False,
                        message="No result returned", request_id=request_id,
                    ))
                    return
                # GoalStatus.STATUS_SUCCEEDED == 4 (action_msgs/msg/GoalStatus)
                succeeded = wrapped.status == 4
                on_result(ActionOutcome(
                    accepted=True,
                    succeeded=succeeded,
                    message="Succeeded" if succeeded else f"Goal finished with status {wrapped.status}",
                    request_id=request_id,
                ))

            result_future.add_done_callback(_on_result)

        send_future.add_done_callback(_on_goal_response)

    def cancel_active_goal(self) -> bool:
        """Best-effort cancel. Returns True if a cancel request was sent."""
        if self._active_goal_handle is None:
            return False
        self._active_goal_handle.cancel_goal_async()
        return True

    @property
    def has_active_goal(self) -> bool:
        return self._active_goal_handle is not None
