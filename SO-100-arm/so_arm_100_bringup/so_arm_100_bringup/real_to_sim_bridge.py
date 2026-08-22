#!/usr/bin/env python3
"""Mirror a real robot's joint states into the Gazebo simulated robot.

hardware.launch.py and gz.launch.py run two independent robots (separate
controller_managers) — nothing wires one's state into the other. This node
bridges them: it listens to the real robot's JointState topic and forwards
positions to the sim's arm_controller (as a JointTrajectory stream) and
gripper_controller (as GripperCommand goals, sent only on change).

The sim's controller_manager/controllers live under the /sim namespace (see
the <ros><namespace>/sim</namespace></ros> block in
so_arm_100_5dof.gazebo.xacro), so the defaults here target /sim/... topics
and can run alongside an un-namespaced hardware.launch.py instance without
colliding on /joint_states, /arm_controller/..., etc.

Parameters
- source_joint_states_topic (str): topic to read the real robot's JointState from.
  Defaults to the relative name 'joint_states', which resolves to plain
  /joint_states as published by hardware.launch.py's joint_state_broadcaster.
- arm_joints (string[]): arm joint names, in the order arm_controller expects.
- gripper_joint (str): gripper joint name.
- target_trajectory_topic (str): sim arm_controller's JointTrajectory topic.
- gripper_action_name (str): sim gripper_controller's GripperCommand action.
- publish_rate (double): rate (Hz) to forward commands at, independent of the
  incoming JointState rate.
- trajectory_time_from_start (double): time_from_start for each forwarded point;
  should be a few times the publish period so arm_controller keeps interpolating
  smoothly instead of stopping dead between points.
- gripper_change_threshold (double): minimum position delta (rad) before a new
  GripperCommand goal is sent, so the action server isn't spammed every tick.
- gripper_max_effort (double): max_effort field on forwarded GripperCommand goals.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

DEFAULT_ARM_JOINTS = [
    "Shoulder_Rotation",
    "Shoulder_Pitch",
    "Elbow",
    "Wrist_Pitch",
    "Wrist_Roll",
]


class RealToSimBridge(Node):
    def __init__(self):
        super().__init__("real_to_sim_bridge")

        self.declare_parameter("source_joint_states_topic", "joint_states")
        self.declare_parameter("arm_joints", DEFAULT_ARM_JOINTS)
        self.declare_parameter("gripper_joint", "Gripper")
        self.declare_parameter("target_trajectory_topic", "/sim/arm_controller/joint_trajectory")
        self.declare_parameter("gripper_action_name", "/sim/gripper_controller/gripper_cmd")
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("trajectory_time_from_start", 0.1)
        self.declare_parameter("gripper_change_threshold", 0.01)
        self.declare_parameter("gripper_max_effort", 50.0)

        source_topic = self.get_parameter("source_joint_states_topic").value
        self.arm_joints = list(self.get_parameter("arm_joints").value)
        self.gripper_joint = self.get_parameter("gripper_joint").value
        target_topic = self.get_parameter("target_trajectory_topic").value
        gripper_action = self.get_parameter("gripper_action_name").value
        publish_rate = self.get_parameter("publish_rate").value
        self.time_from_start = self.get_parameter("trajectory_time_from_start").value
        self.gripper_change_threshold = self.get_parameter("gripper_change_threshold").value
        self.gripper_max_effort = self.get_parameter("gripper_max_effort").value

        self.latest_positions = {}
        self.last_sent_gripper_pos = None
        self.gripper_goal_in_flight = False
        self.warned_missing_joints = False
        self.warned_gripper_server = False

        self.traj_pub = self.create_publisher(JointTrajectory, target_topic, 10)
        self.gripper_client = ActionClient(self, GripperCommand, gripper_action)

        self.create_subscription(JointState, source_topic, self.on_joint_states, 10)
        self.create_timer(1.0 / publish_rate, self.on_timer)

        self.get_logger().info(
            f"Bridging '{source_topic}' -> '{target_topic}' (arm) "
            f"and '{gripper_action}' (gripper) at {publish_rate:.1f} Hz"
        )

    def on_joint_states(self, msg: JointState):
        for name, position in zip(msg.name, msg.position):
            self.latest_positions[name] = position

    def on_timer(self):
        if not self.latest_positions:
            return
        self.forward_arm()
        self.forward_gripper()

    def forward_arm(self):
        try:
            positions = [self.latest_positions[j] for j in self.arm_joints]
        except KeyError:
            if not self.warned_missing_joints:
                missing = [j for j in self.arm_joints if j not in self.latest_positions]
                self.get_logger().warning(
                    f"Waiting for joints not yet seen on source topic: {missing}"
                )
                self.warned_missing_joints = True
            return

        traj = JointTrajectory()
        traj.joint_names = self.arm_joints
        point = JointTrajectoryPoint()
        point.positions = positions
        sec = int(self.time_from_start)
        point.time_from_start = Duration(
            sec=sec, nanosec=int((self.time_from_start - sec) * 1e9)
        )
        traj.points = [point]
        self.traj_pub.publish(traj)

    def forward_gripper(self):
        if self.gripper_joint not in self.latest_positions:
            return
        position = self.latest_positions[self.gripper_joint]

        if (
            self.last_sent_gripper_pos is not None
            and abs(position - self.last_sent_gripper_pos) < self.gripper_change_threshold
        ):
            return
        if self.gripper_goal_in_flight:
            return
        if not self.gripper_client.server_is_ready():
            if not self.warned_gripper_server:
                self.get_logger().warning("gripper_controller/gripper_cmd action server not up yet")
                self.warned_gripper_server = True
            return

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = self.gripper_max_effort

        self.last_sent_gripper_pos = position
        self.gripper_goal_in_flight = True
        future = self.gripper_client.send_goal_async(goal)
        future.add_done_callback(self._on_gripper_goal_response)

    def _on_gripper_goal_response(self, future):
        self.gripper_goal_in_flight = False
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warning("Gripper goal was rejected")


def main(args=None):
    rclpy.init(args=args)
    node = RealToSimBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
