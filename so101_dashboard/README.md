# SO-101 Control Dashboard

A ROS 2 Humble + PySide6 control, telemetry, waypoint, trajectory-recording,
and rosbag dashboard for the SO-101/SO-100 arm.

## Architecture

```
                 ┌──────────────────────┐
                 │     Qt Main Thread   │
                 │  MainWindow, tabs,   │
                 │  ControlPanel, plots │
                 └──────────┬───────────┘
                             │ Qt signals/slots (queued, cross-thread)
                 ┌──────────▼───────────┐
                 │   RobotInterface     │  <- the only class that touches
                 │   (QThread)          │     the ROS worker thread boundary
                 │  DashboardNode:      │
                 │   subscribers        │
                 │   trajectory/gripper │
                 │     action clients   │
                 │   /toggle_torque     │
                 │   rosbag2 writer     │
                 └──────────┬───────────┘
                             ▼
                        ROS 2 graph
```

* `RobotInterface` is the **only** place `rclpy.spin_once()` runs. It owns a
  thread-safe command queue: GUI code calls `request_*()` methods (never
  touches rclpy directly), which enqueue a closure that executes on the ROS
  thread between `spin_once()` calls. Incoming data (joint states, service/
  action results) comes back out as Qt `Signal(object)` emissions, which Qt
  automatically marshals to the GUI thread.
* `DashboardNode` (in `ros/ros_node.py`) is **pure rclpy, no Qt** -- it's
  independently constructible/testable outside the GUI entirely.
* `TorqueManager` is the **only** component allowed to call `/toggle_torque`.
  Every other manager/widget calls into `TorqueManager`, never the service.
* `RobotState` is a read-only mirror other widgets consume; it never
  initiates ROS calls itself.

### The torque workflow

`/toggle_torque` (`std_srvs/srv/Trigger`) is a **toggle**, not
enable/disable, and gives no state feedback. `TorqueManager` tracks the
robot's expected torque state explicitly and only calls the service from a
known, deliberate transition:

```
UNKNOWN/ENABLED/HOLDING
      │ FREE MOVE
      ▼
cancel active goals → TOGGLING → call /toggle_torque
      │ success                          │ failure
      ▼                                  ▼
   FREE_MOVE                           ERROR (never assumed changed)
      │ HOLD
      ▼
CAPTURING_POSITION (waits for a FRESH /joint_states message)
      │ timeout                         │ fresh + valid
      ▼                                  ▼
    ERROR                        PREPARING_HOLD → TOGGLING → call /toggle_torque
      (never re-enables               │ success
       torque on stale/invalid data)   ▼
                              send captured position as the hold goal
                                        │ accepted
                                        ▼
                                     HOLDING
```

Buttons are disabled for the entire `TOGGLING`/`CAPTURING_POSITION` window,
so a double-click can't cause a double-toggle. Any service failure, action
rejection, stale joint state, or out-of-limits captured position lands in
`ERROR` and requires the user to acknowledge it before trying again --
nothing is ever assumed to have succeeded.

## Dependencies

ROS 2 (via rosdep, see `package.xml`): `rclpy`, `sensor_msgs`,
`trajectory_msgs`, `control_msgs`, `std_srvs`, `action_msgs`, `rosbag2_py`,
`rosbag2_interfaces`.

Not available as rosdep keys for Humble -- install via pip:

```bash
pip install --user PySide6 pyqtgraph PyYAML
```

## Install

```bash
cd ~/so100_arm_ws
colcon build --packages-select so101_dashboard
source install/setup.bash
```

## Run

Against the real robot (run alongside `hardware.launch.py` from
`so_arm_100_bringup` -- this package only starts the GUI, not the robot's
own controllers):

```bash
ros2 run so101_dashboard dashboard
# or
ros2 launch so101_dashboard dashboard.launch.py
```

Mock mode (no robot, no ROS system needed -- synthetic joint states,
simulated torque toggling, simulated action results; rosbag recording is
clearly labeled and does **not** write a real bag):

```bash
ros2 run so101_dashboard dashboard --mock
# or
ros2 launch so101_dashboard dashboard.launch.py mock:=true
```

## Configuration

Everything topic/limit/path related lives in `config/`, nowhere else:

* `config.yaml` -- topic/service/action names, update rates, motion
  durations, safety timeouts, storage paths, rosbag topic list.
* `joint_limits.yaml` -- per-joint position/velocity/effort limits. Copied
  from `so_arm_100_description`'s `ros2_control` xacro at the time this was
  written -- keep in sync if that changes.
* `home_position.yaml` -- MOVE HOME target. Matches the "home" SRDF
  `group_state` already defined for this robot's MoveIt config.

## Testing

```bash
cd src/so101_dashboard
pip install --user pytest
PYTHONPATH=$(pwd) QT_QPA_PLATFORM=offscreen python3 -m pytest test/ -v
```

34 tests covering: unit conversions, the full `TorqueManager` state
machine (including service failure, double-click protection, stale/invalid
joint-state rejection), waypoint record/edit/serialize/move-to validation,
and trajectory serialization/playback validation. All use a
`FakeRobotInterface` test double (see `test/conftest.py`) -- no rclpy, no
physical robot required to run them.

## What's implemented

- **Control**: FREE MOVE / HOLD CURRENT POSITION (full safety workflow
  above), STOP MOTION, MOVE HOME, live joint table (position/velocity/
  effort/commanded/error), robot status panel.
- **Telemetry**: Position/Velocity/Effort/Command-vs-Actual plots
  (pyqtgraph), per-joint selection, pause/resume/clear, configurable
  history duration, rolling (non-growing) buffers.
- **Waypoints**: record/rename/delete/duplicate/reorder, YAML save/load,
  move-to with velocity scaling and joint-limit validation.
- **Trajectories**: record actual motion at a configurable rate
  (independent of GUI/ROS publish rate), YAML save/load, playback with
  pause/resume (see note below) via the `FollowJointTrajectory` action.
- **ROS Bag**: real `rosbag2_py` recording (not a hand-rolled format, not a
  subprocess) with topic selection.
- **Mock mode**: exercises the entire app, including the full torque state
  machine, without a robot.

## Known limitations, stated rather than hidden

- **Trajectory playback PAUSE/RESUME** has no native support in
  `FollowJointTrajectory` -- PAUSE cancels the goal and estimates progress
  from wall-clock elapsed time vs. each point's `time_from_start`; RESUME
  re-sends the remaining points as a new goal. This is an approximation of
  where the controller actually was, not an exact read of controller state.
- **`Controller: ACTIVE/INACTIVE`** in the status panel is currently a
  proxy for "joint states are fresh," not a direct subscription to
  `/arm_controller/transition_event`. Wiring that in is straightforward if
  exact controller lifecycle state matters to you.
- **Torque state has no ROS-visible ground truth.** `TorqueManager`'s state
  is the app's best tracked model of what it *asked* the robot to do and
  what the service *reported* -- there is no query service to confirm
  actual servo state independently. This mirrors a real limitation of
  `/toggle_torque` itself, not a shortcut taken here.
- **Gripper hold** after FREE MOVE is sent as a best-effort `GripperCommand`
  goal alongside the arm's hold goal; only the arm goal gates the
  `HOLDING` state transition (a mis-positioned gripper is lower risk than
  the arm).
