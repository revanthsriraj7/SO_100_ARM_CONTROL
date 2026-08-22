#!/usr/bin/env python3
"""Reference template: every function the so101_api SDK exposes, with a
comment on what each one does.

SAFETY: anything that can MOVE the robot or change torque/program state
is commented out below, not executed. Before uncommenting any of those,
make sure `robot = SO101(...)` is actually pointed at the dashboard
instance you mean to command (mock or real) -- if two dashboards are
running, whichever one is bound to that host:port will move.
"""
from so101_api import SO101

robot = SO101('http://127.0.0.1:8000')


# ============================================================
# ROBOT STATE / STATUS -- read-only, safe to run any time
# ============================================================

state = robot.get_state()
# {'connected': bool, 'torque_state': str, 'joints': {...}, 'gripper': {...},
#  'program': {'name', 'state', 'current_element_index', 'current_element_name',
#              'current_loop', 'loop_count'}}
print('state:', state)

status = robot.get_status()
# {'ros_connected', 'joint_states_receiving', 'joint_states_stale',
#  'controller_active', 'mock_mode'}
print('status:', status)

torque_state = robot.get_torque_state()
# {'state': str, 'can_enable': bool, 'can_disable': bool, 'gripper_lock_enabled': bool}
print('torque_state:', torque_state)


# ============================================================
# TORQUE CONTROL -- changes real torque/hold state; commented out
# ============================================================

# robot.enable_torque()    # hold current position (equivalent to HOLD CURRENT POSITION)
# robot.disable_torque()   # free move (equivalent to FREE MOVE)


# ============================================================
# TARGETS (the WAYPOINTS library) -- list/get are read-only; move is not
# ============================================================

targets = robot.get_targets()
# [{'name': str, 'joint_names': [str, ...]}, ...]
print('targets:', targets)

if targets:
    target = robot.get_target(targets[0]['name'])
    # {'name', 'joint_names', 'joints': {joint_name: position}, 'cartesian_pose': {x,y,z} or None}
    print('target detail:', target)

# robot.move_to_target('home', duration_sec=3.0, velocity_scaling=1.0,
#                       acceleration_scaling=1.0, wait=True)  # moves the arm


# ============================================================
# PROGRAMS -- list/get are read-only; run/stop are not
# ============================================================

programs = robot.get_programs()
# [{'name', 'num_elements', 'loop_count', 'is_active'}, ...]
print('programs:', programs)

if programs:
    program = robot.get_program(programs[0]['name'])
    # {'name', 'elements': [{'index','type','name','state',
    #    'wait_duration_sec','velocity_scaling','acceleration_scaling'}, ...],
    #  'loop_count', 'execution_state', 'current_element_index', 'current_loop'}
    print('program detail:', program)

# robot.run_program('remove_bobbin')   # starts it, returns immediately (does not wait)
# robot.stop()                         # safely stops whatever program is running

# Not supported by the current executor -- always raises/returns 501:
# robot.programs.pause() / robot.programs.resume()  # (no flat-surface shortcut for these)


# ============================================================
# GRIPPER -- all of these move the real gripper; commented out
# ============================================================

# robot.open_gripper()                          # wait=True by default (blocks until done)
# robot.close_gripper()
# robot.gripper.position(position=0.8, max_effort=40.0, wait=True)  # arbitrary position


# ============================================================
# Same calls again, via the nested (RoboDK-style) namespaces --
# robot.programs / robot.targets / robot.robot / robot.gripper are just
# thin wrappers over the exact same HTTP endpoints as the flat methods
# above. Pick whichever reads better.
# ============================================================

# robot.robot.state()             == robot.get_state()
# robot.robot.status()            == robot.get_status()
# robot.robot.torque()            == robot.get_torque_state()
# robot.robot.enable_torque()     == robot.enable_torque()
# robot.robot.disable_torque()    == robot.disable_torque()

# robot.targets.list()            == robot.get_targets()
# robot.targets.get(name)         == robot.get_target(name)
# robot.targets.move(name, ...)   == robot.move_to_target(name, ...)

# robot.programs.list()           == robot.get_programs()
# robot.programs.get(name)        == robot.get_program(name)
# robot.programs.run(name)        == robot.run_program(name)
# robot.programs.stop()           == robot.stop()

# robot.gripper.open()            == robot.open_gripper()
# robot.gripper.close()           == robot.close_gripper()
# robot.gripper.position(pos)     -- no flat-surface equivalent


# ============================================================
# Errors -- every mutating call above can raise one of these
# (all subclasses of so101_api.ApiError / SO101Error)
# ============================================================
# from so101_api import ProgramNotFound, TargetNotFound, ProgramBusy, NotSupported, ConnectionFailed
#
# try:
#     robot.run_program('DoesNotExist')
# except ProgramNotFound as exc:
#     print(exc)
