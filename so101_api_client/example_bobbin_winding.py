#!/usr/bin/env python3
"""Example: connect to a running so101_dashboard, print its state, list
programs, run a named program, poll until it finishes, and report the
outcome.

Run the dashboard first (real hardware or --mock), e.g.:
    ros2 run so101_dashboard dashboard --mock

Then, from anywhere with `so101_api` installed:
    python3 example_bobbin_winding.py [program_name] [http://host:port]
"""
from __future__ import annotations

import sys
import time

from so101_api import ApiError, ProgramBusy, ProgramNotFound, SO101

TERMINAL_STATES = {'FINISHED', 'STOPPED', 'FAILED'}


def main() -> int:
    program_name = sys.argv[1] if len(sys.argv) > 1 else 'BobbinWinding'
    base_url = sys.argv[2] if len(sys.argv) > 2 else 'http://127.0.0.1:8000'

    robot = SO101(base_url)

    print(f'Connecting to {base_url} ...')
    state = robot.get_state()
    print(f"Connected: {state['connected']}, torque: {state['torque_state']}")
    print(f"Joints: {state['joints']}")

    print('\nAvailable programs:')
    for p in robot.get_programs():
        print(f"  - {p['name']} ({p['num_elements']} elements, loop_count={p['loop_count']})")

    if program_name not in [p['name'] for p in robot.get_programs()]:
        print(f"\n'{program_name}' not found on this dashboard -- create/teach it in the "
              f'PROGRAMS tab first (this script only runs it; it never invents a program).')
        return 1

    print(f"\nRunning program '{program_name}'...")
    try:
        result = robot.run_program(program_name)
    except ProgramBusy as exc:
        print(f'Could not start -- a program is already running: {exc}')
        return 1
    except ProgramNotFound as exc:
        print(f'Program not found: {exc}')
        return 1
    print(f"Started: {result}")

    print('Monitoring execution...')
    while True:
        state = robot.get_state()
        prog = state['program']
        print(f"  state={prog['state']:<10} element={prog['current_element_index']} "
              f"({prog['current_element_name']}) loop={prog['current_loop']}/{prog['loop_count'] or '∞'}")
        if prog['state'] in TERMINAL_STATES:
            break
        time.sleep(0.5)

    final_state = prog['state']
    if final_state == 'FINISHED':
        print(f"\n'{program_name}' completed successfully.")
        return 0
    else:
        print(f"\n'{program_name}' ended in state {final_state} (not a clean finish).")
        return 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except ApiError as exc:
        print(f'API error: {exc}')
        sys.exit(1)
