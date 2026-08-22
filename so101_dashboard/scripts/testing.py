#!/usr/bin/env python3
"""Run several programs back to back, waiting for each to finish before
starting the next."""
import time

from so101_api import SO101

robot = SO101('http://127.0.0.1:8000')


def run_and_wait(name):
    robot.run_program(name)
    while True:
        prog = robot.get_state()['program']
        print(name, '->', prog['state'], prog['current_element_name'])
        if prog['state'] in ('FINISHED', 'STOPPED', 'FAILED'):
            return prog['state']
        time.sleep(0.5)


for name in ['pick_bobbin_up', 'turn_bobbin_swith_on', 'turn_bobbin_off', 'remove_bobbin']:
    result = run_and_wait(name)
    if result != 'FINISHED':
        print(f'{name} ended in {result}, stopping the sequence.')
        break
