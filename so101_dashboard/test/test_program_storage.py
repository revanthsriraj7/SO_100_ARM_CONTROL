"""Unit tests for Program/ProgramElement JSON serialization."""
import json
import os

import pytest

from so101_dashboard.storage.program_storage import (
    ElementType, Program, ProgramElement, load_program, save_program,
)
from so101_dashboard.storage.waypoint_storage import Waypoint


def make_program() -> Program:
    home = Waypoint.create('Home', ['j1', 'j2'], [0.0, 0.0])
    pick = Waypoint.create('Pick', ['j1', 'j2'], [0.5, -0.3])
    return Program(name='PickAndPlace', elements=[
        ProgramElement(element_type=ElementType.WAYPOINT, waypoint=home),
        ProgramElement(element_type=ElementType.WAYPOINT, waypoint=pick),
        ProgramElement(element_type=ElementType.JAW_CLOSE),
        ProgramElement(element_type=ElementType.WAYPOINT, waypoint=home),
        ProgramElement(element_type=ElementType.JAW_OPEN),
    ])


def test_save_load_round_trip(tmp_path):
    program = make_program()
    path = os.path.join(tmp_path, 'test.json')

    save_program(path, program)
    loaded = load_program(path)

    assert loaded.name == 'PickAndPlace'
    assert len(loaded.elements) == 5
    assert [e.element_type for e in loaded.elements] == [
        ElementType.WAYPOINT, ElementType.WAYPOINT, ElementType.JAW_CLOSE,
        ElementType.WAYPOINT, ElementType.JAW_OPEN,
    ]
    assert loaded.elements[0].waypoint.name == 'Home'
    assert loaded.elements[1].waypoint.joint_positions == [0.5, -0.3]
    assert loaded.elements[2].waypoint is None
    assert loaded.elements[4].waypoint is None


def test_loop_count_defaults_to_one():
    assert Program(name='X').loop_count == 1


def test_loop_count_round_trips(tmp_path):
    program = Program(name='Looper', loop_count=5, elements=[])
    path = os.path.join(tmp_path, 'looper.json')
    save_program(path, program)
    loaded = load_program(path)
    assert loaded.loop_count == 5


def test_loading_a_file_saved_before_loop_count_existed_defaults_to_one(tmp_path):
    path = os.path.join(tmp_path, 'legacy.json')
    with open(path, 'w') as f:
        json.dump({'program_name': 'Legacy', 'version': 1, 'elements': []}, f)
    loaded = load_program(path)
    assert loaded.loop_count == 1


def test_element_scaling_round_trips(tmp_path):
    home = Waypoint.create('Home', ['j1'], [0.0])
    program = Program(name='Scaled', elements=[
        ProgramElement(element_type=ElementType.WAYPOINT, waypoint=home,
                        velocity_scaling=0.75, acceleration_scaling=0.5),
    ])
    path = os.path.join(tmp_path, 'scaled.json')
    save_program(path, program)
    loaded = load_program(path)
    assert loaded.elements[0].velocity_scaling == 0.75
    assert loaded.elements[0].acceleration_scaling == 0.5


def test_element_scaling_defaults_to_none_when_unset():
    home = Waypoint.create('Home', ['j1'], [0.0])
    el = ProgramElement(element_type=ElementType.WAYPOINT, waypoint=home)
    assert el.velocity_scaling is None
    assert el.acceleration_scaling is None


def test_loading_a_file_saved_before_scaling_existed_defaults_to_none(tmp_path):
    # Simulates a program file saved by an older version of the app that
    # never wrote velocity_scaling/acceleration_scaling keys at all.
    path = os.path.join(tmp_path, 'legacy.json')
    with open(path, 'w') as f:
        json.dump({
            'program_name': 'Legacy',
            'version': 1,
            'elements': [
                {'type': 'waypoint', 'name': 'Home', 'joint_names': ['j1'], 'joints': [0.0], 'timestamp': ''},
            ],
        }, f)
    loaded = load_program(path)
    assert loaded.elements[0].velocity_scaling is None
    assert loaded.elements[0].acceleration_scaling is None


def test_display_name():
    home = Waypoint.create('Home', ['j1'], [0.0])
    assert ProgramElement(element_type=ElementType.WAYPOINT, waypoint=home).display_name == 'Home'
    assert ProgramElement(element_type=ElementType.JAW_OPEN).display_name == 'Jaw Open'
    assert ProgramElement(element_type=ElementType.JAW_CLOSE).display_name == 'Jaw Close'


def test_saved_file_matches_expected_schema_shape(tmp_path):
    # Matches the project spec's example schema: program_name/version/elements,
    # each element a flat dict with a "type" discriminator.
    program = make_program()
    path = os.path.join(tmp_path, 'test.json')
    save_program(path, program)

    with open(path) as f:
        raw = json.load(f)

    assert raw['program_name'] == 'PickAndPlace'
    assert 'version' in raw
    assert raw['elements'][0]['type'] == 'waypoint'
    assert raw['elements'][0]['name'] == 'Home'
    assert 'joints' in raw['elements'][0]
    assert raw['elements'][2] == {'type': 'jaw_close'}
    assert raw['elements'][4] == {'type': 'jaw_open'}


def test_load_missing_file_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_program(os.path.join(tmp_path, 'does_not_exist.json'))


def test_load_invalid_json_raises(tmp_path):
    path = os.path.join(tmp_path, 'bad.json')
    with open(path, 'w') as f:
        f.write('{not valid json')
    with pytest.raises(json.JSONDecodeError):
        load_program(path)


def test_load_valid_json_missing_required_keys_raises(tmp_path):
    path = os.path.join(tmp_path, 'incomplete.json')
    with open(path, 'w') as f:
        json.dump({'foo': 'bar'}, f)
    with pytest.raises(ValueError):
        load_program(path)


def test_empty_program_round_trip(tmp_path):
    path = os.path.join(tmp_path, 'empty.json')
    save_program(path, Program(name='Empty'))
    loaded = load_program(path)
    assert loaded.name == 'Empty'
    assert loaded.elements == []


# -- wait/delay elements --

def test_wait_element_defaults_to_one_second():
    el = ProgramElement(element_type=ElementType.WAIT)
    assert el.wait_duration_sec == 1.0


def test_wait_element_display_name():
    el = ProgramElement(element_type=ElementType.WAIT, wait_duration_sec=2.5)
    assert el.display_name == 'Wait 2.5s'


def test_wait_element_round_trips(tmp_path):
    program = Program(name='WithWait', elements=[
        ProgramElement(element_type=ElementType.WAIT, wait_duration_sec=3.5),
    ])
    path = os.path.join(tmp_path, 'wait.json')
    save_program(path, program)
    loaded = load_program(path)
    assert loaded.elements[0].element_type == ElementType.WAIT
    assert loaded.elements[0].wait_duration_sec == 3.5
    assert loaded.elements[0].waypoint is None


def test_wait_element_schema_shape(tmp_path):
    program = Program(name='WithWait', elements=[
        ProgramElement(element_type=ElementType.WAIT, wait_duration_sec=1.5),
    ])
    path = os.path.join(tmp_path, 'wait.json')
    save_program(path, program)
    with open(path) as f:
        raw = json.load(f)
    assert raw['elements'][0] == {'type': 'wait', 'duration_sec': 1.5}


def test_wait_element_missing_duration_defaults_to_one_second(tmp_path):
    # A file that only has {"type": "wait"} (or an even older format that
    # predates this field entirely) must still load cleanly.
    path = os.path.join(tmp_path, 'legacy_wait.json')
    with open(path, 'w') as f:
        json.dump({'program_name': 'X', 'version': 1, 'elements': [{'type': 'wait'}]}, f)
    loaded = load_program(path)
    assert loaded.elements[0].wait_duration_sec == 1.0
