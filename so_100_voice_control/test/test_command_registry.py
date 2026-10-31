import pytest

from so_100_voice_control import voice_commands as vc
from so_100_voice_control.command_registry import CommandConfigError, load_registry


@pytest.fixture(scope='module')
def registry():
    return load_registry(extra_dir='')


@pytest.mark.parametrize('text,action,program', [
    ('Remove the bobbin!', vc.ACTION_RUN_PROGRAM, 'remove_bobbin'),
    ('please turn on the machine', vc.ACTION_RUN_PROGRAM, 'turn_bobbin_on'),
    ('stop the machine', vc.ACTION_RUN_PROGRAM, 'turn_bobbin_off'),  # longer phrase beats "stop"
    ('stop', vc.ACTION_STOP_PROGRAM, None),
    ("what's the robot doing", vc.ACTION_GET_STATUS, None),
    ('next step', vc.ACTION_NEXT_STEP, None),
    ('open the gripper', vc.ACTION_JAW_OPEN, None),
])
def test_matches(registry, text, action, program):
    m = registry.match(text)
    assert (m.action, m.program) == (action, program)


@pytest.mark.parametrize('text', ['', 'make me a sandwich', 'nextdoor', 'unstoppable'])
def test_no_match_on_unrelated_or_partial_words(registry, text):
    assert registry.match(text) is None


def test_sequence_and_spoken_names(registry):
    assert registry.default_workflow() == 'bobbin'
    assert registry.locate('turn_bobbin_off') == ('bobbin', 2)
    assert registry.spoken_name('pick_bobbin_up') == 'Pick bobbin up'
    assert registry.spoken_name('some_new_program') == 'some new program'


def test_extra_dir_adds_commands_without_python(tmp_path):
    (tmp_path / 'weld.yaml').write_text(
        'programs:\n  weld_seam:\n    phrases: [weld the seam]\n'
        'targets:\n  fixture:\n    phrases: [go to the fixture]\n'
    )
    r = load_registry(extra_dir=str(tmp_path))
    assert r.match('weld the seam please').program == 'weld_seam'
    assert r.match('go to the fixture').target == 'fixture'


@pytest.mark.parametrize('body', [
    'actions:\n  explode: [boom]\n',                                              # unknown action
    'bogus: 1\n',                                                                 # unknown key
    'programs:\n  a:\n    phrases: [stop]\n',                                     # clashes with core "stop"
    'programs:\n  a:\n    phrases: [x]\nsequence: [a, missing]\n',                # bad sequence
])
def test_bad_files_are_rejected(tmp_path, body):
    (tmp_path / 'bad.yaml').write_text(body)
    with pytest.raises(CommandConfigError):
        load_registry(extra_dir=str(tmp_path))


def test_parse_command_unknown():
    assert vc.parse_command('make me a sandwich').action == vc.ACTION_UNKNOWN
