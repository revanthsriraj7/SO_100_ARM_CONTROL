"""AutoSaver: edits reach disk (primary + backup) without an explicit SAVE,
and restore_missing() brings back files a colcon build wiped."""
import os

from so101_dashboard.app.autosave import LIBRARY_FILENAME, AutoSaver, restore_missing
from so101_dashboard.storage.waypoint_storage import load_waypoints
from test_program_manager import make_manager as make_pm
from test_waypoint_manager import make_manager as make_wm


def test_record_and_program_edits_are_autosaved_and_backed_up(qapp, fake_ri, tmp_path):
    wm, _ = make_wm(fake_ri)
    pm, _ = make_pm(fake_ri)
    wp_dir, prog_dir, backup = str(tmp_path / 'wp'), str(tmp_path / 'prog'), str(tmp_path / 'bk')
    saver = AutoSaver(wm, pm, wp_dir, prog_dir, backup)

    wm.record('home')
    pm.add_waypoint()
    saver.flush()

    for base in (wp_dir, os.path.join(backup, 'waypoints')):
        assert [w.name for w in load_waypoints(os.path.join(base, LIBRARY_FILENAME))] == ['home']
    for base in (prog_dir, os.path.join(backup, 'programs')):
        assert os.listdir(base) == [f'{pm.active_name}.json']


def test_restore_missing_does_not_overwrite(tmp_path):
    backup, wp_dir, prog_dir = tmp_path / 'bk', tmp_path / 'wp', tmp_path / 'prog'
    (backup / 'waypoints').mkdir(parents=True)
    (backup / 'waypoints' / LIBRARY_FILENAME).write_text('waypoints: []\n')
    (backup / 'programs').mkdir()
    (backup / 'programs' / 'a.json').write_text('old')
    prog_dir.mkdir()
    (prog_dir / 'a.json').write_text('new')

    restore_missing(str(backup), str(wp_dir), str(prog_dir))

    assert (wp_dir / LIBRARY_FILENAME).exists()
    assert (prog_dir / 'a.json').read_text() == 'new'
