"""Autosave for the waypoint library and programs, plus a backup mirror
outside the colcon workspace.

Every edit (RECORD, rename, reorder, add element, ...) schedules a short
debounced write so the library and every program on screen are on disk
without pressing SAVE. Each write is mirrored into a backup directory
(default ~/so100_waypoints) so `colcon build` -- which wipes install/,
where the installed package's relative data/ dir lives -- can't lose
anything. restore_missing() copies backed-up files back on startup.

Layout (identical under the primary data dir and the backup dir):
    waypoints/waypoints.yaml
    programs/<program name>.json
"""
from __future__ import annotations

import os
import re
import shutil
from typing import Optional

from PySide6.QtCore import QObject, QTimer

from ..robot.program_manager import ProgramManager
from ..robot.waypoint_manager import WaypointManager
from ..storage.program_storage import save_program
from ..storage.waypoint_storage import save_waypoints
from ..utils.logging import get_logger

_log = get_logger('app.autosave')

LIBRARY_FILENAME = 'waypoints.yaml'
DEBOUNCE_MS = 300


def program_filename(name: str) -> str:
    # Matches the widget's f'{name}.json', except path separators (which
    # would escape the directory) are replaced.
    return re.sub(r'[\\/]', '_', name) + '.json'


def restore_missing(backup_dir: str, waypoint_dir: str, program_dir: str) -> None:
    """Copy backed-up files that are absent from the primary dirs (e.g.
    after a colcon build wiped them). Never overwrites an existing file."""
    for src_dir, dst_dir in (
        (os.path.join(backup_dir, 'waypoints'), waypoint_dir),
        (os.path.join(backup_dir, 'programs'), program_dir),
    ):
        if not os.path.isdir(src_dir):
            continue
        os.makedirs(dst_dir, exist_ok=True)
        for fname in os.listdir(src_dir):
            dst = os.path.join(dst_dir, fname)
            if os.path.exists(dst):
                continue
            try:
                shutil.copy2(os.path.join(src_dir, fname), dst)
                _log.info(f'Restored {fname} from backup {src_dir}')
            except OSError as exc:
                _log.warning(f'Could not restore {fname} from backup: {exc}')


def _atomic_write(path: str, write) -> None:
    """Write via a temp file + rename so a crash mid-write can't leave a
    truncated file as the only copy."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + '.tmp'
    write(tmp)
    os.replace(tmp, path)


class AutoSaver(QObject):
    def __init__(
        self,
        waypoint_manager: WaypointManager,
        program_manager: ProgramManager,
        waypoint_dir: str,
        program_dir: str,
        backup_dir: Optional[str] = None,
        parent: Optional[QObject] = None,
    ) -> None:
        super().__init__(parent)
        self._wm = waypoint_manager
        self._pm = program_manager
        self._targets = [(waypoint_dir, program_dir)]
        if backup_dir:
            backup_dir = os.path.expanduser(backup_dir)
            self._targets.append((os.path.join(backup_dir, 'waypoints'), os.path.join(backup_dir, 'programs')))

        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.setInterval(DEBOUNCE_MS)
        self._timer.timeout.connect(self.flush)
        self._library_dirty = False
        self._programs_dirty = False

        waypoint_manager.waypoints_changed.connect(self._on_library_changed)
        program_manager.program_changed.connect(self._on_programs_changed)
        program_manager.programs_changed.connect(self._on_programs_changed)

    def _on_library_changed(self) -> None:
        self._library_dirty = True
        self._timer.start()

    def _on_programs_changed(self) -> None:
        self._programs_dirty = True
        self._timer.start()

    def flush(self) -> None:
        """Write anything pending now (also called on app quit)."""
        self._timer.stop()
        if self._library_dirty:
            self._library_dirty = False
            waypoints = self._wm.waypoints
            for wp_dir, _ in self._targets:
                self._try(os.path.join(wp_dir, LIBRARY_FILENAME), lambda p: save_waypoints(p, waypoints))
        if self._programs_dirty:
            self._programs_dirty = False
            for name in self._pm.program_names:
                program = self._pm.get_program(name)
                # Skip the empty placeholder so a fresh launch doesn't
                # litter Untitled.json.
                if program is None or (not program.elements and name.startswith('Untitled')):
                    continue
                for _, prog_dir in self._targets:
                    self._try(
                        os.path.join(prog_dir, program_filename(name)),
                        lambda p, prog=program: save_program(p, prog),
                    )

    @staticmethod
    def _try(path: str, write) -> None:
        try:
            _atomic_write(path, write)
        except OSError as exc:
            _log.warning(f'Autosave to {path} failed: {exc}')
