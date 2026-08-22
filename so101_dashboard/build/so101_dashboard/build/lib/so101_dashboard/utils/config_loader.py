"""Locates and loads the YAML config files.

Tries the installed share directory first (normal `ros2 run` usage), then
falls back to a path relative to this file (running from source / tests,
no colcon install needed).
"""
from __future__ import annotations

import os
from typing import Optional

import yaml


def _share_config_dir() -> Optional[str]:
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('so101_dashboard'), 'config')
    except Exception:  # noqa: BLE001 - ament not available (e.g. plain pytest run)
        return None


def _source_config_dir() -> str:
    # .../so101_dashboard/so101_dashboard/utils/config_loader.py -> .../so101_dashboard/config
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, '..', '..', 'config'))


def find_config_dir(override: Optional[str] = None) -> str:
    if override:
        return override
    share_dir = _share_config_dir()
    if share_dir and os.path.isdir(share_dir):
        return share_dir
    return _source_config_dir()


def load_yaml(config_dir: str, filename: str) -> dict:
    path = os.path.join(config_dir, filename)
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}
