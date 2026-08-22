"""Package-wide logging setup, mirroring so101_dashboard/utils/logging.py's
pattern: plain stdlib logging (not rclpy's), configured once, so every
module in this package can just call get_logger(__name__-ish) without an
rclpy node reference.
"""
from __future__ import annotations

import logging
import sys

_CONFIGURED = False


def configure_logging(level: int = logging.INFO) -> None:
    global _CONFIGURED
    if _CONFIGURED:
        return
    handler = logging.StreamHandler(stream=sys.stdout)
    handler.setFormatter(logging.Formatter(fmt='[%(asctime)s] %(message)s', datefmt='%H:%M:%S'))
    root = logging.getLogger('so_100_voice_control')
    root.setLevel(level)
    root.addHandler(handler)
    root.propagate = False
    _CONFIGURED = True


def get_logger(name: str) -> logging.Logger:
    configure_logging()
    return logging.getLogger(f'so_100_voice_control.{name}')
