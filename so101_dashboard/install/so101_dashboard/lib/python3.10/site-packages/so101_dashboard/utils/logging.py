"""App-wide logging setup.

Plain stdlib `logging`, not rclpy's logger, so that non-ROS modules
(TorqueManager, WaypointManager, ...) can log without needing a live
rclpy node reference, and so unit tests can capture log output without
initializing rclpy at all. The ROS worker thread additionally has access
to rclpy's own node logger for ROS-specific diagnostics; both are fine to
use side by side.
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
    handler.setFormatter(
        logging.Formatter(
            fmt='[%(asctime)s] [%(levelname)s] [%(name)s] %(message)s',
            datefmt='%H:%M:%S',
        )
    )
    root = logging.getLogger('so101_dashboard')
    root.setLevel(level)
    root.addHandler(handler)
    root.propagate = False
    _CONFIGURED = True


def get_logger(name: str) -> logging.Logger:
    """Get a logger under the 'so101_dashboard' namespace.

    Configures the root app logger on first use so callers don't need to
    remember to call configure_logging() themselves in tests/scripts.
    """
    configure_logging()
    return logging.getLogger(f'so101_dashboard.{name}')
