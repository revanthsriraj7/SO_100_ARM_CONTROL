#!/usr/bin/env python3
"""Entry point: `ros2 run so101_dashboard dashboard [--mock] [--config-dir PATH]`"""
from __future__ import annotations

import argparse
import sys

from .app.application import Application


def main(argv=None) -> int:
    raw_args = argv if argv is not None else sys.argv[1:]
    # `ros2 launch`'s Node() action always appends ROS-specific args
    # (--ros-args -r __node:=... etc) to the command line. Strip those
    # before argparse sees them, or launching via dashboard.launch.py
    # fails with "unrecognized arguments" every time.
    try:
        from rclpy.utilities import remove_ros_args
        # remove_ros_args() expects a full argv (program name at [0]) and
        # returns one in the same shape, so pad/unpad around the call.
        raw_args = remove_ros_args(args=['dashboard'] + raw_args)[1:]
    except ImportError:
        pass

    parser = argparse.ArgumentParser(description='SO-101 control/telemetry dashboard')
    parser.add_argument(
        '--mock', action='store_true',
        help='Run against a synthetic joint-state generator instead of a real ROS 2 system.',
    )
    parser.add_argument(
        '--config-dir', default=None,
        help='Override the config directory (defaults to the installed share/so101_dashboard/config).',
    )
    args = parser.parse_args(raw_args)

    app = Application(mock_mode=args.mock, config_dir=args.config_dir)
    return app.run()


if __name__ == '__main__':
    sys.exit(main())
