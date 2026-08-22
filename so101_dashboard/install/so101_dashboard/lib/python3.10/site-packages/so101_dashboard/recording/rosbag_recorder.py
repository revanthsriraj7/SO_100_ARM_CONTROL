"""Records selected topics to a real rosbag2 bag via rosbag2_py's
SequentialWriter -- the official Python rosbag2 API, not a hand-rolled
file format and not a `ros2 bag record` subprocess.

Runs on the ROS worker thread (it's constructed with and creates
subscriptions on the DashboardNode's own Node), driven the same way as
everything else in ros_node.py.
"""
from __future__ import annotations

import time
from typing import List, Tuple

import rosbag2_py
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message

from ..utils.logging import get_logger

_log = get_logger('recording.rosbag_recorder')


class RosbagRecorder:
    def __init__(self, node: Node) -> None:
        self._node = node
        self._writer = None
        self._subscriptions = []
        self._recording = False
        self._start_time = 0.0
        self._bag_path = ''
        self._message_count = 0

    @property
    def is_recording(self) -> bool:
        return self._recording

    @property
    def elapsed_seconds(self) -> float:
        return (time.monotonic() - self._start_time) if self._recording else 0.0

    @property
    def bag_path(self) -> str:
        return self._bag_path

    @property
    def message_count(self) -> int:
        return self._message_count

    def start(self, topics: List[str], output_dir: str) -> Tuple[bool, str]:
        if self._recording:
            return False, 'Already recording'

        available = dict(self._node.get_topic_names_and_types())
        storage_options = rosbag2_py.StorageOptions(uri=output_dir, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')

        writer = rosbag2_py.SequentialWriter()
        try:
            writer.open(storage_options, converter_options)
        except Exception as exc:  # noqa: BLE001
            return False, f'Failed to open bag at {output_dir}: {exc}'

        subscriptions = []
        recorded_topics = []
        skipped_topics = []
        for topic in topics:
            types = available.get(topic)
            if not types:
                skipped_topics.append(topic)
                continue
            type_str = types[0]
            try:
                msg_class = get_message(type_str)
            except Exception as exc:  # noqa: BLE001
                _log.error(f"Could not resolve message type '{type_str}' for {topic}: {exc}")
                skipped_topics.append(topic)
                continue

            writer.create_topic(rosbag2_py.TopicMetadata(topic, type_str, 'cdr'))
            sub = self._node.create_subscription(
                msg_class, topic, self._make_callback(writer, topic), 50,
            )
            subscriptions.append(sub)
            recorded_topics.append(topic)

        if not subscriptions:
            return False, f'No requested topics are currently active: {topics}'

        self._writer = writer
        self._subscriptions = subscriptions
        self._recording = True
        self._start_time = time.monotonic()
        self._bag_path = output_dir
        self._message_count = 0

        msg = f'Recording {len(recorded_topics)} topics to {output_dir}'
        if skipped_topics:
            msg += f' (skipped, not currently active: {skipped_topics})'
        _log.info(msg)
        return True, msg

    def _make_callback(self, writer, topic: str):
        def _callback(msg) -> None:
            if not self._recording:
                return
            data = serialize_message(msg)
            stamp_ns = self._node.get_clock().now().nanoseconds
            writer.write(topic, data, stamp_ns)
            self._message_count += 1
        return _callback

    def stop(self) -> Tuple[bool, str]:
        if not self._recording:
            return False, 'Not recording'
        self._recording = False
        for sub in self._subscriptions:
            self._node.destroy_subscription(sub)
        self._subscriptions = []
        elapsed = time.monotonic() - self._start_time
        path = self._bag_path
        count = self._message_count
        self._writer = None  # SequentialWriter finalizes/closes the bag on destruction
        _log.info(f'Rosbag recording stopped after {elapsed:.1f}s, {count} messages, at {path}')
        return True, f'Stopped after {elapsed:.1f}s ({count} messages) -- {path}'
