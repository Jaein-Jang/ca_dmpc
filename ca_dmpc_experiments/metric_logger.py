#!/usr/bin/env python3
"""Log fleet state streams and derive deadlock intervals."""

from __future__ import annotations

import csv
import json
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


@dataclass
class DeadlockInterval:
    """A detected low-speed deadlock interval."""

    start_time: float
    end_time: float

    @property
    def duration(self) -> float:
        return self.end_time - self.start_time


class MetricLogger(Node):
    """Node that logs fleet states and detects deadlock intervals."""

    def __init__(self) -> None:
        super().__init__('metric_logger')

        self.declare_parameter('fleet_state_topic', '/fleet_states')
        self.declare_parameter('log_dir', '/tmp/ca_dmpc_logs')
        self.declare_parameter('deadlock_speed_threshold', 0.05)
        self.declare_parameter('deadlock_duration_sec', 5.0)
        self.declare_parameter('goal_radius', 0.3)

        fleet_state_topic = str(self.get_parameter('fleet_state_topic').value)
        self._fleet_state_topic = self._normalize_topic(fleet_state_topic)
        self._log_dir = Path(str(self.get_parameter('log_dir').value))
        self._deadlock_speed_threshold = float(self.get_parameter('deadlock_speed_threshold').value)
        self._deadlock_duration_sec = float(self.get_parameter('deadlock_duration_sec').value)
        # Reserved for future goal-completion metrics; not used in current logger logic.
        self._goal_radius = float(self.get_parameter('goal_radius').value)

        self._last_timestamp: Optional[float] = None
        self._low_speed_start: Optional[float] = None
        self._active_deadlock_start: Optional[float] = None

        self._state_log_file = None
        self._fleet_speed_file = None
        self._deadlock_file = None
        self._state_writer: Optional[csv.writer] = None
        self._fleet_speed_writer: Optional[csv.writer] = None
        self._deadlock_writer: Optional[csv.writer] = None

        self._prepare_log_files()

        self._subscription = self.create_subscription(
            String,
            self._fleet_state_topic,
            self._fleet_state_callback,
            10,
        )

        self.get_logger().info(
            f'Logging fleet metrics from {self._fleet_state_topic} into {self._log_dir}'
        )
        self.get_logger().info(
            f'Deadlock threshold={self._deadlock_speed_threshold:.3f} m/s, '
            f'duration={self._deadlock_duration_sec:.2f}s, goal_radius={self._goal_radius:.2f}m'
        )

    @staticmethod
    def _normalize_topic(topic: str) -> str:
        return topic if topic.startswith('/') else f'/{topic}'

    def _prepare_log_files(self) -> None:
        self._log_dir.mkdir(parents=True, exist_ok=True)

        run_stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        state_path = self._log_dir / f'fleet_state_timeseries_{run_stamp}.csv'
        speed_path = self._log_dir / f'fleet_average_speed_{run_stamp}.csv'
        deadlock_path = self._log_dir / f'deadlock_intervals_{run_stamp}.csv'

        self._state_log_file = state_path.open('w', newline='', encoding='utf-8')
        self._fleet_speed_file = speed_path.open('w', newline='', encoding='utf-8')
        self._deadlock_file = deadlock_path.open('w', newline='', encoding='utf-8')

        self._state_writer = csv.writer(self._state_log_file)
        self._fleet_speed_writer = csv.writer(self._fleet_speed_file)
        self._deadlock_writer = csv.writer(self._deadlock_file)

        self._state_writer.writerow(
            ['timestamp', 'robot_name', 'x', 'y', 'yaw', 'linear_speed', 'angular_speed']
        )
        self._fleet_speed_writer.writerow(['timestamp', 'fleet_average_speed'])
        self._deadlock_writer.writerow(['start_time', 'end_time', 'duration'])

        self._flush_all()

    def _fleet_state_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Invalid JSON in fleet state message: {exc}')
            return

        timestamp = self._extract_timestamp(payload)
        robot_states = payload.get('robot_states', [])
        if not isinstance(robot_states, list):
            self.get_logger().warn('fleet state payload has non-list "robot_states". Skipping message.')
            return

        if self._state_writer is None or self._fleet_speed_writer is None:
            self.get_logger().error('CSV writers are not initialized; skipping log write.')
            return

        sum_speed = 0.0
        count = 0

        for robot_state in robot_states:
            if not isinstance(robot_state, dict):
                continue

            robot_name = str(robot_state.get('robot_name', ''))
            x = self._safe_float(robot_state.get('x'))
            y = self._safe_float(robot_state.get('y'))
            yaw = self._safe_float(robot_state.get('yaw'))
            linear_speed = self._safe_float(robot_state.get('linear_speed'))
            angular_speed = self._safe_float(robot_state.get('angular_speed'))

            self._state_writer.writerow(
                [timestamp, robot_name, x, y, yaw, linear_speed, angular_speed]
            )

            sum_speed += linear_speed
            count += 1

        fleet_avg_speed = (sum_speed / count) if count > 0 else 0.0
        self._fleet_speed_writer.writerow([timestamp, fleet_avg_speed])

        if count > 0:
            self._update_deadlock_state(timestamp, fleet_avg_speed)
        else:
            self.get_logger().warn(
                'Skipping deadlock detection: no valid robot states in fleet message.'
            )

        self._last_timestamp = timestamp
        self._flush_all()

    def _extract_timestamp(self, payload: dict) -> float:
        timestamp = payload.get('timestamp')
        ts = self._safe_float(timestamp)
        if ts <= 0.0:
            ts = self.get_clock().now().nanoseconds * 1e-9
        return ts

    @staticmethod
    def _safe_float(value: object) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return 0.0

    def _update_deadlock_state(self, timestamp: float, fleet_avg_speed: float) -> None:
        if fleet_avg_speed < self._deadlock_speed_threshold:
            if self._low_speed_start is None:
                self._low_speed_start = timestamp

            if (
                self._active_deadlock_start is None
                and (timestamp - self._low_speed_start) >= self._deadlock_duration_sec
            ):
                self._active_deadlock_start = self._low_speed_start
                self.get_logger().warn(
                    f'Deadlock started at {self._active_deadlock_start:.3f} '
                    f'(avg speed {fleet_avg_speed:.3f} m/s).'
                )
            return

        if self._active_deadlock_start is not None:
            self._record_deadlock(DeadlockInterval(self._active_deadlock_start, timestamp))
            self._active_deadlock_start = None

        self._low_speed_start = None

    def _record_deadlock(self, interval: DeadlockInterval) -> None:
        if self._deadlock_writer is None:
            self.get_logger().error('Deadlock writer is not initialized.')
            return

        self._deadlock_writer.writerow(
            [interval.start_time, interval.end_time, interval.duration]
        )
        self.get_logger().info(
            f'Deadlock interval recorded: {interval.start_time:.3f} -> '
            f'{interval.end_time:.3f} ({interval.duration:.3f}s)'
        )

    def _finalize_deadlock_state(self) -> None:
        if self._active_deadlock_start is None:
            return

        end_time = self._last_timestamp
        if end_time is None:
            end_time = self.get_clock().now().nanoseconds * 1e-9

        self._record_deadlock(DeadlockInterval(self._active_deadlock_start, end_time))
        self._active_deadlock_start = None

    def _flush_all(self) -> None:
        for file_handle in (self._state_log_file, self._fleet_speed_file, self._deadlock_file):
            if file_handle is not None:
                file_handle.flush()

    def _close_files(self) -> None:
        for file_handle in (self._state_log_file, self._fleet_speed_file, self._deadlock_file):
            if file_handle is not None:
                file_handle.close()

    def destroy_node(self) -> bool:
        self._finalize_deadlock_state()
        self._flush_all()
        self._close_files()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MetricLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
