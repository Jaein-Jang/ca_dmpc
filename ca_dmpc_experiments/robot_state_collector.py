#!/usr/bin/env python3
"""Collect per-robot odometry and publish aggregated fleet state as JSON."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


@dataclass
class RobotState:
    """Latest state snapshot for one robot."""

    robot_name: str
    x: float
    y: float
    yaw: float
    linear_speed: float
    angular_speed: float
    stamp_sec: float


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Convert quaternion orientation to yaw (rad)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class RobotStateCollector(Node):
    """Node that subscribes to robot odometry streams and publishes fleet state."""

    def __init__(self) -> None:
        super().__init__('robot_state_collector')

        self.declare_parameter('robot_names_csv', 'tb1,tb2,tb3')
        self.declare_parameter('odom_topic_suffix', '/odom')
        self.declare_parameter('publish_topic', '/fleet_states')
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('robot_offsets_json', '{}')

        robot_names_csv = str(self.get_parameter('robot_names_csv').value)
        self._robot_names = [name.strip() for name in robot_names_csv.split(',') if name.strip()]
        odom_topic_suffix = str(self.get_parameter('odom_topic_suffix').value)
        publish_topic = str(self.get_parameter('publish_topic').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        robot_offsets_json = str(self.get_parameter('robot_offsets_json').value)

        self._odom_topic_suffix = self._normalize_topic_suffix(odom_topic_suffix)
        self._publish_topic = self._normalize_topic(publish_topic)
        self._publish_period = 1.0 / publish_rate if publish_rate > 0.0 else 0.2

        self._robot_offsets = self._parse_robot_offsets(robot_offsets_json)
        self._latest_states: Dict[str, RobotState] = {}
        self._warning_sent: Dict[str, bool] = {name: False for name in self._robot_names}
        self._offset_warning_sent: Dict[str, bool] = {name: False for name in self._robot_names}

        self._publisher = self.create_publisher(String, self._publish_topic, 10)
        self._subscriptions = []
        for robot_name in self._robot_names:
            topic = f'/{robot_name}{self._odom_topic_suffix}'
            subscription = self.create_subscription(
                Odometry,
                topic,
                lambda msg, name=robot_name: self._odom_callback(name, msg),
                10,
            )
            self._subscriptions.append(subscription)
            self.get_logger().info(f'Subscribed to {topic}')

        self._publish_timer = self.create_timer(self._publish_period, self._publish_fleet_state)

        self.get_logger().info(
            f'Publishing aggregated fleet state to {self._publish_topic} '
            f'at {1.0 / self._publish_period:.2f} Hz'
        )

    @staticmethod
    def _normalize_topic(topic: str) -> str:
        return topic if topic.startswith('/') else f'/{topic}'

    @staticmethod
    def _normalize_topic_suffix(suffix: str) -> str:
        return suffix if suffix.startswith('/') else f'/{suffix}'

    def _parse_robot_offsets(self, raw_json: str) -> Dict[str, Tuple[float, float]]:
        try:
            parsed = json.loads(raw_json)
        except json.JSONDecodeError as exc:
            self.get_logger().error(
                f'Failed to parse "robot_offsets_json": {exc}. Using zero offsets for all robots.'
            )
            return {}

        if not isinstance(parsed, dict):
            self.get_logger().error(
                'Parameter "robot_offsets_json" must decode to a JSON object. '
                'Using zero offsets for all robots.'
            )
            return {}

        offsets: Dict[str, Tuple[float, float]] = {}
        for robot_name, value in parsed.items():
            if not isinstance(robot_name, str) or not isinstance(value, dict):
                continue

            try:
                ox = float(value.get('x', 0.0))
                oy = float(value.get('y', 0.0))
            except (TypeError, ValueError):
                continue

            offsets[robot_name] = (ox, oy)

        return offsets

    def _get_robot_offset(self, robot_name: str) -> Tuple[float, float]:
        offset = self._robot_offsets.get(robot_name)
        if offset is not None:
            return offset

        if not self._offset_warning_sent.get(robot_name, False):
            self.get_logger().warn(
                f'No offset configured for robot "{robot_name}" in "robot_offsets_json". '
                'Using zero offset.'
            )
            self._offset_warning_sent[robot_name] = True

        return 0.0, 0.0

    def _odom_callback(self, robot_name: str, msg: Odometry) -> None:
        pose = msg.pose.pose
        twist = msg.twist.twist

        yaw = quaternion_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        linear_speed = math.sqrt(
            twist.linear.x * twist.linear.x
            + twist.linear.y * twist.linear.y
            + twist.linear.z * twist.linear.z
        )

        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

        self._latest_states[robot_name] = RobotState(
            robot_name=robot_name,
            x=pose.position.x,
            y=pose.position.y,
            yaw=yaw,
            linear_speed=linear_speed,
            angular_speed=twist.angular.z,
            stamp_sec=stamp_sec,
        )
        self._warning_sent[robot_name] = False

    def _build_robot_state_array(self) -> List[dict]:
        states: List[dict] = []
        for robot_name in self._robot_names:
            state: Optional[RobotState] = self._latest_states.get(robot_name)
            if state is None:
                if not self._warning_sent.get(robot_name, False):
                    self.get_logger().warn(
                        f'No odometry received yet for robot "{robot_name}". '
                        'It will be skipped in fleet state output until odom arrives.'
                    )
                    self._warning_sent[robot_name] = True
                continue

            offset_x, offset_y = self._get_robot_offset(robot_name)
            x = state.x + offset_x
            y = state.y + offset_y
            yaw = state.yaw

            states.append(
                {
                    'robot_name': state.robot_name,
                    'x': x,
                    'y': y,
                    'yaw': yaw,
                    'linear_speed': state.linear_speed,
                    'angular_speed': state.angular_speed,
                    'source_stamp': state.stamp_sec,
                    'position_source': 'odom_plus_offset',
                }
            )
        return states

    def _publish_fleet_state(self) -> None:
        timestamp = self.get_clock().now().nanoseconds * 1e-9
        payload = {
            'timestamp': timestamp,
            'robot_states': self._build_robot_state_array(),
        }

        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'))
        self._publisher.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RobotStateCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
