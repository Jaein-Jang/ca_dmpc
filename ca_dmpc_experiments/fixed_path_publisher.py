#!/usr/bin/env python3
"""Publish deterministic fixed paths from JSON-encoded path points."""

from __future__ import annotations

import json
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node


Point2D = Tuple[float, float]


def normalize_topic(topic: str) -> str:
    """Ensure topic name has a leading slash."""
    return topic if topic.startswith('/') else f'/{topic}'


class FixedPathPublisher(Node):
    """Republish a static path at a fixed rate for repeatable benchmark testing."""

    def __init__(self) -> None:
        super().__init__('fixed_path_publisher')

        self.declare_parameter('robot_name', '')
        self.declare_parameter('publish_topic', '/plan')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('path_points', '[]')

        self._robot_name = str(self.get_parameter('robot_name').value)
        self._publish_topic = normalize_topic(str(self.get_parameter('publish_topic').value))
        self._frame_id = str(self.get_parameter('frame_id').value)
        publish_rate = max(1e-3, float(self.get_parameter('publish_rate').value))
        path_points_raw = str(self.get_parameter('path_points').value)

        self._path_points = self._parse_path_points(path_points_raw)
        self._path_valid = self._path_points is not None

        self._publisher = self.create_publisher(Path, self._publish_topic, 10)
        self._timer = self.create_timer(1.0 / publish_rate, self._publish_path)

        if self._path_valid:
            self.get_logger().info(
                f'FixedPathPublisher[{self._robot_name or "unnamed"}] publishing '
                f'{len(self._path_points)} points on {self._publish_topic}'
            )

    def _parse_path_points(self, raw: str) -> Optional[List[Point2D]]:
        try:
            parsed = json.loads(raw)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Failed to parse "path_points" JSON: {exc}')
            return None

        if not isinstance(parsed, list):
            self.get_logger().error('Parameter "path_points" must be a JSON list of {x,y} objects.')
            return None

        points: List[Point2D] = []
        for idx, item in enumerate(parsed):
            if not isinstance(item, dict):
                self.get_logger().error(
                    f'Invalid path point at index {idx}: expected object with x and y.'
                )
                return None
            try:
                x = float(item['x'])
                y = float(item['y'])
            except (KeyError, TypeError, ValueError):
                self.get_logger().error(
                    f'Invalid path point at index {idx}: expected numeric x and y fields.'
                )
                return None
            points.append((x, y))

        if not points:
            self.get_logger().error('Parameter "path_points" decoded successfully but is empty.')
            return None

        return points

    def _build_path_msg(self) -> Path:
        now_msg = self.get_clock().now().to_msg()

        path_msg = Path()
        path_msg.header.stamp = now_msg
        path_msg.header.frame_id = self._frame_id

        poses: List[PoseStamped] = []
        for x, y in self._path_points or []:
            pose = PoseStamped()
            pose.header.stamp = now_msg
            pose.header.frame_id = self._frame_id
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            poses.append(pose)

        path_msg.poses = poses
        return path_msg

    def _publish_path(self) -> None:
        if not self._path_valid:
            return
        self._publisher.publish(self._build_path_msg())


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FixedPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
