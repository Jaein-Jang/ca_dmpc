#!/usr/bin/env python3
"""Broadcast naive predicted trajectories derived from odom and global path."""

from __future__ import annotations

import json
import math
from typing import List, Optional, Sequence, Tuple

import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import String


Point2D = Tuple[float, float]


def distance(a: Point2D, b: Point2D) -> float:
    """Return Euclidean distance between two 2D points."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def normalize_topic(topic: str) -> str:
    """Ensure topic begins with a leading slash."""
    return topic if topic.startswith('/') else f'/{topic}'


def yaw_from_points(a: Point2D, b: Point2D) -> float:
    """Compute heading from point a to point b."""
    return math.atan2(b[1] - a[1], b[0] - a[0])


class TrajectoryBroadcaster(Node):
    """Generate and publish a simple predicted trajectory in JSON format."""

    def __init__(self) -> None:
        super().__init__('trajectory_broadcaster')

        self.declare_parameter('robot_name', '')
        self.declare_parameter('path_topic', '/plan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('publish_topic', '/predicted_trajectory')
        self.declare_parameter('horizon_steps', 15)
        self.declare_parameter('dt', 0.2)
        self.declare_parameter('nominal_speed', 0.2)
        self.declare_parameter('publish_rate', 5.0)

        self._robot_name = str(self.get_parameter('robot_name').value)
        self._path_topic = normalize_topic(str(self.get_parameter('path_topic').value))
        self._odom_topic = normalize_topic(str(self.get_parameter('odom_topic').value))
        self._publish_topic = normalize_topic(str(self.get_parameter('publish_topic').value))
        self._horizon_steps = max(1, int(self.get_parameter('horizon_steps').value))
        self._dt = max(1e-3, float(self.get_parameter('dt').value))
        self._nominal_speed = max(0.0, float(self.get_parameter('nominal_speed').value))
        publish_rate = max(1e-3, float(self.get_parameter('publish_rate').value))

        self._path_points: List[Point2D] = []
        self._odom_position: Optional[Point2D] = None

        self._path_sub = self.create_subscription(Path, self._path_topic, self._path_callback, 10)
        self._odom_sub = self.create_subscription(Odometry, self._odom_topic, self._odom_callback, 10)
        self._pub = self.create_publisher(String, self._publish_topic, 10)
        self._timer = self.create_timer(1.0 / publish_rate, self._publish_prediction)

    def _path_callback(self, msg: Path) -> None:
        self._path_points = [
            (pose_stamped.pose.position.x, pose_stamped.pose.position.y)
            for pose_stamped in msg.poses
        ]

    def _odom_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self._odom_position = (pose.position.x, pose.position.y)

    @staticmethod
    def _nearest_path_index(position: Point2D, path_points: Sequence[Point2D]) -> int:
        return min(range(len(path_points)), key=lambda idx: distance(position, path_points[idx]))

    def _sample_forward_path(
        self,
        path_points: Sequence[Point2D],
        start_index: int,
    ) -> List[Point2D]:
        if len(path_points) == 1:
            return [path_points[0] for _ in range(self._horizon_steps)]

        segment_lengths: List[float] = [0.0]
        for idx in range(start_index + 1, len(path_points)):
            segment_lengths.append(
                segment_lengths[-1] + distance(path_points[idx - 1], path_points[idx])
            )

        sampled_points: List[Point2D] = []
        step_distance = self._nominal_speed * self._dt
        tail_points = list(path_points[start_index:])

        for k in range(self._horizon_steps):
            target_distance = k * step_distance
            sampled_points.append(
                self._interpolate_along_tail(tail_points, segment_lengths, target_distance)
            )

        return sampled_points

    @staticmethod
    def _interpolate_along_tail(
        tail_points: Sequence[Point2D],
        cumulative_lengths: Sequence[float],
        target_distance: float,
    ) -> Point2D:
        if len(tail_points) == 1:
            return tail_points[0]

        if target_distance <= 0.0:
            return tail_points[0]

        if target_distance >= cumulative_lengths[-1]:
            return tail_points[-1]

        for idx in range(1, len(cumulative_lengths)):
            prev_len = cumulative_lengths[idx - 1]
            next_len = cumulative_lengths[idx]
            if target_distance <= next_len:
                ratio = (target_distance - prev_len) / max(next_len - prev_len, 1e-9)
                p0 = tail_points[idx - 1]
                p1 = tail_points[idx]
                x = p0[0] + ratio * (p1[0] - p0[0])
                y = p0[1] + ratio * (p1[1] - p0[1])
                return (x, y)

        return tail_points[-1]

    @staticmethod
    def _estimate_yaws(points: Sequence[Point2D]) -> List[float]:
        if not points:
            return []
        if len(points) == 1:
            return [0.0]

        yaws: List[float] = []
        for idx in range(len(points)):
            if idx < len(points) - 1:
                yaws.append(yaw_from_points(points[idx], points[idx + 1]))
            else:
                yaws.append(yaws[-1])
        return yaws

    def _build_payload(self) -> dict:
        now = self.get_clock().now().nanoseconds * 1e-9
        payload = {
            'timestamp': now,
            'robot_name': self._robot_name,
            'status': 'ok',
            'dt': self._dt,
            'horizon_steps': self._horizon_steps,
            'trajectory': [],
        }

        if self._odom_position is None:
            payload['status'] = 'missing_odom'
            return payload

        if not self._path_points:
            payload['status'] = 'missing_path'
            return payload

        nearest_idx = self._nearest_path_index(self._odom_position, self._path_points)
        sampled_points = self._sample_forward_path(self._path_points, nearest_idx)
        sampled_yaws = self._estimate_yaws(sampled_points)

        payload['trajectory'] = [
            {
                't_index': idx,
                'x': point[0],
                'y': point[1],
                'yaw': sampled_yaws[idx],
            }
            for idx, point in enumerate(sampled_points)
        ]
        return payload

    def _publish_prediction(self) -> None:
        msg = String()
        msg.data = json.dumps(self._build_payload(), separators=(',', ':'))
        self._pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TrajectoryBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
