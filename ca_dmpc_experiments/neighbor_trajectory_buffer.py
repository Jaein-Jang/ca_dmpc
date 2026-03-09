#!/usr/bin/env python3
"""Filter neighbor predicted trajectories within communication radius."""

from __future__ import annotations

import json
import math
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


Point2D = Tuple[float, float]


def normalize_topic(topic: str) -> str:
    """Ensure topic has leading slash."""
    return topic if topic.startswith('/') else f'/{topic}'


def parse_robot_names(csv_value: str) -> List[str]:
    """Parse comma-separated robot names."""
    return [name.strip() for name in csv_value.split(',') if name.strip()]


def distance(a: Point2D, b: Point2D) -> float:
    """Euclidean distance between 2D points."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


class NeighborTrajectoryBuffer(Node):
    """Maintain latest trajectories and publish only nearby neighbors."""

    def __init__(self) -> None:
        super().__init__('neighbor_trajectory_buffer')

        self.declare_parameter('robot_name', '')
        self.declare_parameter('robot_names_csv', 'tb1,tb2,tb3')
        self.declare_parameter('fleet_state_topic', '/fleet_states')
        self.declare_parameter('trajectory_topic_suffix', '/predicted_trajectory')
        self.declare_parameter('publish_topic', '/neighbor_trajectories')
        self.declare_parameter('communication_radius', 3.0)
        self.declare_parameter('publish_rate', 5.0)

        self._robot_name = str(self.get_parameter('robot_name').value)
        self._robot_names = parse_robot_names(str(self.get_parameter('robot_names_csv').value))
        self._fleet_state_topic = normalize_topic(str(self.get_parameter('fleet_state_topic').value))
        self._trajectory_topic_suffix = normalize_topic(
            str(self.get_parameter('trajectory_topic_suffix').value)
        )
        self._publish_topic = normalize_topic(str(self.get_parameter('publish_topic').value))
        self._communication_radius = max(
            0.0, float(self.get_parameter('communication_radius').value)
        )
        publish_rate = max(1e-3, float(self.get_parameter('publish_rate').value))

        self._latest_positions: Dict[str, Point2D] = {}
        self._latest_trajectories: Dict[str, dict] = {}

        self._fleet_sub = self.create_subscription(
            String,
            self._fleet_state_topic,
            self._fleet_state_callback,
            10,
        )

        self._trajectory_subscriptions = []
        for robot in self._robot_names:
            topic = f'/{robot}{self._trajectory_topic_suffix}'
            sub = self.create_subscription(
                String,
                topic,
                lambda msg, robot_name=robot: self._trajectory_callback(robot_name, msg),
                10,
            )
            self._trajectory_subscriptions.append(sub)

        self._publisher = self.create_publisher(String, self._publish_topic, 10)
        self._timer = self.create_timer(1.0 / publish_rate, self._publish_neighbors)

    def _fleet_state_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Failed to parse fleet state JSON; skipping update.')
            return

        robot_states = payload.get('robot_states', [])
        if not isinstance(robot_states, list):
            return

        new_positions: Dict[str, Point2D] = {}
        for entry in robot_states:
            if not isinstance(entry, dict):
                continue
            name = str(entry.get('robot_name', ''))
            if not name:
                continue
            try:
                x = float(entry.get('x', 0.0))
                y = float(entry.get('y', 0.0))
            except (TypeError, ValueError):
                continue
            new_positions[name] = (x, y)

        self._latest_positions = new_positions

    def _trajectory_callback(self, robot_name: str, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'Failed to parse trajectory JSON for {robot_name}.')
            return

        if isinstance(payload, dict):
            self._latest_trajectories[robot_name] = payload

    def _build_payload(self) -> dict:
        now = self.get_clock().now().nanoseconds * 1e-9
        payload = {
            'timestamp': now,
            'robot_name': self._robot_name,
            'communication_radius': self._communication_radius,
            'status': 'ok',
            'neighbors': [],
        }

        target_pos: Optional[Point2D] = self._latest_positions.get(self._robot_name)
        if target_pos is None:
            payload['status'] = 'target_missing_in_fleet_state'
            return payload

        neighbors = []
        for robot_name in self._robot_names:
            if robot_name == self._robot_name:
                continue

            neighbor_pos = self._latest_positions.get(robot_name)
            if neighbor_pos is None:
                continue

            dist = distance(target_pos, neighbor_pos)
            if dist > self._communication_radius:
                continue

            trajectory_payload = self._latest_trajectories.get(robot_name, {})
            trajectory = trajectory_payload.get('trajectory', [])
            if not isinstance(trajectory, list):
                trajectory = []

            neighbors.append(
                {
                    'robot_name': robot_name,
                    'distance': dist,
                    'trajectory': trajectory,
                }
            )

        payload['neighbors'] = neighbors
        return payload

    def _publish_neighbors(self) -> None:
        msg = String()
        msg.data = json.dumps(self._build_payload(), separators=(',', ':'))
        self._publisher.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = NeighborTrajectoryBuffer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
