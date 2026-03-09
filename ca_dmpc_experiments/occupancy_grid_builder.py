#!/usr/bin/env python3
"""Build sparse time-indexed occupancy maps from neighbor trajectories."""

from __future__ import annotations

import json
import math
from collections import defaultdict
from typing import DefaultDict, Dict, Iterable, List, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


Cell = Tuple[int, int]
TimeCells = DefaultDict[int, DefaultDict[Cell, float]]


def normalize_topic(topic: str) -> str:
    """Ensure topic has leading slash."""
    return topic if topic.startswith('/') else f'/{topic}'


class OccupancyGridBuilder(Node):
    """Convert neighbor trajectory predictions into sparse occupancy maps."""

    def __init__(self) -> None:
        super().__init__('occupancy_grid_builder')

        self.declare_parameter('robot_name', '')
        self.declare_parameter('neighbor_topic', '/neighbor_trajectories')
        self.declare_parameter('publish_topic', '/occupancy_debug')
        self.declare_parameter('grid_resolution', 0.1)
        self.declare_parameter('horizon_steps', 15)
        self.declare_parameter('smoothing_enabled', True)
        self.declare_parameter('publish_rate', 5.0)

        self._robot_name = str(self.get_parameter('robot_name').value)
        self._neighbor_topic = normalize_topic(str(self.get_parameter('neighbor_topic').value))
        self._publish_topic = normalize_topic(str(self.get_parameter('publish_topic').value))
        self._grid_resolution = max(1e-6, float(self.get_parameter('grid_resolution').value))
        self._horizon_steps = max(1, int(self.get_parameter('horizon_steps').value))
        self._smoothing_enabled = bool(self.get_parameter('smoothing_enabled').value)
        publish_rate = max(1e-3, float(self.get_parameter('publish_rate').value))

        self._latest_neighbor_payload: dict = {}

        self._neighbor_sub = self.create_subscription(
            String,
            self._neighbor_topic,
            self._neighbor_callback,
            10,
        )
        self._publisher = self.create_publisher(String, self._publish_topic, 10)
        self._timer = self.create_timer(1.0 / publish_rate, self._publish_occupancy)

    def _neighbor_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Failed to parse neighbor trajectory JSON.')
            return

        if isinstance(payload, dict):
            self._latest_neighbor_payload = payload

    def _to_cell(self, x: float, y: float) -> Cell:
        ix = int(math.floor(x / self._grid_resolution))
        iy = int(math.floor(y / self._grid_resolution))
        return ix, iy

    @staticmethod
    def _smoothing_kernel() -> Iterable[Tuple[int, int, float]]:
        # 3x3 kernel: center strongest, cardinal medium, diagonal weak.
        return [
            (-1, -1, 0.25), (0, -1, 0.5), (1, -1, 0.25),
            (-1, 0, 0.5), (0, 0, 1.0), (1, 0, 0.5),
            (-1, 1, 0.25), (0, 1, 0.5), (1, 1, 0.25),
        ]

    def _apply_cell(self, grid: DefaultDict[Cell, float], ix: int, iy: int, value: float) -> None:
        if self._smoothing_enabled:
            for dx, dy, weight in self._smoothing_kernel():
                grid[(ix + dx, iy + dy)] += value * weight
        else:
            grid[(ix, iy)] += value

    def _build_occupancy(self) -> List[dict]:
        neighbors = self._latest_neighbor_payload.get('neighbors', [])
        if not isinstance(neighbors, list):
            neighbors = []

        occupancy: TimeCells = defaultdict(lambda: defaultdict(float))

        for neighbor in neighbors:
            if not isinstance(neighbor, dict):
                continue
            trajectory = neighbor.get('trajectory', [])
            if not isinstance(trajectory, list):
                continue

            for point in trajectory:
                if not isinstance(point, dict):
                    continue

                try:
                    t_index = int(point.get('t_index', 0))
                    if t_index < 0 or t_index >= self._horizon_steps:
                        continue
                    x = float(point.get('x', 0.0))
                    y = float(point.get('y', 0.0))
                except (TypeError, ValueError):
                    continue

                ix, iy = self._to_cell(x, y)
                self._apply_cell(occupancy[t_index], ix, iy, 1.0)

        occupancy_list: List[dict] = []
        for t_index in range(self._horizon_steps):
            cells_map = occupancy.get(t_index, defaultdict(float))
            cells = [
                {'ix': ix, 'iy': iy, 'value': value}
                for (ix, iy), value in cells_map.items()
                if value > 0.0
            ]
            cells.sort(key=lambda cell: (cell['ix'], cell['iy']))
            occupancy_list.append({'t_index': t_index, 'cells': cells})

        return occupancy_list

    def _build_payload(self) -> dict:
        return {
            'timestamp': self.get_clock().now().nanoseconds * 1e-9,
            'robot_name': self._robot_name,
            'grid_resolution': self._grid_resolution,
            'horizon_steps': self._horizon_steps,
            'occupancy': self._build_occupancy(),
        }

    def _publish_occupancy(self) -> None:
        msg = String()
        msg.data = json.dumps(self._build_payload(), separators=(',', ':'))
        self._publisher.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OccupancyGridBuilder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
