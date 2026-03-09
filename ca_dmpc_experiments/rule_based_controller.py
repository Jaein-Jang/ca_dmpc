#!/usr/bin/env python3
"""Rule-based stop/go local controller baseline for multi-robot navigation."""

from __future__ import annotations

import json
import math
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import String


Point2D = Tuple[float, float]


def clamp(value: float, minimum: float, maximum: float) -> float:
    """Clamp value into [minimum, maximum]."""
    return max(minimum, min(value, maximum))


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def distance(a: Point2D, b: Point2D) -> float:
    """Euclidean distance between two points."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Convert quaternion orientation to yaw."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_topic(topic: str) -> str:
    """Ensure topic has leading slash."""
    return topic if topic.startswith('/') else f'/{topic}'


def in_forward_cone(
    self_position: Point2D,
    self_yaw: float,
    other_position: Point2D,
    half_angle_rad: float,
) -> bool:
    """Return True if other robot is inside the forward cone."""
    dx = other_position[0] - self_position[0]
    dy = other_position[1] - self_position[1]
    bearing = math.atan2(dy, dx)
    rel = normalize_angle(bearing - self_yaw)
    return abs(rel) <= half_angle_rad


def should_yield_by_tie_break(self_name: str, other_name: str) -> bool:
    """Deterministic tie-break: lexicographically larger name yields."""
    return self_name > other_name


class RuleBasedController(Node):
    """Simple baseline local controller with stop/go collision-avoidance rules."""

    def __init__(self) -> None:
        super().__init__('rule_based_controller')

        self.declare_parameter('robot_name', '')
        self.declare_parameter('path_topic', '/plan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('fleet_state_topic', '/fleet_states')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('max_linear_speed', 0.22)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('stop_distance', 0.45)
        self.declare_parameter('slow_distance', 0.8)
        self.declare_parameter('front_cone_half_angle_deg', 25.0)
        self.declare_parameter('tie_distance_margin', 0.02)

        self._robot_name = str(self.get_parameter('robot_name').value)
        self._path_topic = normalize_topic(str(self.get_parameter('path_topic').value))
        self._odom_topic = normalize_topic(str(self.get_parameter('odom_topic').value))
        self._fleet_state_topic = normalize_topic(str(self.get_parameter('fleet_state_topic').value))
        self._cmd_vel_topic = normalize_topic(str(self.get_parameter('cmd_vel_topic').value))

        self._lookahead_distance = max(0.05, float(self.get_parameter('lookahead_distance').value))
        self._max_linear_speed = max(0.0, float(self.get_parameter('max_linear_speed').value))
        self._max_angular_speed = max(0.0, float(self.get_parameter('max_angular_speed').value))
        self._goal_tolerance = max(0.01, float(self.get_parameter('goal_tolerance').value))
        control_rate = max(1e-3, float(self.get_parameter('control_rate').value))

        self._stop_distance = max(0.0, float(self.get_parameter('stop_distance').value))
        self._slow_distance = max(
            self._stop_distance,
            float(self.get_parameter('slow_distance').value),
        )
        self._front_cone_half_angle_rad = math.radians(
            float(self.get_parameter('front_cone_half_angle_deg').value)
        )
        self._tie_distance_margin = max(0.0, float(self.get_parameter('tie_distance_margin').value))

        self._control_period = 1.0 / control_rate
        self._cmd_frame_id = f'{self._robot_name}/base_link' if self._robot_name else 'base_link'

        self._path_points: List[Point2D] = []
        self._current_pose: Optional[Tuple[float, float, float]] = None
        self._latest_fleet_positions: Dict[str, Point2D] = {}
        self._fleet_state_received = False
        self._goal_reached = False

        self._path_sub = self.create_subscription(Path, self._path_topic, self._path_callback, 10)
        self._odom_sub = self.create_subscription(Odometry, self._odom_topic, self._odom_callback, 10)
        self._fleet_sub = self.create_subscription(
            String,
            self._fleet_state_topic,
            self._fleet_state_callback,
            10,
        )
        self._cmd_pub = self.create_publisher(TwistStamped, self._cmd_vel_topic, 10)

        self._timer = self.create_timer(self._control_period, self._control_loop)

    def _path_callback(self, msg: Path) -> None:
        self._path_points = [
            (pose_stamped.pose.position.x, pose_stamped.pose.position.y)
            for pose_stamped in msg.poses
        ]
        self._goal_reached = False

    def _odom_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        yaw = quaternion_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        self._current_pose = (pose.position.x, pose.position.y, yaw)

    def _fleet_state_callback(self, msg: String) -> None:
        parsed = self._parse_fleet_state(msg.data)
        if parsed is not None:
            self._latest_fleet_positions = parsed
            self._fleet_state_received = True

    def _parse_fleet_state(self, raw_json: str) -> Optional[Dict[str, Point2D]]:
        try:
            payload = json.loads(raw_json)
        except json.JSONDecodeError:
            self.get_logger().warn('Failed to parse fleet_state JSON.')
            return None

        robot_states = payload.get('robot_states', [])
        if not isinstance(robot_states, list):
            return None

        positions: Dict[str, Point2D] = {}
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

            if name == self._robot_name:
                continue

            positions[name] = (x, y)

        return positions

    def _control_loop(self) -> None:
        if self._current_pose is None or not self._path_points or not self._fleet_state_received:
            self._publish_stop()
            return

        nominal_cmd = self._compute_nominal_command()
        if nominal_cmd is None:
            self._publish_stop()
            return

        final_cmd = self._apply_rule_overrides(nominal_cmd)
        self._cmd_pub.publish(final_cmd)


    def _compute_nominal_command(self) -> Optional[TwistStamped]:
        if self._current_pose is None or not self._path_points:
            return None

        x, y, yaw = self._current_pose
        current_position = (x, y)
        goal = self._path_points[-1]

        if distance(current_position, goal) <= self._goal_tolerance:
            self._goal_reached = True
            return self._make_stop_cmd()

        target = self._select_lookahead_target(current_position, self._path_points)
        return self._compute_path_following_cmd(current_position, yaw, target)

    def _select_lookahead_target(
        self,
        current_position: Point2D,
        path_points: Sequence[Point2D],
    ) -> Point2D:
        closest_index = min(
            range(len(path_points)),
            key=lambda idx: distance(current_position, path_points[idx]),
        )

        for point in path_points[closest_index:]:
            if distance(current_position, point) >= self._lookahead_distance:
                return point

        return path_points[-1]

    def _compute_path_following_cmd(
        self,
        current_position: Point2D,
        yaw: float,
        target: Point2D,
    ) -> TwistStamped:
        dx = target[0] - current_position[0]
        dy = target[1] - current_position[1]
        target_heading = math.atan2(dy, dx)
        alpha = normalize_angle(target_heading - yaw)

        heading_scale = max(0.0, math.cos(alpha))
        linear_speed = self._max_linear_speed * heading_scale

        lookahead = max(self._lookahead_distance, 1e-3)
        angular_speed = 2.0 * linear_speed * math.sin(alpha) / lookahead
        angular_speed = clamp(angular_speed, -self._max_angular_speed, self._max_angular_speed)

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self._cmd_frame_id
        cmd.twist.linear.x = clamp(linear_speed, 0.0, self._max_linear_speed)
        cmd.twist.angular.z = angular_speed
        return cmd

    def _apply_rule_overrides(self, nominal_cmd: TwistStamped) -> TwistStamped:
        if self._current_pose is None:
            return self._make_stop_cmd()

        self_position = (self._current_pose[0], self._current_pose[1])
        self_yaw = self._current_pose[2]

        front_obstacles: List[Tuple[str, float]] = []
        for other_name, other_pos in self._latest_fleet_positions.items():
            d = distance(self_position, other_pos)
            if in_forward_cone(
                self_position,
                self_yaw,
                other_pos,
                self._front_cone_half_angle_rad,
            ):
                front_obstacles.append((other_name, d))

        if not front_obstacles:
            return nominal_cmd

        for other_name, d in front_obstacles:
            if d <= (self._stop_distance + self._tie_distance_margin):
                if should_yield_by_tie_break(self._robot_name, other_name):
                    return self._make_stop_cmd()

        nearest_distance = min(d for _, d in front_obstacles)
        if nearest_distance <= self._stop_distance:
            return self._make_stop_cmd()

        if nearest_distance < self._slow_distance:
            denom = max(self._slow_distance - self._stop_distance, 1e-9)
            scale = (nearest_distance - self._stop_distance) / denom
            scale = clamp(scale, 0.0, 1.0)

            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = self._cmd_frame_id
            cmd.twist.linear.x = nominal_cmd.twist.linear.x * scale
            cmd.twist.angular.z = nominal_cmd.twist.angular.z
            return cmd

        return nominal_cmd

    def _make_stop_cmd(self) -> TwistStamped:
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self._cmd_frame_id
        return cmd

    def _publish_stop(self) -> None:
        self._cmd_pub.publish(self._make_stop_cmd())


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RuleBasedController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
