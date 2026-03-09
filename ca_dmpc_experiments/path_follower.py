#!/usr/bin/env python3
"""Simple baseline global path follower for differential-drive robots."""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


Point2D = Tuple[float, float]


def clamp(value: float, minimum: float, maximum: float) -> float:
    """Clamp a value to [minimum, maximum]."""
    return max(minimum, min(value, maximum))


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Convert quaternion to yaw."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def distance(a: Point2D, b: Point2D) -> float:
    """Euclidean distance between two 2D points."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


class PathFollower(Node):
    """Pure-pursuit-like path follower suitable as a baseline controller."""

    def __init__(self) -> None:
        super().__init__('path_follower')

        self.declare_parameter('robot_name', '')
        self.declare_parameter('path_topic', '/plan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('max_linear_speed', 0.22)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('control_rate', 10.0)

        self._robot_name = str(self.get_parameter('robot_name').value)
        self._cmd_frame_id = (
            f'{self._robot_name}/base_link' if self._robot_name else 'base_link'
        )
        self._path_topic = self._normalize_topic(str(self.get_parameter('path_topic').value))
        self._odom_topic = self._normalize_topic(str(self.get_parameter('odom_topic').value))
        self._cmd_vel_topic = self._normalize_topic(str(self.get_parameter('cmd_vel_topic').value))
        self._lookahead_distance = max(0.05, float(self.get_parameter('lookahead_distance').value))
        self._max_linear_speed = max(0.0, float(self.get_parameter('max_linear_speed').value))
        self._max_angular_speed = max(0.0, float(self.get_parameter('max_angular_speed').value))
        self._goal_tolerance = max(0.01, float(self.get_parameter('goal_tolerance').value))
        control_rate = max(1e-3, float(self.get_parameter('control_rate').value))
        self._control_period = 1.0 / control_rate

        self._path_points: List[Point2D] = []
        self._current_pose: Optional[Tuple[float, float, float]] = None  # x, y, yaw
        self._goal_reached = False

        self._path_sub = self.create_subscription(Path, self._path_topic, self._path_callback, 10)
        self._odom_sub = self.create_subscription(Odometry, self._odom_topic, self._odom_callback, 10)
        self._cmd_pub = self.create_publisher(TwistStamped, self._cmd_vel_topic, 10)

        self._control_timer = self.create_timer(self._control_period, self._control_loop)

        self.get_logger().info(
            f'PathFollower[{self._robot_name or "unnamed"}] '
            f'path={self._path_topic}, odom={self._odom_topic}, cmd_vel={self._cmd_vel_topic}'
        )

    @staticmethod
    def _normalize_topic(topic: str) -> str:
        return topic if topic.startswith('/') else f'/{topic}'

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

    def _control_loop(self) -> None:
        if self._current_pose is None or not self._path_points:
            self._publish_stop()
            return

        x, y, yaw = self._current_pose
        current_position = (x, y)
        goal = self._path_points[-1]

        if distance(current_position, goal) <= self._goal_tolerance:
            if not self._goal_reached:
                self.get_logger().info(
                    f'PathFollower[{self._robot_name or "unnamed"}] goal reached within tolerance '
                    f'({self._goal_tolerance:.2f}m).'
                )
            self._goal_reached = True
            self._publish_stop()
            return

        target = self._select_lookahead_target(current_position, self._path_points)
        cmd = self._compute_cmd(current_position, yaw, target)
        self._cmd_pub.publish(cmd)

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

    def _compute_cmd(self, current_position: Point2D, yaw: float, target: Point2D) -> TwistStamped:
        dx = target[0] - current_position[0]
        dy = target[1] - current_position[1]
        target_heading = math.atan2(dy, dx)
        alpha = normalize_angle(target_heading - yaw)

        # Slow down forward speed when heading error is large.
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

    def _publish_stop(self) -> None:
        stop_cmd = TwistStamped()
        stop_cmd.header.stamp = self.get_clock().now().to_msg()
        stop_cmd.header.frame_id = self._cmd_frame_id
        self._cmd_pub.publish(stop_cmd)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
