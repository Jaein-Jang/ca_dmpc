#!/usr/bin/env python3
"""Standard MPC baseline controller without congestion / occupancy cost."""

from __future__ import annotations

import json
import math
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from scipy.optimize import minimize
from std_msgs.msg import String


Point2D = Tuple[float, float]
State = Tuple[float, float, float]
Control = Tuple[float, float]
Reference = Tuple[float, float, float]
NeighborTrajectory = Dict[int, Point2D]


def clamp(value: float, minimum: float, maximum: float) -> float:
    """Clamp value into [minimum, maximum]."""
    return max(minimum, min(value, maximum))


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def distance(a: Point2D, b: Point2D) -> float:
    """Euclidean distance between two 2D points."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def normalize_topic(topic: str) -> str:
    """Ensure topic has leading slash."""
    return topic if topic.startswith('/') else f'/{topic}'


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Convert quaternion orientation to yaw."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class StandardMPCController(Node):
    """Local Standard MPC controller that tracks path and soft-penalizes unsafe proximity."""

    def __init__(self) -> None:
        super().__init__('standard_mpc_controller')

        self.declare_parameter('robot_name', '')
        self.declare_parameter('path_topic', '/plan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('neighbor_topic', '/neighbor_trajectories')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('horizon_steps', 15)
        self.declare_parameter('dt', 0.2)
        self.declare_parameter('max_linear_speed', 0.22)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('control_rate', 5.0)
        self.declare_parameter('safety_distance', 0.6)
        self.declare_parameter('slack_penalty', 300.0)
        self.declare_parameter('q_pos', 10.0)
        self.declare_parameter('q_yaw', 1.0)
        self.declare_parameter('r_v', 0.5)
        self.declare_parameter('r_w', 0.2)
        self.declare_parameter('rd_v', 0.2)
        self.declare_parameter('rd_w', 0.1)
        self.declare_parameter('solver_maxiter', 50)

        self._robot_name = str(self.get_parameter('robot_name').value)
        self._path_topic = normalize_topic(str(self.get_parameter('path_topic').value))
        self._odom_topic = normalize_topic(str(self.get_parameter('odom_topic').value))
        self._neighbor_topic = normalize_topic(str(self.get_parameter('neighbor_topic').value))
        self._cmd_vel_topic = normalize_topic(str(self.get_parameter('cmd_vel_topic').value))

        self._horizon_steps = max(1, int(self.get_parameter('horizon_steps').value))
        self._dt = max(1e-3, float(self.get_parameter('dt').value))
        self._max_linear_speed = max(0.0, float(self.get_parameter('max_linear_speed').value))
        self._max_angular_speed = max(0.0, float(self.get_parameter('max_angular_speed').value))
        self._goal_tolerance = max(0.01, float(self.get_parameter('goal_tolerance').value))
        control_rate = max(1e-3, float(self.get_parameter('control_rate').value))

        self._safety_distance = max(0.0, float(self.get_parameter('safety_distance').value))
        self._slack_penalty = max(0.0, float(self.get_parameter('slack_penalty').value))
        self._q_pos = max(0.0, float(self.get_parameter('q_pos').value))
        self._q_yaw = max(0.0, float(self.get_parameter('q_yaw').value))
        self._r_v = max(0.0, float(self.get_parameter('r_v').value))
        self._r_w = max(0.0, float(self.get_parameter('r_w').value))
        self._rd_v = max(0.0, float(self.get_parameter('rd_v').value))
        self._rd_w = max(0.0, float(self.get_parameter('rd_w').value))
        self._solver_maxiter = max(1, int(self.get_parameter('solver_maxiter').value))

        self._control_period = 1.0 / control_rate
        self._cmd_frame_id = f'{self._robot_name}/base_link' if self._robot_name else 'base_link'

        self._path_points: List[Point2D] = []
        self._current_state: Optional[State] = None
        self._neighbor_trajectories: List[NeighborTrajectory] = []

        self._u_warm_start: Optional[List[float]] = None
        self._prev_command: Optional[Control] = None

        self._path_sub = self.create_subscription(Path, self._path_topic, self._path_callback, 10)
        self._odom_sub = self.create_subscription(Odometry, self._odom_topic, self._odom_callback, 10)
        self._neighbor_sub = self.create_subscription(
            String,
            self._neighbor_topic,
            self._neighbor_callback,
            10,
        )
        self._cmd_pub = self.create_publisher(TwistStamped, self._cmd_vel_topic, 10)
        self._timer = self.create_timer(self._control_period, self._control_loop)

    def _path_callback(self, msg: Path) -> None:
        self._path_points = [
            (pose_stamped.pose.position.x, pose_stamped.pose.position.y)
            for pose_stamped in msg.poses
        ]

    def _odom_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        yaw = quaternion_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        self._current_state = (pose.position.x, pose.position.y, yaw)

    def _neighbor_callback(self, msg: String) -> None:
        self._neighbor_trajectories = self._parse_neighbor_payload(msg.data)

    def _parse_neighbor_payload(self, raw_json: str) -> List[NeighborTrajectory]:
        try:
            payload = json.loads(raw_json)
        except json.JSONDecodeError:
            self.get_logger().warn('Failed to parse neighbor trajectory JSON; using empty neighbors.')
            return []

        neighbors = payload.get('neighbors', [])
        if not isinstance(neighbors, list):
            return []

        parsed_neighbors: List[NeighborTrajectory] = []
        for neighbor in neighbors:
            if not isinstance(neighbor, dict):
                continue
            trajectory = neighbor.get('trajectory', [])
            if not isinstance(trajectory, list):
                continue

            traj_points: NeighborTrajectory = {}
            for point in trajectory:
                if not isinstance(point, dict):
                    continue
                try:
                    t_index = int(point.get('t_index', -1))
                    x = float(point.get('x', 0.0))
                    y = float(point.get('y', 0.0))
                except (TypeError, ValueError):
                    continue
                if t_index < 0:
                    continue
                traj_points[t_index] = (x, y)

            if traj_points:
                parsed_neighbors.append(traj_points)

        return parsed_neighbors

    @staticmethod
    def _nearest_path_index(current_position: Point2D, path_points: Sequence[Point2D]) -> int:
        return min(
            range(len(path_points)),
            key=lambda idx: distance(current_position, path_points[idx]),
        )

    def _sample_reference_points(self, current_position: Point2D) -> List[Point2D]:
        nearest_idx = self._nearest_path_index(current_position, self._path_points)
        last_idx = len(self._path_points) - 1

        sampled: List[Point2D] = []
        for k in range(self._horizon_steps):
            idx = min(nearest_idx + k, last_idx)
            sampled.append(self._path_points[idx])

        return sampled

    @staticmethod
    def _estimate_reference_yaws(points: Sequence[Point2D]) -> List[float]:
        if not points:
            return []
        if len(points) == 1:
            return [0.0]

        yaws: List[float] = []
        for idx in range(len(points)):
            if idx < len(points) - 1:
                p0 = points[idx]
                p1 = points[idx + 1]
                yaws.append(math.atan2(p1[1] - p0[1], p1[0] - p0[0]))
            else:
                yaws.append(yaws[-1])
        return yaws

    def _build_reference(self, current_position: Point2D) -> List[Reference]:
        points = self._sample_reference_points(current_position)
        yaws = self._estimate_reference_yaws(points)
        return [(points[k][0], points[k][1], yaws[k]) for k in range(self._horizon_steps)]

    def _rollout(self, initial_state: State, controls_flat: Sequence[float]) -> List[Tuple[float, float, float, float, float]]:
        x, y, yaw = initial_state
        trajectory: List[Tuple[float, float, float, float, float]] = []

        for k in range(self._horizon_steps):
            v = float(controls_flat[2 * k])
            w = float(controls_flat[2 * k + 1])

            x = x + self._dt * v * math.cos(yaw)
            y = y + self._dt * v * math.sin(yaw)
            yaw = normalize_angle(yaw + self._dt * w)

            trajectory.append((x, y, yaw, v, w))

        return trajectory

    def _objective(self, controls_flat: Sequence[float], initial_state: State, reference: Sequence[Reference]) -> float:
        rollout = self._rollout(initial_state, controls_flat)
        cost = 0.0

        prev_v = self._prev_command[0] if self._prev_command is not None else 0.0
        prev_w = self._prev_command[1] if self._prev_command is not None else 0.0

        for k, (x, y, yaw, v, w) in enumerate(rollout):
            rx, ry, ryaw = reference[k]
            dx = x - rx
            dy = y - ry
            dyaw = normalize_angle(yaw - ryaw)

            # Tracking objective.
            cost += self._q_pos * (dx * dx + dy * dy)
            cost += self._q_yaw * (dyaw * dyaw)

            # Control effort objective.
            cost += self._r_v * (v * v) + self._r_w * (w * w)

            # Input smoothness objective.
            dv = v - prev_v
            dw = w - prev_w
            cost += self._rd_v * (dv * dv) + self._rd_w * (dw * dw)
            prev_v = v
            prev_w = w

            # Soft safety penalty from neighbor predicted positions.
            for neighbor_traj in self._neighbor_trajectories:
                neighbor_point = neighbor_traj.get(k)
                if neighbor_point is None:
                    continue
                dist = distance((x, y), neighbor_point)
                violation = max(0.0, self._safety_distance - dist)
                cost += self._slack_penalty * (violation * violation)

        return cost

    def _control_bounds(self) -> List[Tuple[float, float]]:
        bounds: List[Tuple[float, float]] = []
        for _ in range(self._horizon_steps):
            bounds.append((0.0, self._max_linear_speed))
            bounds.append((-self._max_angular_speed, self._max_angular_speed))
        return bounds

    def _initial_guess(self) -> List[float]:
        n = 2 * self._horizon_steps
        if self._u_warm_start is not None and len(self._u_warm_start) == n:
            return list(self._u_warm_start)
        return [0.0] * n

    def _update_warm_start(self, solution: Sequence[float]) -> None:
        n = 2 * self._horizon_steps
        if len(solution) != n:
            self._u_warm_start = None
            return

        shifted = list(solution[2:])
        shifted.extend([solution[-2], solution[-1]])
        self._u_warm_start = shifted

    def _solve_mpc(self, initial_state: State, reference: Sequence[Reference]) -> Optional[Control]:
        guess = self._initial_guess()
        bounds = self._control_bounds()

        result = minimize(
            self._objective,
            guess,
            args=(initial_state, reference),
            method='SLSQP',
            bounds=bounds,
            options={
                'maxiter': self._solver_maxiter,
                'disp': False,
            },
        )

        if not result.success:
            self.get_logger().warn(f'MPC solver failed: {result.message}')
            return None

        if len(result.x) < 2:
            return None

        self._update_warm_start(result.x)
        v0 = clamp(float(result.x[0]), 0.0, self._max_linear_speed)
        w0 = clamp(float(result.x[1]), -self._max_angular_speed, self._max_angular_speed)
        return (v0, w0)

    def _is_goal_reached(self, current_position: Point2D) -> bool:
        if not self._path_points:
            return False
        goal = self._path_points[-1]
        return distance(current_position, goal) <= self._goal_tolerance

    def _make_twist_msg(self, v: float, w: float) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._cmd_frame_id
        msg.twist.linear.x = v
        msg.twist.angular.z = w
        return msg

    def _publish_stop(self) -> None:
        self._cmd_pub.publish(self._make_twist_msg(0.0, 0.0))

    def _publish_control(self, control: Control) -> None:
        self._cmd_pub.publish(self._make_twist_msg(control[0], control[1]))

    def _control_loop(self) -> None:
        if self._current_state is None:
            self._publish_stop()
            return

        if not self._path_points:
            self._publish_stop()
            return

        current_position = (self._current_state[0], self._current_state[1])
        if self._is_goal_reached(current_position):
            self._publish_stop()
            return

        reference = self._build_reference(current_position)
        control = self._solve_mpc(self._current_state, reference)

        if control is None:
            if self._prev_command is not None:
                self._publish_control(self._prev_command)
            else:
                self._publish_stop()
            return

        self._prev_command = control
        self._publish_control(control)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = StandardMPCController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
