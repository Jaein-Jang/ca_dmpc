#!/usr/bin/env python3
"""Adapt Nav2 global path frame for local MPC consumers."""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


Point3D = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]


def normalize_topic(topic: str) -> str:
    """Ensure topic has leading slash."""
    return topic if topic.startswith('/') else f'/{topic}'


def quaternion_conjugate(q: Quat) -> Quat:
    """Return quaternion conjugate."""
    return (-q[0], -q[1], -q[2], q[3])


def quaternion_multiply(a: Quat, b: Quat) -> Quat:
    """Hamilton product of two quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def normalize_quaternion(q: Quat) -> Quat:
    """Return normalized quaternion."""
    norm = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if norm <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm)


def rotate_vector_by_quaternion(v: Point3D, q: Quat) -> Point3D:
    """Rotate 3D vector v by quaternion q."""
    qn = normalize_quaternion(q)
    vq: Quat = (v[0], v[1], v[2], 0.0)
    rotated = quaternion_multiply(quaternion_multiply(qn, vq), quaternion_conjugate(qn))
    return (rotated[0], rotated[1], rotated[2])


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert yaw to quaternion message."""
    half = 0.5 * yaw
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


class PathFrameAdapter(Node):
    """Transform incoming Path to a target frame and republish."""

    def __init__(self) -> None:
        super().__init__('path_frame_adapter')

        self.declare_parameter('robot_name', '')
        self.declare_parameter('input_path_topic', '/plan')
        self.declare_parameter('output_path_topic', '/plan_mpc')
        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('resample_spacing', 0.0)
        self.declare_parameter('lookup_timeout_sec', 0.1)

        self._robot_name = str(self.get_parameter('robot_name').value)
        self._input_path_topic = normalize_topic(str(self.get_parameter('input_path_topic').value))
        self._output_path_topic = normalize_topic(str(self.get_parameter('output_path_topic').value))
        self._target_frame = str(self.get_parameter('target_frame').value).strip() or 'odom'
        self._resample_spacing = max(0.0, float(self.get_parameter('resample_spacing').value))
        self._lookup_timeout_sec = max(1e-3, float(self.get_parameter('lookup_timeout_sec').value))
        self._lookup_timeout = Duration(seconds=self._lookup_timeout_sec)

        self._last_warn_ns: Optional[int] = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._path_sub = self.create_subscription(
            Path,
            self._input_path_topic,
            self._path_callback,
            10,
        )
        self._path_pub = self.create_publisher(Path, self._output_path_topic, 10)

    def _warn_throttled(self, message: str, min_interval_sec: float = 1.0) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self._last_warn_ns is None:
            self._last_warn_ns = now_ns
            self.get_logger().warn(message)
            return

        if (now_ns - self._last_warn_ns) * 1e-9 >= min_interval_sec:
            self._last_warn_ns = now_ns
            self.get_logger().warn(message)

    def _path_source_frame(self, msg: Path) -> Optional[str]:
        source = msg.header.frame_id.strip()
        if source:
            return source
        if msg.poses:
            source = msg.poses[0].header.frame_id.strip()
            if source:
                return source
        return None

    @staticmethod
    def _stamp_to_time(path_msg: Path) -> Time:
        stamp = path_msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return Time()
        return Time.from_msg(stamp)

    def _lookup_transform(self, target_frame: str, source_frame: str, path_msg: Path) -> TransformStamped:
        return self._tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            self._stamp_to_time(path_msg),
            timeout=self._lookup_timeout,
        )

    @staticmethod
    def _transform_pose(
        pose_msg: PoseStamped,
        transform: TransformStamped,
        target_frame: str,
        target_stamp,
    ) -> PoseStamped:
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z

        q_tf: Quat = (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        )
        q_pose: Quat = (
            pose_msg.pose.orientation.x,
            pose_msg.pose.orientation.y,
            pose_msg.pose.orientation.z,
            pose_msg.pose.orientation.w,
        )
        q_out = normalize_quaternion(quaternion_multiply(q_tf, q_pose))

        p_in: Point3D = (
            pose_msg.pose.position.x,
            pose_msg.pose.position.y,
            pose_msg.pose.position.z,
        )
        p_rot = rotate_vector_by_quaternion(p_in, q_tf)

        out = PoseStamped()
        out.header.stamp = target_stamp
        out.header.frame_id = target_frame
        out.pose.position.x = tx + p_rot[0]
        out.pose.position.y = ty + p_rot[1]
        out.pose.position.z = tz + p_rot[2]
        out.pose.orientation.x = q_out[0]
        out.pose.orientation.y = q_out[1]
        out.pose.orientation.z = q_out[2]
        out.pose.orientation.w = q_out[3]
        return out

    @staticmethod
    def _distance_2d(a: PoseStamped, b: PoseStamped) -> float:
        dx = b.pose.position.x - a.pose.position.x
        dy = b.pose.position.y - a.pose.position.y
        return math.hypot(dx, dy)

    @staticmethod
    def _resample_path_2d(
        poses: Sequence[PoseStamped],
        spacing: float,
        target_frame: str,
        target_stamp,
    ) -> List[PoseStamped]:
        if spacing <= 0.0 or len(poses) < 2:
            return list(poses)

        cumulative: List[float] = [0.0]
        for idx in range(1, len(poses)):
            cumulative.append(cumulative[-1] + PathFrameAdapter._distance_2d(poses[idx - 1], poses[idx]))

        total_length = cumulative[-1]
        if total_length <= 1e-9:
            return list(poses)

        targets: List[float] = []
        d = 0.0
        while d < total_length:
            targets.append(d)
            d += spacing
        targets.append(total_length)

        resampled: List[PoseStamped] = []
        seg = 1
        for td in targets:
            while seg < len(cumulative) and cumulative[seg] < td:
                seg += 1

            if seg >= len(cumulative):
                p = poses[-1]
                out = PoseStamped()
                out.header.stamp = target_stamp
                out.header.frame_id = target_frame
                out.pose = p.pose
                resampled.append(out)
                continue

            p0 = poses[seg - 1]
            p1 = poses[seg]
            d0 = cumulative[seg - 1]
            d1 = cumulative[seg]
            ratio = 0.0 if d1 <= d0 else (td - d0) / (d1 - d0)
            ratio = max(0.0, min(1.0, ratio))

            x = p0.pose.position.x + ratio * (p1.pose.position.x - p0.pose.position.x)
            y = p0.pose.position.y + ratio * (p1.pose.position.y - p0.pose.position.y)
            z = p0.pose.position.z + ratio * (p1.pose.position.z - p0.pose.position.z)
            yaw = math.atan2(
                p1.pose.position.y - p0.pose.position.y,
                p1.pose.position.x - p0.pose.position.x,
            )

            out = PoseStamped()
            out.header.stamp = target_stamp
            out.header.frame_id = target_frame
            out.pose.position.x = x
            out.pose.position.y = y
            out.pose.position.z = z
            out.pose.orientation = yaw_to_quaternion(yaw)
            resampled.append(out)

        return resampled

    def _path_callback(self, msg: Path) -> None:
        source_frame = self._path_source_frame(msg)
        if source_frame is None:
            self._warn_throttled('Path frame adapter: incoming path has no frame_id; skipping.')
            return

        target_stamp = msg.header.stamp
        transformed_poses: List[PoseStamped]
        if source_frame == self._target_frame:
            transformed_poses = []
            for pose in msg.poses:
                out = PoseStamped()
                out.header.stamp = target_stamp
                out.header.frame_id = self._target_frame
                out.pose = pose.pose
                transformed_poses.append(out)
        else:
            try:
                transform = self._lookup_transform(self._target_frame, source_frame, msg)
            except TransformException as exc:
                robot = f'[{self._robot_name}] ' if self._robot_name else ''
                self._warn_throttled(
                    f'{robot}Path frame adapter TF unavailable ({source_frame}->{self._target_frame}): {exc}'
                )
                return

            transformed_poses = [
                self._transform_pose(pose, transform, self._target_frame, target_stamp)
                for pose in msg.poses
            ]

        output_poses = self._resample_path_2d(
            transformed_poses,
            self._resample_spacing,
            self._target_frame,
            target_stamp,
        )

        out_msg = Path()
        out_msg.header.stamp = target_stamp
        out_msg.header.frame_id = self._target_frame
        out_msg.poses = output_poses
        self._path_pub.publish(out_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PathFrameAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
