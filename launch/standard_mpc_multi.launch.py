#!/usr/bin/env python3
"""Launch one Standard MPC controller per robot."""

from __future__ import annotations

from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parse_robot_names(raw_names: str) -> List[str]:
    return [name.strip() for name in raw_names.split(',') if name.strip()]


def _create_nodes(context, *args, **kwargs):
    robot_names = _parse_robot_names(LaunchConfiguration('robot_names').perform(context))

    nodes = []
    for robot_name in robot_names:
        nodes.append(
            Node(
                package='ca_dmpc_experiments',
                executable='standard_mpc_controller',
                name=f'{robot_name}_standard_mpc_controller',
                output='screen',
                parameters=[
                    {
                        'robot_name': robot_name,
                        'path_topic': f'/{robot_name}/plan',
                        'odom_topic': f'/{robot_name}/odom',
                        'neighbor_topic': f'/{robot_name}/neighbor_trajectories',
                        'cmd_vel_topic': f'/{robot_name}/cmd_vel',
                    }
                ],
            )
        )

    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot_names',
                default_value='tb1,tb3',
                description='Comma-separated robot names (e.g. tb1,tb3)',
            ),
            OpaqueFunction(function=_create_nodes),
        ]
    )
