#!/usr/bin/env python3
"""Launch one occupancy grid builder per robot."""

from __future__ import annotations

from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def parse_robot_names(raw_names: str) -> List[str]:
    """Parse comma-separated robot names."""
    return [name.strip() for name in raw_names.split(',') if name.strip()]


def create_nodes(context, *args, **kwargs):
    robot_names = parse_robot_names(LaunchConfiguration('robot_names').perform(context))

    nodes = []
    for robot_name in robot_names:
        nodes.append(
            Node(
                package='ca_dmpc_experiments',
                executable='occupancy_grid_builder',
                name=f'{robot_name}_occupancy_grid_builder',
                output='screen',
                parameters=[
                    {
                        'robot_name': robot_name,
                        'neighbor_topic': f'/{robot_name}/neighbor_trajectories',
                        'publish_topic': f'/{robot_name}/occupancy_debug',
                        'grid_resolution': LaunchConfiguration('grid_resolution'),
                        'horizon_steps': LaunchConfiguration('horizon_steps'),
                        'smoothing_enabled': LaunchConfiguration('smoothing_enabled'),
                    }
                ],
            )
        )
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument('robot_names', default_value='tb1,tb2,tb3'),
            DeclareLaunchArgument('grid_resolution', default_value='0.1'),
            DeclareLaunchArgument('horizon_steps', default_value='15'),
            DeclareLaunchArgument('smoothing_enabled', default_value='true'),
            OpaqueFunction(function=create_nodes),
        ]
    )
