#!/usr/bin/env python3
"""Launch one neighbor trajectory buffer per robot."""

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
                executable='neighbor_trajectory_buffer',
                name=f'{robot_name}_neighbor_trajectory_buffer',
                output='screen',
                parameters=[
                    {
                        'robot_name': robot_name,
                        'robot_names_csv': LaunchConfiguration('robot_names'),
                        'fleet_state_topic': LaunchConfiguration('fleet_state_topic'),
                        'communication_radius': LaunchConfiguration('communication_radius'),
                        'publish_topic': f'/{robot_name}/neighbor_trajectories',
                    }
                ],
            )
        )
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument('robot_names', default_value='tb1,tb2,tb3'),
            DeclareLaunchArgument('fleet_state_topic', default_value='/fleet_states'),
            DeclareLaunchArgument('communication_radius', default_value='3.0'),
            OpaqueFunction(function=create_nodes),
        ]
    )
