#!/usr/bin/env python3
"""Launch full CA-DMPC pipeline (predictor, neighbor buffer, occupancy, controller) per robot."""

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
                executable='trajectory_broadcaster',
                name=f'{robot_name}_trajectory_broadcaster',
                output='screen',
                parameters=[
                    {
                        'robot_name': robot_name,
                        'path_topic': f'/{robot_name}/plan',
                        'odom_topic': f'/{robot_name}/odom',
                        'publish_topic': f'/{robot_name}/predicted_trajectory',
                        'horizon_steps': LaunchConfiguration('horizon_steps'),
                        'dt': LaunchConfiguration('dt'),
                        'nominal_speed': LaunchConfiguration('nominal_speed'),
                    }
                ],
            )
        )

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

        nodes.append(
            Node(
                package='ca_dmpc_experiments',
                executable='ca_dmpc_controller',
                name=f'{robot_name}_ca_dmpc_controller',
                output='screen',
                parameters=[
                    {
                        'robot_name': robot_name,
                        'path_topic': f'/{robot_name}/plan',
                        'odom_topic': f'/{robot_name}/odom',
                        'neighbor_topic': f'/{robot_name}/neighbor_trajectories',
                        'occupancy_topic': f'/{robot_name}/occupancy_debug',
                        'cmd_vel_topic': f'/{robot_name}/cmd_vel',
                        'horizon_steps': LaunchConfiguration('horizon_steps'),
                        'dt': LaunchConfiguration('dt'),
                        'wc': LaunchConfiguration('wc'),
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
            DeclareLaunchArgument(
                'fleet_state_topic',
                default_value='/fleet_states',
                description='Topic for aggregated fleet state JSON',
            ),
            DeclareLaunchArgument(
                'communication_radius',
                default_value='3.0',
                description='Neighbor communication radius in meters',
            ),
            DeclareLaunchArgument(
                'horizon_steps',
                default_value='15',
                description='Prediction/control horizon steps shared across the CA-DMPC pipeline',
            ),
            DeclareLaunchArgument(
                'dt',
                default_value='0.2',
                description='Prediction/control time step in seconds',
            ),
            DeclareLaunchArgument(
                'nominal_speed',
                default_value='0.2',
                description='Nominal speed used by trajectory broadcasters',
            ),
            DeclareLaunchArgument(
                'grid_resolution',
                default_value='0.1',
                description='Occupancy grid cell size in meters',
            ),
            DeclareLaunchArgument(
                'smoothing_enabled',
                default_value='true',
                description='Enable sparse occupancy smoothing kernel',
            ),
            DeclareLaunchArgument(
                'wc',
                default_value='1.0',
                description='Congestion cost weight used by ca_dmpc_controller',
            ),
            OpaqueFunction(function=create_nodes),
        ]
    )
