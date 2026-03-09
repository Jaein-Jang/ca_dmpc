#!/usr/bin/env python3
"""Launch multi-robot MPC stack that consumes Nav2 global paths on /{robot}/plan."""

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
    controller_type = LaunchConfiguration('controller_type').perform(context).strip().lower()
    use_ca_controller = controller_type in ('ca', 'ca_dmpc')
    if controller_type not in ('standard', 'ca', 'ca_dmpc'):
        raise RuntimeError('controller_type must be one of: standard, ca, ca_dmpc')

    nodes = [
        Node(
            package='ca_dmpc_experiments',
            executable='robot_state_collector',
            name='robot_state_collector',
            output='screen',
            parameters=[
                {
                    'robot_names_csv': LaunchConfiguration('robot_names'),
                    'publish_topic': LaunchConfiguration('fleet_state_topic'),
                    'publish_rate': LaunchConfiguration('fleet_state_publish_rate'),
                }
            ],
        )
    ]

    for robot_name in robot_names:
        nodes.append(
            Node(
                package='ca_dmpc_experiments',
                executable='path_frame_adapter',
                name=f'{robot_name}_path_frame_adapter',
                output='screen',
                parameters=[
                    {
                        'robot_name': robot_name,
                        'input_path_topic': f'/{robot_name}/plan',
                        'output_path_topic': f'/{robot_name}/plan_mpc',
                        'target_frame': LaunchConfiguration('path_target_frame'),
                        'resample_spacing': LaunchConfiguration('path_resample_spacing'),
                    }
                ],
            )
        )

        nodes.append(
            Node(
                package='ca_dmpc_experiments',
                executable='trajectory_broadcaster',
                name=f'{robot_name}_trajectory_broadcaster',
                output='screen',
                parameters=[
                    {
                        'robot_name': robot_name,
                        'path_topic': f'/{robot_name}/plan_mpc',
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

        if use_ca_controller:
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

        controller_parameters = {
            'robot_name': robot_name,
            'path_topic': f'/{robot_name}/plan_mpc',
            'odom_topic': f'/{robot_name}/odom',
            'neighbor_topic': f'/{robot_name}/neighbor_trajectories',
            'cmd_vel_topic': f'/{robot_name}/cmd_vel',
            'horizon_steps': LaunchConfiguration('horizon_steps'),
            'dt': LaunchConfiguration('dt'),
        }
        if use_ca_controller:
            controller_parameters['occupancy_topic'] = f'/{robot_name}/occupancy_debug'
            controller_parameters['wc'] = LaunchConfiguration('wc')

        controller_executable = 'ca_dmpc_controller' if use_ca_controller else 'standard_mpc_controller'
        controller_name_suffix = 'ca_dmpc_controller' if use_ca_controller else 'standard_mpc_controller'
        nodes.append(
            Node(
                package='ca_dmpc_experiments',
                executable=controller_executable,
                name=f'{robot_name}_{controller_name_suffix}',
                output='screen',
                parameters=[controller_parameters],
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
                'controller_type',
                default_value='standard',
                description='Controller mode: standard or ca',
            ),
            DeclareLaunchArgument(
                'fleet_state_topic',
                default_value='/fleet_states',
                description='Topic for aggregated fleet state JSON',
            ),
            DeclareLaunchArgument(
                'fleet_state_publish_rate',
                default_value='5.0',
                description='Fleet-state publish rate in Hz',
            ),
            DeclareLaunchArgument(
                'communication_radius',
                default_value='3.0',
                description='Neighbor communication radius in meters',
            ),
            DeclareLaunchArgument(
                'path_target_frame',
                default_value='odom',
                description='Target frame for adapted local MPC path',
            ),
            DeclareLaunchArgument(
                'path_resample_spacing',
                default_value='0.0',
                description='Optional path resampling spacing in meters (0 disables resampling)',
            ),
            DeclareLaunchArgument(
                'horizon_steps',
                default_value='15',
                description='Prediction/control horizon steps',
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
                description='Occupancy grid cell size in meters (CA mode)',
            ),
            DeclareLaunchArgument(
                'smoothing_enabled',
                default_value='true',
                description='Enable sparse occupancy smoothing kernel (CA mode)',
            ),
            DeclareLaunchArgument(
                'wc',
                default_value='1.0',
                description='Congestion cost weight (CA mode)',
            ),
            OpaqueFunction(function=create_nodes),
        ]
    )
