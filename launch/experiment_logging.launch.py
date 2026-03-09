#!/usr/bin/env python3
"""Launch logging infrastructure for multi-robot experiments."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def _create_nodes(context, *args, **kwargs):
    state_collector_node = Node(
        package='ca_dmpc_experiments',
        executable='robot_state_collector',
        name='robot_state_collector',
        output='screen',
        parameters=[
            {
                'robot_names_csv': LaunchConfiguration('robot_names'),
                'robot_offsets_json': ParameterValue(
                    LaunchConfiguration('robot_offsets_json'),
                    value_type=str,
                ),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'publish_topic': LaunchConfiguration('fleet_state_topic'),
            }
        ],
    )

    metric_logger_node = Node(
        package='ca_dmpc_experiments',
        executable='metric_logger',
        name='metric_logger',
        output='screen',
        parameters=[
            {
                'fleet_state_topic': LaunchConfiguration('fleet_state_topic'),
                'log_dir': LaunchConfiguration('log_dir'),
                'deadlock_speed_threshold': LaunchConfiguration('deadlock_speed_threshold'),
                'deadlock_duration_sec': LaunchConfiguration('deadlock_duration_sec'),
            }
        ],
    )

    return [state_collector_node, metric_logger_node]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot_names',
                default_value='tb1,tb2,tb3',
                description='Comma-separated robot names (e.g. tb1,tb2,tb3)',
            ),
            DeclareLaunchArgument(
                'log_dir',
                default_value='/tmp/ca_dmpc_logs',
                description='Directory where experiment CSV logs are written',
            ),
            DeclareLaunchArgument(
                'publish_rate',
                default_value='5.0',
                description='Fleet state publish rate in Hz for robot_state_collector',
            ),
            DeclareLaunchArgument(
                'robot_offsets_json',
                default_value='{}',
                description='JSON object mapping robot names to spawn offsets '
                            '(e.g. {"tb1":{"x":-1.5,"y":-0.5}})',
            ),
            DeclareLaunchArgument(
                'fleet_state_topic',
                default_value='/fleet_states',
                description='Topic used to publish/subscribe aggregated fleet state JSON',
            ),
            DeclareLaunchArgument(
                'deadlock_speed_threshold',
                default_value='0.05',
                description='Fleet average speed threshold (m/s) for deadlock detection',
            ),
            DeclareLaunchArgument(
                'deadlock_duration_sec',
                default_value='5.0',
                description='Continuous low-speed duration (sec) required to declare deadlock',
            ),
            OpaqueFunction(function=_create_nodes),
        ]
    )
