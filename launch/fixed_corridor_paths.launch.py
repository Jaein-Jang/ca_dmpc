#!/usr/bin/env python3
"""Launch fixed path publishers for corridor_bottleneck_v1 benchmark."""

from __future__ import annotations

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    tb1_path_json = '[{"x":0.0,"y":0.0},{"x":0.5,"y":0.0},{"x":1.0,"y":0.0},{"x":1.5,"y":0.0},{"x":2.0,"y":0.0},{"x":2.5,"y":0.0},{"x":3.0,"y":0.0}]'
    tb3_path_json = '[{"x":0.0,"y":0.0},{"x":-0.5,"y":0.0},{"x":-1.0,"y":0.0},{"x":-1.5,"y":0.0},{"x":-2.0,"y":0.0},{"x":-2.5,"y":0.0},{"x":-3.0,"y":0.0}]'

    tb1_node = Node(
        package='ca_dmpc_experiments',
        executable='fixed_path_publisher',
        name='tb1_fixed_path_publisher',
        output='screen',
        parameters=[
            {
                'robot_name': 'tb1',
                'publish_topic': '/tb1/plan',
                'frame_id': 'odom',
                'publish_rate': 1.0,
                'path_points': ParameterValue(tb1_path_json, value_type=str),
            }
        ],
    )

    tb3_node = Node(
        package='ca_dmpc_experiments',
        executable='fixed_path_publisher',
        name='tb3_fixed_path_publisher',
        output='screen',
        parameters=[
            {
                'robot_name': 'tb3',
                'publish_topic': '/tb3/plan',
                'frame_id': 'odom',
                'publish_rate': 1.0,
                'path_points': ParameterValue(tb3_path_json, value_type=str),
            }
        ],
    )

    return LaunchDescription([tb1_node, tb3_node])
