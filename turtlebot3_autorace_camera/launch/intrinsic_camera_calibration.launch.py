#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Hyungyu Kim

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    composable_nodes = [
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_node',
            namespace='camera',
            parameters=[{'queue_size': 20}]
        ),

        ComposableNode(
            package='image_proc',
            plugin='image_proc::DebayerNode',
            name='debayer_node',
            namespace='camera',
            remappings=[
                ('image_raw', 'image_rect'),
                ('image_color/compressed', 'image_rect_color/compressed')
            ]
        )
    ]

    republish_node = Node(
        package='image_transport',
        executable='republish',
        name='republish',
        output='screen',
        arguments=[
            'compressed',
            'raw'],
        remappings=[
            ('in/compressed',
                '/camera/image_raw/compressed'),
            ('out',
                '/camera/image')]
    )

    image_proc_container = ComposableNodeContainer(
        name='image_proc_container',
        namespace='camera',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
    )

    return LaunchDescription([
        republish_node,
        image_proc_container
    ])
