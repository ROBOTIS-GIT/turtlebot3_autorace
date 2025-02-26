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
# Author: Jun

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mission_arg = DeclareLaunchArgument(
        'mission',
        default_value='intersection',
        description='Mission type [intersection, construction, parking, level_crossing, tunnel]'
    )

    mission = LaunchConfiguration('mission')

    detect_sign_node = Node(
        package='turtlebot3_autorace_detect',
        executable=['detect_', mission, '_sign'],
        name=['detect_', mission, '_sign'],
        output='screen',
        remappings=[
            ('/detect/image_input', '/camera/image_compensated'),
            ('/detect/image_input/compressed', '/camera/image_compensated/compressed'),
            ('/detect/image_output', '/detect/image_traffic_sign'),
            ('/detect/image_output/compressed', '/detect/image_traffic_sign/compressed'),
        ]
    )

    return LaunchDescription([mission_arg, detect_sign_node])
