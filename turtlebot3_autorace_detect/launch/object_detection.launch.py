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
# Author: YeonSoo Noh

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/ubuntu/best.pt',
        description='Path to the YOLO model file'
    )
    test_mode_arg = DeclareLaunchArgument(
        'test_mode',
        default_value='false',
        description='Run in test mode without task manager'
    )
    model_path = LaunchConfiguration('model_path')
    test_mode = LaunchConfiguration('test_mode')

    object_detection_node = Node(
        package='turtlebot3_autorace_detect',
        executable='object_detection',
        name='object_detection_node',
        output='screen',
        parameters=[{'model_path': model_path, 'test_mode': test_mode}]
    )

    return LaunchDescription([
        model_path_arg,
        test_mode_arg,
        object_detection_node,
    ])
