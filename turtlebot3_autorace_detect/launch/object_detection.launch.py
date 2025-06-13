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
        default_value='~/Downloads/best.pt',
        description='Path to the YOLO model file'
    )
    model_path = LaunchConfiguration('model_path')

    object_detection_node = Node(
        package='turtlebot3_yolo_object_detection',
        executable='turtlebot3_yolo_object_detection',
        name='turtlebot3_object_detection_node',
        output='screen',
        parameters=[{'model_path': model_path}]
    )

    return LaunchDescription([
        model_path_arg,
        object_detection_node,
    ])
