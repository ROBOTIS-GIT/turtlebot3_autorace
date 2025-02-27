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
# Author: ChanHyeong Lee

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    calibration_mode_arg = DeclareLaunchArgument(
        'calibration_mode',
        default_value='False',
        description='calibration mode type [Ture, False]'
        )
    calibration_mode = LaunchConfiguration('calibration_mode')

    # path for parameter file
    pkg_share = get_package_share_directory('turtlebot3_autorace_detect')
    param_file = os.path.join(pkg_share, 'param', 'level', 'level.yaml')

    detect_level_crossing_node = Node(
        package='turtlebot3_autorace_detect',
        executable='detect_level_crossing',
        name='detect_level_crossing',
        output='screen',
        parameters=[
            param_file,
            {'is_detection_calibration_mode': calibration_mode}
        ],
        remappings=[
            ('/detect/image_input', '/camera/image_compensated'),
            ('/detect/image_input/compressed', '/camera/image_compensated/compressed'),
            ('/detect/image_output', '/detect/image_level'),
            ('/detect/image_output/compressed', '/detect/image_level/compressed'),
            ('/detect/image_output_sub1', '/detect/image_level_color_filtered'),
            ('/detect/image_output_sub1/compressed',
                '/detect/image_level_color_filtered/compressed'),
        ]
    )

    ld = LaunchDescription()
    ld.add_action(calibration_mode_arg)
    ld.add_action(detect_level_crossing_node)
    return ld
