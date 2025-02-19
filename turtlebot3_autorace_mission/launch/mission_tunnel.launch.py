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
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get the share directory of turtlebot3_navigation2 package
    nav2_pkg_share = get_package_share_directory('turtlebot3_navigation2')

    # Map file path
    map_file = os.path.join(
            get_package_share_directory('turtlebot3_autorace_mission'),
            'map',
            'map.yaml')

    # Include the navigation2 launch file with required arguments
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg_share, 'launch', 'navigation2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'map': map_file
        }.items()
    )

    # Execute the set_init_pose.py node via ros2 run command
    mission_tunnel = ExecuteProcess(
        cmd=['ros2', 'run', 'turtlebot3_autorace_mission', 'mission_tunnel.py'],
        output='screen'
    )

    return LaunchDescription([
        nav2_launch,
        mission_tunnel,
    ])
