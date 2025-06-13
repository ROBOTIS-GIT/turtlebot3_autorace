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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='turtlebot3_autorace_mission',
            executable='undocking_node',
            parameters=[
                {'use_sim_time': use_sim_time}],
            output='screen'),

        Node(
            package='turtlebot3_autorace_mission',
            executable='alley_mission_node',
            parameters=[
                {'use_sim_time': use_sim_time}],
            output='screen'),

        Node(
            package='turtlebot3_autorace_detect',
            executable='aruco_tracker',
            parameters=[
                {'use_sim_time': use_sim_time}],
            output='screen'),

        Node(
            package='turtlebot3_autorace_mission',
            executable='aruco_parking.py',
            parameters=[
                {'use_sim_time': use_sim_time}],
            output='screen'),

        Node(
            package='turtlebot3_autorace_mission',
            executable='task_manager_node',
            parameters=[
                {'use_sim_time': use_sim_time}],
            output='screen'),
    ])
