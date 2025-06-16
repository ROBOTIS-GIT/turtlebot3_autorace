#!/usr/bin/env python3
#
################################################################################
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
################################################################################

# Authors: ChanHyeong Lee

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix for joint names, useful for multi-robot setup.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="turtlebot3_arm.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        ),
    ]

    prefix = LaunchConfiguration("prefix")
    description_file = LaunchConfiguration("description_file")

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([FindPackageShare("turtlebot3_autorace_mission"), "config", description_file]),
            " ",
            "prefix:=",
            prefix,
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("turtlebot3_autorace_mission"), "config", "ros2_controllers.yaml"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description, robot_controllers],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[robot_description],
        output="both",
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
        parameters=[robot_description],
        output="both",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    return LaunchDescription(
        declared_arguments
        + [
            control_node,
            joint_state_broadcaster_spawner,
            joint_trajectory_controller_spawner,
            robot_state_publisher_node,
        ]
    )
