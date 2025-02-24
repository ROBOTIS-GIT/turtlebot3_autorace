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

import math

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node


class MissionTunnel(Node):

    def __init__(self):
        super().__init__('mission_tunnel')

        self.declare_parameter(
            'init_pose.position.x', 0.0)
        self.declare_parameter(
            'init_pose.position.y', 0.0)
        self.declare_parameter(
            'init_pose.position.z', 0.0)
        self.declare_parameter(
            'init_pose.orientation.x', 0.0)
        self.declare_parameter(
            'init_pose.orientation.y', 0.0)
        self.declare_parameter(
            'init_pose.orientation.yaw', 0)
        self.declare_parameter(
            'goal_pose.position.x', 0.0)
        self.declare_parameter(
            'goal_pose.position.y', 0.0)
        self.declare_parameter(
            'goal_pose.position.z', 0.0)
        self.declare_parameter(
            'goal_pose.orientation.x', 0.0)
        self.declare_parameter(
            'goal_pose.orientation.y', 0.0)
        self.declare_parameter(
            'goal_pose.orientation.yaw', 0)

        self.init_position_x = self.get_parameter(
            'init_pose.position.x').value
        self.init_position_y = self.get_parameter(
            'init_pose.position.y').value
        self.init_position_z = self.get_parameter(
            'init_pose.position.z').value
        self.init_orientation_x = self.get_parameter(
            'init_pose.orientation.x').value
        self.init_orientation_y = self.get_parameter(
            'init_pose.orientation.y').value
        self.init_orientation_yaw = self.get_parameter(
            'init_pose.orientation.yaw').value
        self.goal_position_x = self.get_parameter(
            'goal_pose.position.x').value
        self.goal_position_y = self.get_parameter(
            'goal_pose.position.y').value
        self.goal_position_z = self.get_parameter(
            'goal_pose.position.z').value
        self.goal_orientation_x = self.get_parameter(
            'goal_pose.orientation.x').value
        self.goal_orientation_y = self.get_parameter(
            'goal_pose.orientation.y').value
        self.goal_orientation_yaw = self.get_parameter(
            'goal_pose.orientation.yaw').value

        self.init_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        self.goal_pub = self.create_publisher(
            PoseStamped, '/goal_pose', 10)

        self.start_timer = self.create_timer(1.0, self.start_initial_phase)
        self.started = False

        self.init_timer = None
        self.phase_timer = None
        self.goal_timer = None
        self.shutdown_timer = None

    def start_initial_phase(self):
        if self.started:
            return
        self.started = True

        self.start_timer.cancel()

        self.init_timer = self.create_timer(0.1, self.publish_initial_pose)
        self.phase_timer = self.create_timer(1.0, self.start_goal_phase)

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.pose.position.x = self.init_position_x
        msg.pose.pose.position.y = self.init_position_y
        msg.pose.pose.position.z = self.init_position_z

        yaw = math.radians(self.init_orientation_yaw)
        msg.pose.pose.orientation.x = self.init_orientation_x
        msg.pose.pose.orientation.y = self.init_orientation_y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        msg.pose.covariance = [0.0] * 36

        self.init_pose_pub.publish(msg)

    def start_goal_phase(self):
        if self.init_timer is not None:
            self.init_timer.cancel()
        if self.phase_timer is not None:
            self.phase_timer.cancel()

        self.goal_timer = self.create_timer(0.1, self.publish_goal)
        self.shutdown_timer = self.create_timer(2.0, self.shutdown_node)

    def publish_goal(self):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'

        # Set goal pose
        goal_msg.pose.position.x = self.goal_position_x
        goal_msg.pose.position.y = self.goal_position_y
        goal_msg.pose.position.z = self.goal_position_z

        yaw = math.radians(self.goal_orientation_yaw)
        goal_msg.pose.orientation.x = self.goal_orientation_x
        goal_msg.pose.orientation.y = self.goal_orientation_y
        goal_msg.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal_pub.publish(goal_msg)

    def shutdown_node(self):
        if self.goal_timer is not None:
            self.goal_timer.cancel()
        if self.shutdown_timer is not None:
            self.shutdown_timer.cancel()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MissionTunnel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
