#!/usr/bin/env python3

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

# Author: ChanHyeong Lee

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class Nav2GoalPublisher(Node):
    def __init__(self):
        super().__init__('nav2_goal_publisher')
        # Create publisher for initial pose (/initialpose topic, PoseWithCovarianceStamped type)
        self.init_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # Create publisher for goal pose (/goal_pose topic, PoseStamped type)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Timer to delay the start of publishing by 5 seconds
        self.start_timer = self.create_timer(1.0, self.start_initial_phase)
        # Ensure the timer runs only once by cancelling it in the callback
        self.started = False

        # Goal publishing timer and shutdown timer are not created yet
        self.init_timer = None
        self.phase_timer = None
        self.goal_timer = None
        self.shutdown_timer = None

    def start_initial_phase(self):
        # Make sure this callback runs only once
        if self.started:
            return
        self.started = True

        # Cancel the start timer
        self.start_timer.cancel()

        # Create a timer to publish initial pose every 0.1 seconds (runs for 1 second)
        self.init_timer = self.create_timer(0.1, self.publish_initial_pose)
        # Create a timer to switch to goal publishing phase after 1 second
        self.phase_timer = self.create_timer(1.0, self.start_goal_phase)

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        # Set initial pose
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0

        yaw = math.radians(0)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        msg.pose.covariance = [0.0] * 36

        self.init_pose_pub.publish(msg)

    def start_goal_phase(self):
        # Cancel the initial pose timer and phase timer
        if self.init_timer is not None:
            self.init_timer.cancel()
        if self.phase_timer is not None:
            self.phase_timer.cancel()

        # Create a timer to publish goal pose every 0.1 seconds (runs for 1 second)
        self.goal_timer = self.create_timer(0.1, self.publish_goal)
        # Create a timer to shutdown the node after 1 second
        self.shutdown_timer = self.create_timer(2.0, self.shutdown_node)

    def publish_goal(self):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'

        # Set goal pose
        goal_msg.pose.position.x = 1.4
        goal_msg.pose.position.y = -1.4
        goal_msg.pose.position.z = 0.0

        yaw = math.radians(0)
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
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
    node = Nav2GoalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
