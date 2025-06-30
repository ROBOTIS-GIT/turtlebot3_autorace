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

import time

import rclpy
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class ArmController(Node):
    POSITION_TOLERANCE = 0.3

    def __init__(self):
        super().__init__('arm_controller')
        self.current_position = 0.0

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10)

        self.subscription = self.create_subscription(
            String,
            '/arm_control',
            self.arm_control_callback,
            10)

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        self.state_change_client = self.create_client(
            Trigger,
            'state_change_trigger'
        )

        self.get_logger().info('Joint Trajectory Publisher node has been configured')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Joint Trajectory Publisher node has been activated')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Joint Trajectory Publisher node has been deactivated')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.destroy_publisher(self.publisher)
        self.destroy_subscription(self.subscription)
        self.destroy_subscription(self.joint_state_sub)
        self.destroy_client(self.state_change_client)

        self.publisher = None
        self.subscription = None
        self.joint_state_sub = None
        self.state_change_client = None
        self.current_position = 0.0

        self.get_logger().info('Joint Trajectory Publisher node has been cleaned up')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.destroy_publisher(self.publisher)
        self.destroy_subscription(self.subscription)
        self.get_logger().info('Joint Trajectory Publisher node has been shut down')
        return TransitionCallbackReturn.SUCCESS

    def joint_state_callback(self, msg):
        if 'joint1' in msg.name:
            idx = msg.name.index('joint1')
            self.current_position = msg.position[idx]

    def verify_position(self, target_pos):
        start_time = time.time()
        timeout = 5.0

        while time.time() - start_time < timeout:
            if abs(self.current_position - target_pos) < self.POSITION_TOLERANCE:
                self.get_logger().info(
                    f'Position verified: {self.current_position:.2f} (target: {target_pos:.2f})')
                return True
            time.sleep(0.1)

        self.get_logger().warn(
            f'Position verification failed: {self.current_position:.2f}'
            f'(target: {target_pos:.2f})')
        return False

    def trigger_callback(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('Successfully sent state change trigger')
            else:
                self.get_logger().error('Failed to send state change trigger')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

    def arm_control_callback(self, msg):
        trajectory_msg = JointTrajectory()

        trajectory_msg.joint_names = ['joint1']

        point = JointTrajectoryPoint()
        if msg.data == 'pick_up':
            position = -1.0
        elif msg.data == 'place':
            position = 1.0
        else:
            self.get_logger().warn(f'Unknown command: {msg.data}')
            return
        point.positions = [position]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 0

        trajectory_msg.points.append(point)

        self.publisher.publish(trajectory_msg)
        self.get_logger().info(f'Published joint trajectory with position {position}')

        time.sleep(3.0)
        request = Trigger.Request()
        future = self.state_change_client.call_async(request)
        future.add_done_callback(self.trigger_callback)


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
