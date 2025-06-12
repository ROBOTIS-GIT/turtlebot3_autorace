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

from enum import Enum
import math
import time

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import LifecycleState
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import QoSProfile
from scipy.spatial.transform import Rotation
from std_msgs.msg import Int32
from std_srvs.srv import Trigger
import tf2_ros


class ArUcoParking(LifecycleNode):

    def __init__(self):
        super().__init__('aruco_parking')

        self.ParkingSequence = Enum(
            'ParkingSequence',
            'waiting search_parking_lot align_direction move_nearby_parking_lot parking stop initialization'
        )

        self.NearbySequence = Enum(
            'NearbySequence',
            'initial_turn go_straight turn_right parking'
        )

        self.marker_frame = 'ar_marker_0'
        self.tf_buffer = None
        self.tf_listener = None
        self.timer = None
        self.is_active = False

        self.current_nearby_sequence = self.NearbySequence.initial_turn.value
        self.current_parking_sequence = self.ParkingSequence.waiting.value

        self.robot_2d_pose_x = .0
        self.robot_2d_pose_y = .0
        self.robot_2d_theta = .0
        self.marker_2d_pose_x = .0
        self.marker_2d_pose_y = .0
        self.marker_2d_theta = .0

        self.parking_distance_to_marker = 0.30

        self.previous_robot_2d_theta = .0
        self.total_robot_2d_theta = .0
        self.is_triggered = False

        self.is_sequence_finished = False
        self.is_odom_received = False
        self.is_marker_pose_received = False

        self.sub_odom_robot = None
        self.pub_cmd_vel = None
        self.state_change_client = None

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        try:
            self.sub_odom_robot = self.create_subscription(
                Odometry,
                '/odom',
                self.get_robot_odom,
                qos_profile=QoSProfile(depth=10)
            )

            self.pub_cmd_vel = self.create_lifecycle_publisher(
                TwistStamped,
                '/cmd_vel',
                qos_profile=QoSProfile(depth=10)
            )

            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.timer = self.create_timer(0.1, self.get_marker_odom)

            self.state_change_client = self.create_client(
                Trigger,
                'state_change_trigger'
            )

            self.get_logger().info('ArUco parking node configured successfully')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Configuration failed: {str(e)}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        try:
            ret = super().on_activate(state)
            if ret != TransitionCallbackReturn.SUCCESS:
                return ret

            self.is_active = True
            self.get_logger().info('Start parking sequence with marker 0')
            self.current_parking_sequence = self.ParkingSequence.search_parking_lot.value
            self.timer = self.create_timer(0.1, self._run)
            self.get_logger().info('ArUco parking node activated successfully')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Activation failed: {str(e)}')
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        try:
            ret = super().on_deactivate(state)
            if ret != TransitionCallbackReturn.SUCCESS:
                return ret

            self.is_active = False
            self.get_logger().info('ArUco parking node deactivated successfully')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Deactivation failed: {str(e)}')
            return TransitionCallbackReturn.FAILURE

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        try:
            self.sub_odom_robot = None
            self.pub_cmd_vel = None
            self.tf_buffer = None
            self.tf_listener = None
            self.timer = None

            self.marker_frame = 'ar_marker_0'
            self.current_nearby_sequence = self.NearbySequence.initial_turn.value
            self.current_parking_sequence = self.ParkingSequence.waiting.value

            self.robot_2d_pose_x = .0
            self.robot_2d_pose_y = .0
            self.robot_2d_theta = .0
            self.marker_2d_pose_x = .0
            self.marker_2d_pose_y = .0
            self.marker_2d_theta = .0

            self.previous_robot_2d_theta = .0
            self.total_robot_2d_theta = .0
            self.is_triggered = False

            self.is_sequence_finished = False
            self.is_odom_received = False
            self.is_marker_pose_received = False
            self.state_change_client = None

            self.get_logger().info('ArUco parking node cleaned up successfully')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Cleanup failed: {str(e)}')
            return TransitionCallbackReturn.FAILURE

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        try:
            self.get_logger().info('ArUco parking node shutting down...')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Shutdown failed: {str(e)}')
            return TransitionCallbackReturn.FAILURE

    def get_robot_odom(self, robot_odom_msg):
        if not self.is_active:
            return

        if not self.is_odom_received:
            self.is_odom_received = True
            self.get_logger().info('odom is received!')

        pos_x, pos_y, theta = self.get_2D_robot_pose(robot_odom_msg)

        self.robot_2d_pose_x = pos_x
        self.robot_2d_pose_y = pos_y
        self.robot_2d_theta = theta

        if (self.robot_2d_theta - self.previous_robot_2d_theta) > 5.:
            d_theta = (self.robot_2d_theta - self.previous_robot_2d_theta) - 2 * math.pi
        elif (self.robot_2d_theta - self.previous_robot_2d_theta) < -5.:
            d_theta = (self.robot_2d_theta - self.previous_robot_2d_theta) + 2 * math.pi
        else:
            d_theta = (self.robot_2d_theta - self.previous_robot_2d_theta)

        self.total_robot_2d_theta += d_theta
        self.previous_robot_2d_theta = self.robot_2d_theta
        self.robot_2d_theta = self.total_robot_2d_theta

    def get_marker_odom(self):
        if not self.is_active:
            return

        if self.marker_frame is None:
            return

        result = self.get_2D_marker_pose()
        if result is None:
            return
        pos_x, pos_y, theta = result

        self.marker_2d_pose_x = pos_x
        self.marker_2d_pose_y = pos_y
        self.marker_2d_theta = theta - math.pi

        self.is_marker_pose_received = True

    def _run(self):
        if not self.is_active:
            return

        if self.marker_frame is not None:
            self.fn_parking()

    def fn_parking(self):
        if self.current_parking_sequence == self.ParkingSequence.search_parking_lot.value:
            self.is_sequence_finished = self.seq_find_goal()
            if self.is_sequence_finished:
                self.get_logger().info('Finished find goal sequence')
                self.is_sequence_finished = False
                self.current_parking_sequence = self.ParkingSequence.align_direction.value

        elif self.current_parking_sequence == self.ParkingSequence.align_direction.value:
            self.is_sequence_finished = self.seq_align_direction()
            if self.is_sequence_finished:
                self.get_logger().info('Finished align direction sequence')
                self.is_sequence_finished = False
                self.current_parking_sequence = self.ParkingSequence.move_nearby_parking_lot.value

        elif self.current_parking_sequence == self.ParkingSequence.move_nearby_parking_lot.value:
            self.is_sequence_finished = self.seq_move_nearby_parking_lot()
            if self.is_sequence_finished:
                self.get_logger().info('Finished move nearby parking lot sequence')
                self.is_sequence_finished = False
                self.current_parking_sequence = self.ParkingSequence.parking.value

        elif self.current_parking_sequence == self.ParkingSequence.parking.value:
            self.is_sequence_finished = self.seq_parking()
            if self.is_sequence_finished:
                self.get_logger().info('Finished parking sequence')
                self.is_sequence_finished = False
                self.current_parking_sequence = self.ParkingSequence.initialization.value

        elif self.current_parking_sequence == self.ParkingSequence.initialization.value:
            self.fn_stop()
            self.get_logger().info('--------------------')
            self.current_parking_sequence = self.ParkingSequence.initialization.value
            self.set_init()

    def seq_find_goal(self):
        if not self.is_marker_pose_received:
            self.desired_angle_turn = -0.3
            self.fn_turn(self.desired_angle_turn)
        else:
            self.fn_stop()
            return True

    def seq_align_direction(self):
        desired_angle_turn = -1. * math.atan2(self.marker_2d_pose_y, self.marker_2d_pose_x)
        self.fn_turn(desired_angle_turn)

        if abs(desired_angle_turn) < 0.01:
            self.fn_stop()
            time.sleep(1)
            if abs(desired_angle_turn) < 0.01:
                return True
        return False

    def seq_move_nearby_parking_lot(self):
        if self.current_nearby_sequence == self.NearbySequence.initial_turn.value:
            if not self.is_triggered:
                self.is_triggered = True
                self.initial_robot_pose_theta = self.robot_2d_theta
                self.initial_robot_pose_x = self.robot_2d_pose_x
                self.initial_robot_pose_y = self.robot_2d_pose_y
                self.initial_marker_pose_theta = self.marker_2d_theta
                self.initial_marker_pose_x = self.marker_2d_pose_x

            if self.initial_marker_pose_theta < 0.0:
                desired_angle_turn = (math.pi / 2.0) + self.initial_marker_pose_theta - (
                    self.robot_2d_theta - self.initial_robot_pose_theta)
            elif self.initial_marker_pose_theta > 0.0:
                desired_angle_turn = -(math.pi / 2.0) + self.initial_marker_pose_theta - (
                    self.robot_2d_theta - self.initial_robot_pose_theta)
            else:
                desired_angle_turn = 0.0
            desired_angle_turn = -1. * desired_angle_turn

            self.fn_turn(desired_angle_turn)

            if abs(desired_angle_turn) < 0.05:
                self.fn_stop()
                self.current_nearby_sequence = self.NearbySequence.go_straight.value
                self.is_triggered = False

        elif self.current_nearby_sequence == self.NearbySequence.go_straight.value:
            dist_from_start = self.calculate_distance_points(
                self.initial_robot_pose_x, self.robot_2d_pose_x,
                self.initial_robot_pose_y, self.robot_2d_pose_y)

            desired_dist = self.initial_marker_pose_x * abs(
                math.cos((math.pi / 2.) - self.initial_marker_pose_theta))
            remained_dist = desired_dist - dist_from_start

            self.fn_go_straight()
            if remained_dist < 0.001:
                self.fn_stop()
                self.current_nearby_sequence = self.NearbySequence.turn_right.value

        elif self.current_nearby_sequence == self.NearbySequence.turn_right.value:
            if not self.is_triggered:
                self.is_triggered = True
                self.initial_robot_pose_theta = self.robot_2d_theta

            if self.initial_marker_pose_theta < 0.0:
                desired_angle_turn = -(math.pi / 2.0) + (
                    self.robot_2d_theta - self.initial_robot_pose_theta)
            elif self.initial_marker_pose_theta > 0.0:
                desired_angle_turn = (math.pi / 2.0) + (
                    self.robot_2d_theta - self.initial_robot_pose_theta)
            else:
                desired_angle_turn = 0.0

            self.fn_turn(desired_angle_turn)

            if abs(desired_angle_turn) < 0.05:
                self.fn_stop()
                self.current_nearby_sequence = self.NearbySequence.parking.value
                self.is_triggered = False
                return True

        return False

    def seq_parking(self):
        desired_angle_turn = math.atan2(self.marker_2d_pose_y, self.marker_2d_pose_x)
        self.fn_track_marker(-desired_angle_turn)
        if abs(self.marker_2d_pose_x) < self.parking_distance_to_marker:
            self.fn_stop()
            request = Trigger.Request()
            future = self.state_change_client.call_async(request)
            future.add_done_callback(self.trigger_callback)
            return True
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

    def fn_stop(self):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'
        twist.twist.linear.x = 0.0
        twist.twist.angular.z = 0.0
        self.pub_cmd_vel.publish(twist)

    def fn_turn(self, theta):
        Kp = 1.0
        angular_z = Kp * theta

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'
        twist.twist.linear.x = 0.0
        twist.twist.angular.z = -angular_z
        self.pub_cmd_vel.publish(twist)

    def fn_go_straight(self):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'
        twist.twist.linear.x = 0.1
        twist.twist.angular.z = 0.0
        self.pub_cmd_vel.publish(twist)

    def fn_track_marker(self, theta):
        Kp = 0.6
        angular_z = Kp * theta

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'
        twist.twist.linear.x = 0.1
        twist.twist.angular.z = -angular_z
        self.pub_cmd_vel.publish(twist)

    def get_2D_robot_pose(self, robot_odom_msg):
        quaternion = (
            robot_odom_msg.pose.pose.orientation.x,
            robot_odom_msg.pose.pose.orientation.y,
            robot_odom_msg.pose.pose.orientation.z,
            robot_odom_msg.pose.pose.orientation.w)

        r = Rotation.from_quat(quaternion)
        euler_angles = r.as_euler('xyz', degrees=False)
        theta = euler_angles[2] % (2 * np.pi)

        pos_x = robot_odom_msg.pose.pose.position.x
        pos_y = robot_odom_msg.pose.pose.position.y

        return pos_x, pos_y, theta

    def get_2D_marker_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'base_link', self.marker_frame, rclpy.time.Time().to_msg())
        except Exception:
            return

        quaternion = (
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w
        )

        rotation = Rotation.from_quat(quaternion)
        euler_angles = rotation.as_euler('xyz', degrees=False)
        theta = (euler_angles[2] + np.pi / 2.) % (2 * np.pi)

        pos_x = trans.transform.translation.x
        pos_y = trans.transform.translation.y

        return pos_x, pos_y, theta

    def calculate_distance_points(self, x1, x2, y1, y2):
        return math.sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)

    def set_init(self):
        self.get_logger().info('Initialize sequences')
        self.marker_frame = 'ar_marker_0'
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.get_marker_odom)

        self.current_nearby_sequence = self.NearbySequence.initial_turn.value
        self.current_parking_sequence = self.ParkingSequence.waiting.value

        self.robot_2d_pose_x = .0
        self.robot_2d_pose_y = .0
        self.robot_2d_theta = .0
        self.marker_2d_pose_x = .0
        self.marker_2d_pose_y = .0
        self.marker_2d_theta = .0

        self.previous_robot_2d_theta = .0
        self.total_robot_2d_theta = .0
        self.is_triggered = False

        self.is_sequence_finished = False
        self.is_odom_received = False
        self.is_marker_pose_received = False

        self.timer = self.create_timer(0.1, self._run)


def main(args=None):
    rclpy.init(args=args)
    node = ArUcoParking()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
