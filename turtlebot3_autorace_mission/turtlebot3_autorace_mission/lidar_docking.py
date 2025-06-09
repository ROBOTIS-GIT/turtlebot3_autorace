#!/usr/bin/env python3
#################################################################################
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
#################################################################################
#
# Authors: ChanHyeong Lee

import math
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import numpy
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import LifecycleState
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import LaserScan


class LiDARDocking(LifecycleNode):

    KP = 0.01
    DISTANCE_THRESHOLD = 0.05
    HEADING_ALIGNMENT_THRESHOLD = 1.0

    def __init__(self):
        super().__init__('lidar_docking')
        self.is_active = False

        self.cv_bridge = None
        self.current_yaw_deg = 0.0
        self.heading_aligned = False
        self.parking_finished = False
        self.final_alignment_started = False

        self.sub_scan = None
        self.sub_odom = None
        self.pub_lidar_img = None
        self.pub_cmd_vel = None

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        try:
            self.cv_bridge = CvBridge()

            self.sub_scan = self.create_subscription(
                LaserScan,
                '/scan',
                self.scan_callback,
                qos_profile_sensor_data
            )
            self.sub_odom = self.create_subscription(
                Odometry,
                '/odom',
                self.odom_callback,
                10
            )

            self.pub_lidar_img = self.create_lifecycle_publisher(
                CompressedImage,
                '/control/wall_image/compressed',
                qos_profile_sensor_data
            )
            self.pub_cmd_vel = self.create_lifecycle_publisher(
                TwistStamped,
                '/cmd_vel',
                10
            )

            self.get_logger().info('LiDAR docking node configured successfully')
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
            self.get_logger().info('LiDAR docking node activated successfully')
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
            self.get_logger().info('LiDAR docking node deactivated successfully')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Deactivation failed: {str(e)}')
            return TransitionCallbackReturn.FAILURE

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        try:
            self.sub_scan = None
            self.sub_odom = None
            self.pub_lidar_img = None
            self.pub_cmd_vel = None

            self.is_active = False
            self.current_yaw_deg = 0.0
            self.heading_aligned = False
            self.parking_finished = False
            self.final_alignment_started = False
            self.cv_bridge = None

            self.get_logger().info('LiDAR docking node cleaned up successfully')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Cleanup failed: {str(e)}')
            return TransitionCallbackReturn.FAILURE

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        try:
            self.get_logger().info('LiDAR docking node shutting down...')
            def shutdown_node():
                time.sleep(0.1)
                rclpy.shutdown()
            import threading
            threading.Thread(target=shutdown_node).start()
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Shutdown failed: {str(e)}')
            return TransitionCallbackReturn.FAILURE

    def odom_callback(self, msg: Odometry):
        if not self.is_active:
            return

        q = msg.pose.pose.orientation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_yaw_deg = math.degrees(yaw)

        if not self.heading_aligned:
            if abs(self.current_yaw_deg) > self.HEADING_ALIGNMENT_THRESHOLD:
                self.heading_aligned = False
                self.align_heading()
            else:
                if not self.heading_aligned:
                    self.get_logger().info('Initial heading alignment completed.')
                self.heading_aligned = True

    def align_heading(self):
        if not self.is_active:
            return

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        angular_vel = -0.3 if self.current_yaw_deg > 0 else 0.3
        cmd.twist.angular.z = angular_vel
        cmd.twist.linear.x = 0.0

        self.pub_cmd_vel.publish(cmd)

    def scan_callback(self, scan_msg: LaserScan):
        if not self.is_active:
            return

        if not self.heading_aligned:
            return

        width, height = 400, 400
        img = numpy.zeros((height, width, 3), dtype=numpy.uint8)
        cx, cy = width // 2, height // 2
        scale = 400

        roi_points = []
        for i, range in enumerate(scan_msg.ranges):
            if not numpy.isfinite(range):
                continue

            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            distance_x = range * math.cos(angle)
            distance_y = range * math.sin(angle)

            raw_x = cx + distance_x * scale
            raw_y = cy + distance_y * scale
            x = int(raw_x)
            y = int(raw_y)

            color = (100, 100, 100)

            if -0.1 < distance_x < 0.5 and -0.2 < distance_y < 0.2:
                roi_points.append((raw_x, raw_y))
                color = (0, 0, 255)

            cv2.circle(img, (y, x), 1, color, -1)

        if len(roi_points) > 0:
            roi_np = numpy.array(roi_points)
            mean_x = int(numpy.mean(roi_np[:, 0]))
            mean_y = int(numpy.mean(roi_np[:, 1]))

            cv2.circle(img, (mean_y, mean_x), 3, (255, 0, 0), -1)

            try:
                self.distance_to_wall = float((mean_x - cx) / scale)
                error_px = mean_y - cx

                self.tracking_center_point(error_px)

            except Exception as e:
                self.get_logger().warn(f'Processing failed: {e}')
                return

        cv2.circle(img, (cx, cy), 4, (255, 255, 255), -1)
        img = cv2.rotate(img, cv2.ROTATE_180)

        try:
            img_msg = self.cv_bridge.cv2_to_compressed_imgmsg(img)
            self.pub_lidar_img.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')

    def tracking_center_point(self, error_px: int):
        if not self.is_active:
            return

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        if not self.parking_finished:
            if self.distance_to_wall > self.DISTANCE_THRESHOLD:
                cmd.twist.linear.x = 0.03
                cmd.twist.angular.z = self.KP * error_px
            else:
                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = 0.0
                self.parking_finished = True
                self.final_alignment_started = True
                self.get_logger().info('Parking completed! Starting final heading alignment.')

        elif self.final_alignment_started:
            if abs(self.current_yaw_deg) > self.HEADING_ALIGNMENT_THRESHOLD:
                self.align_heading()
                self.get_logger().info(f'Final heading alignment in progress: {self.current_yaw_deg:.2f} degrees')
                return
            else:
                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = 0.0
                self.final_alignment_started = False
                self.get_logger().info('Final heading alignment completed!')

        self.pub_cmd_vel.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    lidar_docking = LiDARDocking()
    try:
        rclpy.spin(lidar_docking)
    except KeyboardInterrupt:
        pass
    finally:
        lidar_docking.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
