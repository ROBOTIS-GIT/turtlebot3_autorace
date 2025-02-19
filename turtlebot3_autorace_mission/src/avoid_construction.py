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

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import UInt8

from turtlebot3_autorace_msgs.msg import Objects


def euler_from_quaternion(msg):
    """
    Convert a quaternion into Euler angles (roll, pitch, yaw).

    msg: geometry_msgs.msg.Quaternion.
    return: (roll, pitch, yaw) tuple.
    """
    x = msg.x
    y = msg.y
    z = msg.z
    w = msg.w
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = max(min(t2, 1.0), -1.0)
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


class DangerZoneAvoidance(Node):
    def __init__(self):
        super().__init__('danger_zone_avoidance')

        self.tracked_objects_sub = self.create_subscription(
            Objects,
            '/tracked_objects',
            self.tracked_objects_callback,
            10
        )
        self.lane_state_sub = self.create_subscription(
            UInt8,
            '/detect/lane_state',
            self.lane_state_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_projected',
            self.image_callback,
            10
        )

        self.bridge = CvBridge()
        self.lane_detected = False

        self.image_pub = self.create_publisher(
            Image,
            '/lane_detection/image_result',
            10
        )
        self.avoid_cmd_pub = self.create_publisher(
            Twist,
            '/avoid_control',
            10
        )
        self.avoid_active_pub = self.create_publisher(
            Bool,
            '/avoid_active',
            10
        )

        # Parameter settings
        self.danger_distance = 0.24    # Danger zone y threshold (meters)
        self.danger_width = 0.12       # Danger zone x width (meters)
        self.speed = 0.03              # Forward speed during avoidance

        # PD control parameters (for turning)
        self.turn_Kp = 0.45
        self.turn_Kd = 0.03
        self.turn_threshold = 0.05     # Tolerance to decide when turning is complete
        self.last_turn_error = 0.0

        # State machine variables
        # States: "NORMAL", "AVOID_TURN", "AVOID_STRAIGHT", "RETURN_TURN"
        self.state = "NORMAL"
        self.turn_direction = None     # "left" or "right"
        self.desired_theta = None      # Target angle (radians)
        self.original_theta = None     # Original heading at the start of avoidance mode

        # Current odom information
        self.current_theta = 0.0
        self.current_pos_x = 0.0
        self.current_pos_y = 0.0

        # lane_state value: 1 (left lane only), 2 (both), 3 (right lane only), 4 (not detected)
        self.lane_state = None

        self.tracked = None

        self.timer = self.create_timer(0.1, self.process_loop)

    def tracked_objects_callback(self, msg):
        self.tracked = msg.objects
        self.visualize_clusters(self.tracked)

    def lane_state_callback(self, msg):
        self.lane_state = msg.data

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        euler = euler_from_quaternion(q)
        self.current_theta = euler[2]
        self.current_pos_x = msg.pose.pose.position.x
        self.current_pos_y = msg.pose.pose.position.y

    def image_callback(self, msg):
        # If avoidance mode is not active, do not perform lane detection.
        if self.state == "NORMAL":
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            self.get_logger().error("Image conversion failed")
            return

        img_height, img_width = cv_image.shape[:2]

        # Define ROI
        roi_size = img_height
        roi_x_start = (img_width - roi_size) // 2
        roi_x_end = roi_x_start + roi_size
        roi = cv_image[0:roi_size, roi_x_start:roi_x_end]

        margin = 20

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)
        lines = cv2.HoughLinesP(
            edges, 1, np.pi/180, threshold=50, minLineLength=100, maxLineGap=10
        )

        positive_found = False
        negative_found = False

        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    # Calculate line length
                    line_length = np.hypot(x2 - x1, y2 - y1)
                    if line_length < 100:
                        continue
                    # Check if the line covers near the top and bottom of the ROI
                    if max(y1, y2) < margin or min(y1, y2) > (roi_size - margin):
                        continue

                    if (x2 - x1) == 0:
                        continue
                    slope = (y2 - y1) / (x2 - x1)
                    if slope > 0:
                        positive_found = True
                    elif slope < 0:
                        negative_found = True
            if positive_found and negative_found:
                self.lane_detected = True
            else:
                self.lane_detected = False
        else:
            self.lane_detected = False

        overlay_text = "Lane Detected" if self.lane_detected else "Lane Not Detected"
        text_color = (0, 255, 0) if self.lane_detected else (0, 0, 255)
        cv2.putText(
            roi, overlay_text, (50, 50),
            cv2.FONT_HERSHEY_SIMPLEX, 1, text_color, 2
        )

        result_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.image_pub.publish(result_msg)

    def process_loop(self):
        if self.state == "NORMAL":
            self.process_normal_state()
        elif self.state == "AVOID_TURN":
            self.process_avoid_turn_state()
        elif self.state == "AVOID_STRAIGHT":
            self.process_avoid_straight_state()
        elif self.state == "RETURN_TURN":
            self.process_return_turn_state()

    def process_normal_state(self):
        if self.tracked is not None:
            danger_detected = False
            for obj in self.tracked:
                for pt in obj.raw_points:
                    if (
                        pt.y > 0 and
                        pt.y < self.danger_distance and
                        abs(pt.x) < (self.danger_width / 2)
                    ):
                        danger_detected = True
                        break
                if danger_detected:
                    break
            if danger_detected:
                self.get_logger().info("Danger zone intrusion detected.")
                if self.lane_state in [1, 3]:
                    self.original_theta = self.current_theta
                    if self.lane_state == 1:
                        self.turn_direction = "right"
                        self.desired_theta = self.normalize_angle(
                            self.current_theta - math.radians(80)
                        )
                    elif self.lane_state == 3:
                        self.turn_direction = "left"
                        self.desired_theta = self.normalize_angle(
                            self.current_theta + math.radians(80)
                        )
                    self.get_logger().info(f"Avoidance mode on: turning {self.turn_direction}.")
                    self.state = "AVOID_TURN"
                else:
                    self.get_logger().info("lane_state value does not meet activation conditions.")

    def process_avoid_turn_state(self):
        error = self.normalize_angle(self.desired_theta - self.current_theta)
        angular_z = self.turn_Kp * error + self.turn_Kd * (error - self.last_turn_error)
        self.last_turn_error = error
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_z
        self.avoid_cmd_pub.publish(twist)
        self.publish_active(True)
        if abs(error) < self.turn_threshold:
            self.get_logger().info("Avoidance turn completed. Moving straight.")
            self.last_turn_error = 0.0
            self.state = "AVOID_STRAIGHT"

    def process_avoid_straight_state(self):
        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = 0.0
        self.avoid_cmd_pub.publish(twist)
        self.publish_active(True)
        if self.lane_detected:
            self.get_logger().info("Lane detected. Starting return turn.")
            self.desired_theta = self.normalize_angle(self.original_theta)
            self.last_turn_error = 0.0
            self.state = "RETURN_TURN"

    def process_return_turn_state(self):
        error = self.normalize_angle(self.desired_theta - self.current_theta)
        angular_z = self.turn_Kp * error + self.turn_Kd * (error - self.last_turn_error)
        self.last_turn_error = error
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_z
        self.avoid_cmd_pub.publish(twist)
        self.publish_active(True)
        if abs(error) < self.turn_threshold:
            self.get_logger().info("Return turn completed. Switching to normal driving.")
            self.state = "NORMAL"
            self.publish_active(False)
            twist = Twist()
            self.avoid_cmd_pub.publish(twist)

    def publish_active(self, active: bool):
        bool_msg = Bool()
        bool_msg.data = active
        self.avoid_active_pub.publish(bool_msg)

    def visualize_clusters(self, tracked):
        img_vis = np.zeros((500, 500, 3), dtype=np.uint8)
        scale = 400.0  # 1 m = 400 pixels
        center = (img_vis.shape[1] // 2, img_vis.shape[0] // 2)
        robot_width_m = 0.12
        robot_height_m = 0.12
        robot_width_px = int(robot_width_m * scale)
        robot_height_px = int(robot_height_m * scale)
        tb_top_left = (center[0] - robot_width_px // 2, center[1] - robot_height_px // 2)
        tb_bottom_right = (center[0] + robot_width_px // 2, center[1] + robot_height_px // 2)
        cv2.rectangle(img_vis, tb_top_left, tb_bottom_right, (255, 0, 0), 2)
        cv2.putText(img_vis, 'TurtleBot', (tb_top_left[0], tb_top_left[1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        danger_height = self.danger_distance - (robot_height_m / 2)
        danger_width = robot_width_m
        dz_top_left = (
            center[0] - int(danger_width * scale / 2), tb_top_left[1] - int(danger_height * scale)
            )
        dz_bottom_right = (
            center[0] + int(danger_width * scale / 2), tb_top_left[1]
            )
        overlay = img_vis.copy()
        cv2.rectangle(overlay, dz_top_left, dz_bottom_right, (0, 0, 255), -1)
        alpha = 0.3
        img_vis = cv2.addWeighted(overlay, alpha, img_vis, 1 - alpha, 0)
        if tracked is not None:
            for obj in tracked:
                centroid_x = obj.x
                centroid_y = obj.y
                obj_id = obj.id
                raw_points = obj.raw_points
                for pt in raw_points:
                    x_pt = int(center[0] + pt.x * scale)
                    y_pt = int(center[1] - pt.y * scale)
                    cv2.circle(img_vis, (x_pt, y_pt), 2, (0, 255, 0), -1)
                cx = int(center[0] + centroid_x * scale)
                cy = int(center[1] - centroid_y * scale)
                cv2.circle(img_vis, (cx, cy), 4, (0, 0, 255), -1)
                cv2.putText(img_vis, f'ID:{obj_id}', (cx + 5, cy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.imshow("Cluster Points & Danger Zone", img_vis)
        cv2.waitKey(1)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = DangerZoneAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
