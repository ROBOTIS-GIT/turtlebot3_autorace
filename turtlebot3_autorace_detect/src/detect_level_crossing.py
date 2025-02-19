#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
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

# Authors: Leon Jung, [AuTURBO] Kihoon Kim (https://github.com/auturbo), Gilbert, Ashe Kim, ChanHyeong Lee

import math
import time
from enum import Enum

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import IntegerRange
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_msgs.msg import UInt8


def fnCalcDistanceDot2Line(a, b, c, x0, y0):
    distance = abs(x0 * a + y0 * b + c) / math.sqrt(a * a + b * b)
    return distance


def fnCalcDistanceDot2Dot(x1, y1, x2, y2):
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return distance


def fnArrangeIndexOfPoint(arr):
    new_arr = arr[:]
    arr_idx = list(range(len(arr)))
    for i in range(len(arr)):
        for j in range(i + 1, len(arr)):
            if new_arr[i] < new_arr[j]:
                new_arr[i], new_arr[j] = new_arr[j], new_arr[i]
                arr_idx[i], arr_idx[j] = arr_idx[j], arr_idx[i]
    return arr_idx


def fnCheckLinearity(point1, point2, point3):
    threshold_linearity = 50
    x1, y1 = point1
    x2, y2 = point3
    if x2 - x1 != 0:
        a = (y2 - y1) / (x2 - x1)
    else:
        a = 1000
    b = -1
    c = y1 - a * x1
    err = fnCalcDistanceDot2Line(a, b, c, point2[0], point2[1])
    return err < threshold_linearity


def fnCheckDistanceIsEqual(point1, point2, point3):
    threshold_distance_equality = 3
    distance1 = fnCalcDistanceDot2Dot(point1[0], point1[1], point2[0], point2[1])
    distance2 = fnCalcDistanceDot2Dot(point2[0], point2[1], point3[0], point3[1])
    std = np.std([distance1, distance2])
    return std < threshold_distance_equality


# ------------------------
# ROS2 Node: DetectLevelNode
# ------------------------
class DetectLevelNode(Node):
    def __init__(self):
        super().__init__('detect_level')
        self.get_logger().info("Starting detect_level node (ROS2)")

        hue_range = IntegerRange(from_value=0, to_value=179, step=1)
        sat_range = IntegerRange(from_value=0, to_value=255, step=1)
        light_range = IntegerRange(from_value=0, to_value=255, step=1)

        hue_l_descriptor = ParameterDescriptor(
            description="Lower hue threshold",
            integer_range=[hue_range]
        )
        hue_h_descriptor = ParameterDescriptor(
            description="Upper hue threshold",
            integer_range=[hue_range]
        )
        sat_l_descriptor = ParameterDescriptor(
            description="Lower saturation threshold",
            integer_range=[sat_range]
        )
        sat_h_descriptor = ParameterDescriptor(
            description="Upper saturation threshold",
            integer_range=[sat_range]
        )
        light_l_descriptor = ParameterDescriptor(
            description="Lower value (lightness) threshold",
            integer_range=[light_range]
        )
        light_h_descriptor = ParameterDescriptor(
            description="Upper value (lightness) threshold",
            integer_range=[light_range]
        )

        # delcare parameters
        self.declare_parameter("detect.level.red.hue_l", 0, descriptor=hue_l_descriptor)
        self.declare_parameter("detect.level.red.hue_h", 179, descriptor=hue_h_descriptor)
        self.declare_parameter("detect.level.red.saturation_l", 24, descriptor=sat_l_descriptor)
        self.declare_parameter("detect.level.red.saturation_h", 255, descriptor=sat_h_descriptor)
        self.declare_parameter("detect.level.red.lightness_l", 207, descriptor=light_l_descriptor)
        self.declare_parameter("detect.level.red.lightness_h", 162, descriptor=light_h_descriptor)

        self.declare_parameter("is_detection_calibration_mode", False)
        self.declare_parameter("sub_image_type", "compressed")  # "raw" or "compressed"
        self.declare_parameter("pub_image_type", "compressed")  # "raw" or "compressed"

        # get parameters
        self.hue_red_l = self.get_parameter("detect.level.red.hue_l").value
        self.hue_red_h = self.get_parameter("detect.level.red.hue_h").value
        self.saturation_red_l = self.get_parameter("detect.level.red.saturation_l").value
        self.saturation_red_h = self.get_parameter("detect.level.red.saturation_h").value
        self.lightness_red_l = self.get_parameter("detect.level.red.lightness_l").value
        self.lightness_red_h = self.get_parameter("detect.level.red.lightness_h").value
        self.is_calibration_mode = self.get_parameter("is_detection_calibration_mode").value

        self.sub_image_type = self.get_parameter("sub_image_type").value
        self.pub_image_type = self.get_parameter("pub_image_type").value

        self.add_on_set_parameters_callback(self.on_parameter_change)

        self.StepOfLevelCrossing = Enum('StepOfLevelCrossing', 'pass_level exit')

        self.is_level_crossing_finished = False
        self.stop_bar_count = 0
        self.counter = 1
        self.cv_image = None

        self.cv_bridge = CvBridge()

        # create publishers
        if self.pub_image_type == "compressed":
            self.pub_image_level = self.create_publisher(
                CompressedImage, '/detect/image_output/compressed', 10)
            if self.is_calibration_mode:
                self.pub_image_color_filtered = self.create_publisher(
                    CompressedImage, '/detect/image_output_sub1/compressed', 10)
        else:  # raw
            self.pub_image_level = self.create_publisher(
                Image, '/detect/image_output', 10)
            if self.is_calibration_mode:
                self.pub_image_color_filtered = self.create_publisher(
                    Image, '/detect/image_output_sub1', 10)

        self.pub_level_crossing_return = self.create_publisher(
            UInt8, '/detect/level_crossing_stamped', 10)
        self.pub_max_vel = self.create_publisher(
            Float64, '/control/max_vel',)

        # create subscribers
        if self.sub_image_type == "compressed":
            self.create_subscription(
                CompressedImage,
                '/detect/image_input/compressed',
                self.get_image,
                10
                )
        else:  # raw
            self.create_subscription(
                Image,
                '/detect/image_input',
                self.get_image,
                10
                )

        self.create_subscription(
            UInt8,
            '/detect/level_crossing_order',
            self.level_crossing_order,
            10
            )

        self.timer = self.create_timer(1.0/15.0, self.timer_callback)

        time.sleep(1.0)

    def on_parameter_change(self, params):
        for param in params:
            if param.name == "detect.level.red.hue_l":
                self.hue_red_l = param.value
            elif param.name == "detect.level.red.hue_h":
                self.hue_red_h = param.value
            elif param.name == "detect.level.red.saturation_l":
                self.saturation_red_l = param.value
            elif param.name == "detect.level.red.saturation_h":
                self.saturation_red_h = param.value
            elif param.name == "detect.level.red.lightness_l":
                self.lightness_red_l = param.value
            elif param.name == "detect.level.red.lightness_h":
                self.lightness_red_h = param.value
        self.get_logger().info("Dynamic parameters updated.")
        return SetParametersResult(successful=True)

    def timer_callback(self):
        if self.cv_image is not None:
            self.find_level()

    def get_image(self, image_msg):
        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1

        if self.sub_image_type == "compressed":
            np_arr = np.frombuffer(image_msg.data, np.uint8)
            self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            try:
                self.cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
            except Exception as e:
                self.get_logger().error("CV Bridge error: %s" % str(e))

    def level_crossing_order(self, order_msg):
        pub_level_crossing_return = UInt8()
        if order_msg.data == self.StepOfLevelCrossing.pass_level.value:
            while rclpy.ok():
                is_level_detected, _, _ = self.find_level()
                rclpy.spin_once(self, timeout_sec=0.01)
                if is_level_detected:
                    self.get_logger().info("Level Detected")
                    max_vel_msg = Float64()
                    max_vel_msg.data = 0.03
                    self.pub_max_vel.publish(max_vel_msg)
                    break

            while rclpy.ok():
                _, is_level_close, _ = self.find_level()
                rclpy.spin_once(self, timeout_sec=0.01)
                if is_level_close:
                    self.get_logger().info("STOP")
                    max_vel_msg = Float64()
                    max_vel_msg.data = 0.0
                    self.pub_max_vel.publish(max_vel_msg)
                    break

            while rclpy.ok():
                _, _, is_level_opened = self.find_level()
                rclpy.spin_once(self, timeout_sec=0.01)
                if is_level_opened:
                    self.get_logger().info("GO")
                    max_vel_msg = Float64()
                    max_vel_msg.data = 0.05
                    self.pub_max_vel.publish(max_vel_msg)
                    break

            pub_level_crossing_return.data = self.StepOfLevelCrossing.exit.value

        self.pub_level_crossing_return.publish(pub_level_crossing_return)
        self.get_logger().info(pub_level_crossing_return.data)
        time.sleep(3.0)

    def find_level(self):
        mask = self.mask_red_of_level()
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        return self.find_rect_of_level(mask)

    def mask_red_of_level(self):
        if self.cv_image is None:
            return None
        image = self.cv_image.copy()
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_red = np.array([self.hue_red_l, self.saturation_red_l, self.lightness_red_l])
        upper_red = np.array([self.hue_red_h, self.saturation_red_h, self.lightness_red_h])
        mask = cv2.inRange(hsv, lower_red, upper_red)

        if self.is_calibration_mode:
            if self.pub_image_type == "compressed":
                comp_img_msg = self.cv_bridge.cv2_to_compressed_imgmsg(mask, dst_format="jpg")
                self.pub_image_color_filtered.publish(comp_img_msg)
            else:
                img_msg = self.cv_bridge.cv2_to_imgmsg(mask, encoding="mono8")
                self.pub_image_color_filtered.publish(img_msg)

        mask = cv2.bitwise_not(mask)
        return mask

    def find_rect_of_level(self, mask):
        is_level_detected = False
        is_level_close = False
        is_level_opened = False

        params = cv2.SimpleBlobDetector_Params()
        params.minThreshold = 0
        params.maxThreshold = 255
        params.filterByArea = True
        params.minArea = 250
        params.maxArea = 2000
        params.filterByCircularity = True
        params.minCircularity = 0.3
        params.filterByConvexity = True
        params.minConvexity = 0.9

        # create blob detector
        detector = cv2.SimpleBlobDetector_create(params)
        keypts = detector.detect(mask)
        frame = cv2.drawKeypoints(
            self.cv_image, keypts,
            np.array([]),
            (0, 255, 255),
            cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
            )

        mean_x = 0.0
        mean_y = 0.0

        if len(keypts) == 3:
            for kp in keypts:
                mean_x += kp.pt[0] / 3
                mean_y += kp.pt[1] / 3

            arr_distances = [
                fnCalcDistanceDot2Dot(mean_x, mean_y, kp.pt[0], kp.pt[1]) for kp in keypts
                ]
            idx_order = fnArrangeIndexOfPoint(arr_distances)

            frame = cv2.line(
                frame,
                (int(keypts[idx_order[0]].pt[0]), int(keypts[idx_order[0]].pt[1])),
                (int(keypts[idx_order[1]].pt[0]), int(keypts[idx_order[1]].pt[1])),
                (255, 0, 0), 5)
            frame = cv2.circle(frame, (int(mean_x), int(mean_y)), 5, (255, 255, 0), -1)

            point1 = [int(keypts[idx_order[0]].pt[0]), int(keypts[idx_order[0]].pt[1] - 1)]
            point2 = [int(keypts[idx_order[2]].pt[0]), int(keypts[idx_order[2]].pt[1] - 1)]
            point3 = [int(keypts[idx_order[1]].pt[0]), int(keypts[idx_order[1]].pt[1] - 1)]

            dx = point3[0] - point1[0]
            dy = point3[1] - point1[1]
            if dx == 0:
                slope = float('inf')
            else:
                slope = dy / dx

            # Determine the orientation of points based on slope
            if dx == 0 or abs(slope) > 2.0:
                is_level_opened = True
                self.stop_bar_state = 'go'
                self.get_logger().info(self.stop_bar_state)
            else:
                # Otherwise, linearity and distance checks are performed with existing logic
                is_rects_linear = fnCheckLinearity(point1, point2, point3)
                is_rects_dist_equal = fnCheckDistanceIsEqual(point1, point2, point3)

                if is_rects_linear or is_rects_dist_equal:
                    distance_bar2car = 25 / fnCalcDistanceDot2Dot(
                        point1[0], point1[1], point2[0], point2[1])
                    self.stop_bar_count = 50
                    if distance_bar2car > 1.0:
                        is_level_detected = True
                        self.stop_bar_state = 'slowdown'
                        self.get_logger().info(self.stop_bar_state)
                    else:
                        is_level_close = True
                        self.stop_bar_state = 'stop'
                        self.get_logger().info(self.stop_bar_state)

        elif len(keypts) <= 1:
            is_level_opened = True
            self.stop_bar_state = 'go'

        if self.pub_image_type == "compressed":
            comp_img_msg = self.cv_bridge.cv2_to_compressed_imgmsg(frame, dst_format="jpg")
            self.pub_image_level.publish(comp_img_msg)
        else:
            img_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.pub_image_level.publish(img_msg)

        return is_level_detected, is_level_close, is_level_opened


def main(args=None):
    rclpy.init(args=args)
    node = DetectLevelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
