#!/usr/bin/env python3
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

# Author: Leon Jung, Gilbert, Ashe Kim, ChanHyeong Lee

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
from rcl_interfaces.msg import IntegerRange
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image


class DetectTrafficLight(Node):
    def __init__(self):
        super().__init__('detect_traffic_light')

        hue_descriptor = ParameterDescriptor(
            integer_range=[IntegerRange(from_value=0, to_value=179, step=1)],
            description='Hue Value (0~179)'
        )
        sat_descriptor = ParameterDescriptor(
            integer_range=[IntegerRange(from_value=0, to_value=255, step=1)],
            description='Saturation Value (0~255)'
        )
        light_descriptor = ParameterDescriptor(
            integer_range=[IntegerRange(from_value=0, to_value=255, step=1)],
            description='Lightness Value (0~255)'
        )

        self.declare_parameter('hue_red_l', 0, hue_descriptor)
        self.declare_parameter('hue_red_h', 26, hue_descriptor)
        self.declare_parameter('saturation_red_l', 239, sat_descriptor)
        self.declare_parameter('saturation_red_h', 255, sat_descriptor)
        self.declare_parameter('lightness_red_l', 123, light_descriptor)
        self.declare_parameter('lightness_red_h', 250, light_descriptor)

        self.declare_parameter('hue_yellow_l', 19, hue_descriptor)
        self.declare_parameter('hue_yellow_h', 33, hue_descriptor)
        self.declare_parameter('saturation_yellow_l', 237, sat_descriptor)
        self.declare_parameter('saturation_yellow_h', 255, sat_descriptor)
        self.declare_parameter('lightness_yellow_l', 231, light_descriptor)
        self.declare_parameter('lightness_yellow_h', 255, light_descriptor)

        self.declare_parameter('hue_green_l', 40, hue_descriptor)
        self.declare_parameter('hue_green_h', 113, hue_descriptor)
        self.declare_parameter('saturation_green_l', 210, sat_descriptor)
        self.declare_parameter('saturation_green_h', 255, sat_descriptor)
        self.declare_parameter('lightness_green_l', 131, light_descriptor)
        self.declare_parameter('lightness_green_h', 255, light_descriptor)

        self.declare_parameter('is_calibration_mode', False)

        self.hue_red_l = self.get_parameter(
            'hue_red_l').get_parameter_value().integer_value
        self.hue_red_h = self.get_parameter(
            'hue_red_h').get_parameter_value().integer_value
        self.saturation_red_l = self.get_parameter(
            'saturation_red_l').get_parameter_value().integer_value
        self.saturation_red_h = self.get_parameter(
            'saturation_red_h').get_parameter_value().integer_value
        self.lightness_red_l = self.get_parameter(
            'lightness_red_l').get_parameter_value().integer_value
        self.lightness_red_h = self.get_parameter(
            'lightness_red_h').get_parameter_value().integer_value

        self.hue_yellow_l = self.get_parameter(
            'hue_yellow_l').get_parameter_value().integer_value
        self.hue_yellow_h = self.get_parameter(
            'hue_yellow_h').get_parameter_value().integer_value
        self.saturation_yellow_l = self.get_parameter(
            'saturation_yellow_l').get_parameter_value().integer_value
        self.saturation_yellow_h = self.get_parameter(
            'saturation_yellow_h').get_parameter_value().integer_value
        self.lightness_yellow_l = self.get_parameter(
            'lightness_yellow_l').get_parameter_value().integer_value
        self.lightness_yellow_h = self.get_parameter(
            'lightness_yellow_h').get_parameter_value().integer_value

        self.hue_green_l = self.get_parameter(
            'hue_green_l').get_parameter_value().integer_value
        self.hue_green_h = self.get_parameter(
            'hue_green_h').get_parameter_value().integer_value
        self.saturation_green_l = self.get_parameter(
            'saturation_green_l').get_parameter_value().integer_value
        self.saturation_green_h = self.get_parameter(
            'saturation_green_h').get_parameter_value().integer_value
        self.lightness_green_l = self.get_parameter(
            'lightness_green_l').get_parameter_value().integer_value
        self.lightness_green_h = self.get_parameter(
            'lightness_green_h').get_parameter_value().integer_value

        self.is_calibration_mode = self.get_parameter(
            'is_calibration_mode').get_parameter_value().bool_value
        if self.is_calibration_mode:
            self.add_on_set_parameters_callback(self.get_detect_traffic_light_param)

        self.sub_image_type = "raw"
        self.pub_image_type = "compressed"

        self.counter = 1

        if self.sub_image_type == "compressed":
            self.sub_image_original = self.create_subscription(
                CompressedImage, '/detect/image_input/compressed', self.get_image, 1)
        else:
            self.sub_image_original = self.create_subscription(
                Image, '/detect/image_input', self.get_image, 1)

        if self.pub_image_type == "compressed":
            self.pub_image_traffic_light = self.create_publisher(
                CompressedImage, '/detect/image_output/compressed', 1)
        else:
            self.pub_image_traffic_light = self.create_publisher(
                Image, '/detect/image_output', 1)

        if self.is_calibration_mode:
            if self.pub_image_type == "compressed":
                self.pub_image_red_light = self.create_publisher(
                    CompressedImage, '/detect/image_output_sub1/compressed', 1)
                self.pub_image_yellow_light = self.create_publisher(
                    CompressedImage, '/detect/image_output_sub2/compressed', 1)
                self.pub_image_green_light = self.create_publisher(
                    CompressedImage, '/detect/image_output_sub3/compressed', 1)
            else:
                self.pub_image_red_light = self.create_publisher(
                    Image, '/detect/image_output_sub1', 1)
                self.pub_image_yellow_light = self.create_publisher(
                    Image, '/detect/image_output_sub2', 1)
                self.pub_image_green_light = self.create_publisher(
                    Image, '/detect/image_output_sub3', 1)

        self.cvBridge = CvBridge()
        self.cv_image = None

        self.is_image_available = False
        self.is_traffic_light_finished = False

        self.green_count = 0
        self.yellow_count = 0
        self.red_count = 0
        self.stop_count = 0
        self.off_traffic = False

        time.sleep(1)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def get_detect_traffic_light_param(self, params):
        for param in params:
            if param.name == 'hue_red_l':
                self.hue_red_l = param.value
                self.get_logger().info(f"hue_red_l set to: {param.value}")
            elif param.name == 'hue_red_h':
                self.hue_red_h = param.value
                self.get_logger().info(f"hue_red_h set to: {param.value}")
            elif param.name == 'saturation_red_l':
                self.saturation_red_l = param.value
                self.get_logger().info(f"saturation_red_l set to: {param.value}")
            elif param.name == 'saturation_red_h':
                self.saturation_red_h = param.value
                self.get_logger().info(f"saturation_red_h set to: {param.value}")
            elif param.name == 'lightness_red_l':
                self.lightness_red_l = param.value
                self.get_logger().info(f"lightness_red_l set to: {param.value}")
            elif param.name == 'lightness_red_h':
                self.lightness_red_h = param.value
                self.get_logger().info(f"lightness_red_h set to: {param.value}")
            elif param.name == 'hue_yellow_l':
                self.hue_yellow_l = param.value
                self.get_logger().info(f"hue_yellow_l set to: {param.value}")
            elif param.name == 'hue_yellow_h':
                self.hue_yellow_h = param.value
                self.get_logger().info(f"hue_yellow_h set to: {param.value}")
            elif param.name == 'saturation_yellow_l':
                self.saturation_yellow_l = param.value
                self.get_logger().info(f"saturation_yellow_l set to: {param.value}")
            elif param.name == 'saturation_yellow_h':
                self.saturation_yellow_h = param.value
                self.get_logger().info(f"saturation_yellow_h set to: {param.value}")
            elif param.name == 'lightness_yellow_l':
                self.lightness_yellow_l = param.value
                self.get_logger().info(f"lightness_yellow_l set to: {param.value}")
            elif param.name == 'lightness_yellow_h':
                self.lightness_yellow_h = param.value
                self.get_logger().info(f"lightness_yellow_h set to: {param.value}")
            elif param.name == 'hue_green_l':
                self.hue_green_l = param.value
                self.get_logger().info(f"hue_green_l set to: {param.value}")
            elif param.name == 'hue_green_h':
                self.hue_green_h = param.value
                self.get_logger().info(f"hue_green_h set to: {param.value}")
            elif param.name == 'saturation_green_l':
                self.saturation_green_l = param.value
                self.get_logger().info(f"saturation_green_l set to: {param.value}")
            elif param.name == 'saturation_green_h':
                self.saturation_green_h = param.value
                self.get_logger().info(f"saturation_green_h set to: {param.value}")
            elif param.name == 'lightness_green_l':
                self.lightness_green_l = param.value
                self.get_logger().info(f"lightness_green_l set to: {param.value}")
            elif param.name == 'lightness_green_h':
                self.lightness_green_h = param.value
                self.get_logger().info(f"lightness_green_h set to: {param.value}")
        return SetParametersResult(successful=True)

    def get_image(self, image_msg):
        # Processing every 3 frames to reduce frame processing load
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
                self.cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")
            except CvBridgeError as e:
                self.get_logger().error(f"CvBridge Error: {e}")
                return

        self.is_image_available = True

    def timer_callback(self):
        if self.is_image_available and not self.is_traffic_light_finished:
            self.find_traffic_light()

    def find_traffic_light(self):
        cv_image_mask = self.mask_green_traffic_light()
        cv_image_mask = cv2.GaussianBlur(cv_image_mask, (5, 5), 0)

        status1 = self.find_circle_of_traffic_light(cv_image_mask, 'green')
        if status1 == 1 or status1 == 5:
            self.get_logger().info("detect GREEN")
            self.stop_count = 0
            self.green_count += 1
        else:
            self.green_count = 0
            cv_image_mask = self.mask_yellow_traffic_light()
            cv_image_mask = cv2.GaussianBlur(cv_image_mask, (5, 5), 0)
            status2 = self.find_circle_of_traffic_light(cv_image_mask, 'yellow')
            if status2 == 2:
                self.get_logger().info("detect YELLOW")
                self.yellow_count += 1
            else:
                self.yellow_count = 0
                cv_image_mask = self.mask_red_traffic_light()
                cv_image_mask = cv2.GaussianBlur(cv_image_mask, (5, 5), 0)
                status3 = self.find_circle_of_traffic_light(cv_image_mask, 'red')
                if status3 == 3:
                    self.get_logger().info("detect RED")
                    self.red_count += 1
                elif status3 == 4:
                    self.red_count = 0
                    self.stop_count += 1
                else:
                    self.red_count = 0
                    self.stop_count = 0

        if self.green_count >= 3:
            self.get_logger().info("GREEN")
            cv2.putText(self.cv_image, "GREEN", (self.point_col, self.point_low),
                        cv2.FONT_HERSHEY_DUPLEX, 0.5, (80, 255, 0))

        if self.yellow_count >= 3:
            self.get_logger().info("YELLOW")
            cv2.putText(self.cv_image, "YELLOW", (self.point_col, self.point_low),
                        cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, 255))

        if self.red_count >= 3:
            self.get_logger().info("RED")
            cv2.putText(self.cv_image, "RED", (self.point_col, self.point_low),
                        cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 0, 255))

        if self.stop_count >= 8:
            self.get_logger().info("STOP")
            self.off_traffic = True
            cv2.putText(self.cv_image, "STOP", (self.point_col, self.point_low),
                        cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 0, 255))

        if self.pub_image_type == "compressed":
            self.pub_image_traffic_light.publish(
                self.cvBridge.cv2_to_compressed_imgmsg(self.cv_image, "jpg"))
        else:
            self.pub_image_traffic_light.publish(
                self.cvBridge.cv2_to_imgmsg(self.cv_image, "bgr8"))

    def mask_red_traffic_light(self):
        image = np.copy(self.cv_image)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_red = np.array([self.hue_red_l, self.saturation_red_l, self.lightness_red_l])
        upper_red = np.array([self.hue_red_h, self.saturation_red_h, self.lightness_red_h])

        mask = cv2.inRange(hsv, lower_red, upper_red)

        if self.is_calibration_mode:
            if self.pub_image_type == "compressed":
                self.pub_image_red_light.publish(
                    self.cvBridge.cv2_to_compressed_imgmsg(mask, "jpg"))
            else:
                self.pub_image_red_light.publish(
                    self.cvBridge.cv2_to_imgmsg(mask, "mono8"))

        mask = cv2.bitwise_not(mask)
        return mask

    def mask_yellow_traffic_light(self):
        image = np.copy(self.cv_image)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array(
            [self.hue_yellow_l, self.saturation_yellow_l, self.lightness_yellow_l]
            )
        upper_yellow = np.array(
            [self.hue_yellow_h, self.saturation_yellow_h, self.lightness_yellow_h]
            )

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        if self.is_calibration_mode:
            if self.pub_image_type == "compressed":
                self.pub_image_yellow_light.publish(
                    self.cvBridge.cv2_to_compressed_imgmsg(mask, "jpg"))
            else:
                self.pub_image_yellow_light.publish(
                    self.cvBridge.cv2_to_imgmsg(mask, "mono8"))

        mask = cv2.bitwise_not(mask)
        return mask

    def mask_green_traffic_light(self):
        image = np.copy(self.cv_image)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_green = np.array([self.hue_green_l, self.saturation_green_l, self.lightness_green_l])
        upper_green = np.array([self.hue_green_h, self.saturation_green_h, self.lightness_green_h])

        mask = cv2.inRange(hsv, lower_green, upper_green)

        if self.is_calibration_mode:
            if self.pub_image_type == "compressed":
                self.pub_image_green_light.publish(
                    self.cvBridge.cv2_to_compressed_imgmsg(mask, "jpg"))
            else:
                self.pub_image_green_light.publish(
                    self.cvBridge.cv2_to_imgmsg(mask, "mono8"))

        mask = cv2.bitwise_not(mask)
        return mask

    def find_circle_of_traffic_light(self, mask, find_color):
        status = 0
        params = cv2.SimpleBlobDetector_Params()
        params.minThreshold = 0
        params.maxThreshold = 255
        params.filterByArea = True
        params.minArea = 50
        params.maxArea = 600
        params.filterByCircularity = True
        params.minCircularity = 0.4
        params.filterByConvexity = True
        params.minConvexity = 0.6

        detector = cv2.SimpleBlobDetector_create(params)
        keypts = detector.detect(mask)

        col1 = 180
        col2 = 270
        col3 = 305
        low1 = 50
        low2 = 170
        low3 = 270

        for i in range(len(keypts)):
            self.point_col = int(keypts[i].pt[0])
            self.point_low = int(keypts[i].pt[1])
            if col1 < self.point_col < col2 and low1 < self.point_low < low2:
                if find_color == 'green':
                    status = 1
                elif find_color == 'yellow':
                    status = 2
                elif find_color == 'red':
                    status = 3
            elif col2 < self.point_col < col3 and low1 < self.point_low < low3:
                if find_color == 'red':
                    status = 4
                elif find_color == 'green':
                    status = 5
            else:
                status = 6

        return status


def main(args=None):
    rclpy.init(args=args)
    node = DetectTrafficLight()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
