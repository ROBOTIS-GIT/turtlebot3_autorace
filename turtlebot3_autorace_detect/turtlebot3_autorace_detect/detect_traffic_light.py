#!/usr/bin/env python3
#
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
#
# Author: Leon Jung, Gilbert, Ashe Kim, ChanHyeong Lee

import time

import cv2
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import numpy as np
from rcl_interfaces.msg import IntegerRange
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image


class DetectTrafficLight(Node):

    def __init__(self):
        super().__init__('detect_traffic_light')
        parameter_descriptor_hue = ParameterDescriptor(
            integer_range=[IntegerRange(from_value=0, to_value=179, step=1)],
            description='Hue Value (0~179)'
        )
        parameter_descriptor_saturation_lightness = ParameterDescriptor(
            integer_range=[IntegerRange(from_value=0, to_value=255, step=1)],
            description='Saturation/Lightness Value (0~255)'
        )

        self.declare_parameter(
            'red.hue_l', 0, parameter_descriptor_hue)
        self.declare_parameter(
            'red.hue_h', 179, parameter_descriptor_hue)
        self.declare_parameter(
            'red.saturation_l', 0, parameter_descriptor_saturation_lightness)
        self.declare_parameter(
            'red.saturation_h', 255, parameter_descriptor_saturation_lightness)
        self.declare_parameter(
            'red.lightness_l', 0, parameter_descriptor_saturation_lightness)
        self.declare_parameter(
            'red.lightness_h', 255, parameter_descriptor_saturation_lightness)

        self.declare_parameter(
            'yellow.hue_l', 0, parameter_descriptor_hue)
        self.declare_parameter(
            'yellow.hue_h', 179, parameter_descriptor_hue)
        self.declare_parameter(
            'yellow.saturation_l', 0, parameter_descriptor_saturation_lightness)
        self.declare_parameter(
            'yellow.saturation_h', 255, parameter_descriptor_saturation_lightness)
        self.declare_parameter(
            'yellow.lightness_l', 0, parameter_descriptor_saturation_lightness)
        self.declare_parameter(
            'yellow.lightness_h', 255, parameter_descriptor_saturation_lightness)

        self.declare_parameter(
            'green.hue_l', 0, parameter_descriptor_hue)
        self.declare_parameter(
            'green.hue_h', 179, parameter_descriptor_hue)
        self.declare_parameter(
            'green.saturation_l', 0, parameter_descriptor_saturation_lightness)
        self.declare_parameter(
            'green.saturation_h', 255, parameter_descriptor_saturation_lightness)
        self.declare_parameter(
            'green.lightness_l', 0, parameter_descriptor_saturation_lightness)
        self.declare_parameter(
            'green.lightness_h', 255, parameter_descriptor_saturation_lightness)

        self.declare_parameter('is_calibration_mode', False)

        self.hue_red_l = self.get_parameter(
            'red.hue_l').get_parameter_value().integer_value
        self.hue_red_h = self.get_parameter(
            'red.hue_h').get_parameter_value().integer_value
        self.saturation_red_l = self.get_parameter(
            'red.saturation_l').get_parameter_value().integer_value
        self.saturation_red_h = self.get_parameter(
            'red.saturation_h').get_parameter_value().integer_value
        self.lightness_red_l = self.get_parameter(
            'red.lightness_l').get_parameter_value().integer_value
        self.lightness_red_h = self.get_parameter(
            'red.lightness_h').get_parameter_value().integer_value

        self.hue_yellow_l = self.get_parameter(
            'yellow.hue_l').get_parameter_value().integer_value
        self.hue_yellow_h = self.get_parameter(
            'yellow.hue_h').get_parameter_value().integer_value
        self.saturation_yellow_l = self.get_parameter(
            'yellow.saturation_l').get_parameter_value().integer_value
        self.saturation_yellow_h = self.get_parameter(
            'yellow.saturation_h').get_parameter_value().integer_value
        self.lightness_yellow_l = self.get_parameter(
            'yellow.lightness_l').get_parameter_value().integer_value
        self.lightness_yellow_h = self.get_parameter(
            'yellow.lightness_h').get_parameter_value().integer_value

        self.hue_green_l = self.get_parameter(
            'green.hue_l').get_parameter_value().integer_value
        self.hue_green_h = self.get_parameter(
            'green.hue_h').get_parameter_value().integer_value
        self.saturation_green_l = self.get_parameter(
            'green.saturation_l').get_parameter_value().integer_value
        self.saturation_green_h = self.get_parameter(
            'green.saturation_h').get_parameter_value().integer_value
        self.lightness_green_l = self.get_parameter(
            'green.lightness_l').get_parameter_value().integer_value
        self.lightness_green_h = self.get_parameter(
            'green.lightness_h').get_parameter_value().integer_value

        self.is_calibration_mode = self.get_parameter(
            'is_calibration_mode').get_parameter_value().bool_value
        if self.is_calibration_mode:
            self.add_on_set_parameters_callback(self.get_detect_traffic_light_param)

        self.sub_image_type = 'raw'
        self.pub_image_type = 'compressed'

        self.counter = 1

        if self.sub_image_type == 'compressed':
            self.sub_image_original = self.create_subscription(
                CompressedImage, '/detect/image_input/compressed', self.get_image, 1)
        else:
            self.sub_image_original = self.create_subscription(
                Image, '/detect/image_input', self.get_image, 1)

        if self.pub_image_type == 'compressed':
            self.pub_image_traffic_light = self.create_publisher(
                CompressedImage, '/detect/image_output/compressed', 1)
        else:
            self.pub_image_traffic_light = self.create_publisher(
                Image, '/detect/image_output', 1)

        if self.is_calibration_mode:
            if self.pub_image_type == 'compressed':
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

        self.status = 0
        self.green_count = 0
        self.yellow_count = 0
        self.red_count = 0
        self.stop_count = 0
        self.off_traffic = False

        time.sleep(1)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def get_detect_traffic_light_param(self, params):
        for param in params:
            if param.name == 'red.hue_l':
                self.hue_red_l = param.value
                self.get_logger().info(f'red.hue_l set to: {param.value}')
            elif param.name == 'red.hue_h':
                self.hue_red_h = param.value
                self.get_logger().info(f'red.hue_h set to: {param.value}')
            elif param.name == 'red.saturation_l':
                self.saturation_red_l = param.value
                self.get_logger().info(f'red.saturation_l set to: {param.value}')
            elif param.name == 'red.saturation_h':
                self.saturation_red_h = param.value
                self.get_logger().info(f'red.saturation_h set to: {param.value}')
            elif param.name == 'red.lightness_l':
                self.lightness_red_l = param.value
                self.get_logger().info(f'red.lightness_l set to: {param.value}')
            elif param.name == 'red.lightness_h':
                self.lightness_red_h = param.value
                self.get_logger().info(f'red.lightness_h set to: {param.value}')
            elif param.name == 'yellow.hue_l':
                self.hue_yellow_l = param.value
                self.get_logger().info(f'yellow.hue_l set to: {param.value}')
            elif param.name == 'yellow.hue_h':
                self.hue_yellow_h = param.value
                self.get_logger().info(f'yellow.hue_h set to: {param.value}')
            elif param.name == 'yellow.saturation_l':
                self.saturation_yellow_l = param.value
                self.get_logger().info(f'yellow.saturation_l set to: {param.value}')
            elif param.name == 'yellow.saturation_h':
                self.saturation_yellow_h = param.value
                self.get_logger().info(f'yellow.saturation_h set to: {param.value}')
            elif param.name == 'yellow.lightness_l':
                self.lightness_yellow_l = param.value
                self.get_logger().info(f'yellow.lightness_l set to: {param.value}')
            elif param.name == 'yellow.lightness_h':
                self.lightness_yellow_h = param.value
                self.get_logger().info(f'yellow.lightness_h set to: {param.value}')
            elif param.name == 'green.hue_l':
                self.hue_green_l = param.value
                self.get_logger().info(f'green.hue_l set to: {param.value}')
            elif param.name == 'green.hue_h':
                self.hue_green_h = param.value
                self.get_logger().info(f'green.hue_h set to: {param.value}')
            elif param.name == 'green.saturation_l':
                self.saturation_green_l = param.value
                self.get_logger().info(f'green.saturation_l set to: {param.value}')
            elif param.name == 'green.saturation_h':
                self.saturation_green_h = param.value
                self.get_logger().info(f'green.saturation_h set to: {param.value}')
            elif param.name == 'green.lightness_l':
                self.lightness_green_l = param.value
                self.get_logger().info(f'green.lightness_l set to: {param.value}')
            elif param.name == 'green.lightness_h':
                self.lightness_green_h = param.value
                self.get_logger().info(f'green.lightness_h set to: {param.value}')
        return SetParametersResult(successful=True)

    def get_image(self, image_msg):
        # Processing every 3 frames to reduce frame processing load
        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1

        if self.sub_image_type == 'compressed':
            np_arr = np.frombuffer(image_msg.data, np.uint8)
            self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            try:
                self.cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, 'bgr8')
            except CvBridgeError as e:
                self.get_logger().error(f'CvBridge Error: {e}')
                return

        self.is_image_available = True

    def timer_callback(self):
        if self.is_image_available:
            self.find_traffic_light()

    def find_traffic_light(self):
        cv_image_mask_red = self.mask_red_traffic_light()
        cv_image_mask_red = cv2.GaussianBlur(cv_image_mask_red, (5, 5), 0)
        detect_red = self.find_circle_of_traffic_light(cv_image_mask_red, 'red')
        if detect_red:
            cv2.putText(self.cv_image, 'RED', (self.point_x, self.point_y),
                        cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 0, 255))

        cv_image_mask_yellow = self.mask_yellow_traffic_light()
        cv_image_mask_yellow = cv2.GaussianBlur(cv_image_mask_yellow, (5, 5), 0)
        detect_yellow = self.find_circle_of_traffic_light(cv_image_mask_yellow, 'yellow')
        if detect_yellow:
            cv2.putText(self.cv_image, 'YELLOW', (self.point_x, self.point_y),
                        cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, 255))

        cv_image_mask_green = self.mask_green_traffic_light()
        cv_image_mask_green = cv2.GaussianBlur(cv_image_mask_green, (5, 5), 0)
        detect_green = self.find_circle_of_traffic_light(cv_image_mask_green, 'green')
        if detect_green:
            cv2.putText(self.cv_image, 'GREEN', (self.point_x, self.point_y),
                        cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, 0))

        if self.pub_image_type == 'compressed':
            self.pub_image_traffic_light.publish(
                self.cvBridge.cv2_to_compressed_imgmsg(self.cv_image, 'jpg'))
        else:
            self.pub_image_traffic_light.publish(
                self.cvBridge.cv2_to_imgmsg(self.cv_image, 'bgr8'))

    def mask_red_traffic_light(self):
        image = np.copy(self.cv_image)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_red = np.array([self.hue_red_l, self.saturation_red_l, self.lightness_red_l])
        upper_red = np.array([self.hue_red_h, self.saturation_red_h, self.lightness_red_h])

        mask = cv2.inRange(hsv, lower_red, upper_red)

        if self.is_calibration_mode:
            if self.pub_image_type == 'compressed':
                self.pub_image_red_light.publish(
                    self.cvBridge.cv2_to_compressed_imgmsg(mask, 'jpg'))
            else:
                self.pub_image_red_light.publish(
                    self.cvBridge.cv2_to_imgmsg(mask, 'mono8'))

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
            if self.pub_image_type == 'compressed':
                self.pub_image_yellow_light.publish(
                    self.cvBridge.cv2_to_compressed_imgmsg(mask, 'jpg'))
            else:
                self.pub_image_yellow_light.publish(
                    self.cvBridge.cv2_to_imgmsg(mask, 'mono8'))

        mask = cv2.bitwise_not(mask)
        return mask

    def mask_green_traffic_light(self):
        image = np.copy(self.cv_image)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_green = np.array([self.hue_green_l, self.saturation_green_l, self.lightness_green_l])
        upper_green = np.array([self.hue_green_h, self.saturation_green_h, self.lightness_green_h])

        mask = cv2.inRange(hsv, lower_green, upper_green)

        if self.is_calibration_mode:
            if self.pub_image_type == 'compressed':
                self.pub_image_green_light.publish(
                    self.cvBridge.cv2_to_compressed_imgmsg(mask, 'jpg'))
            else:
                self.pub_image_green_light.publish(
                    self.cvBridge.cv2_to_imgmsg(mask, 'mono8'))

        mask = cv2.bitwise_not(mask)
        return mask

    def find_circle_of_traffic_light(self, mask, color):
        detect_result = False
        params = cv2.SimpleBlobDetector_Params()
        params.minThreshold = 0
        params.maxThreshold = 255
        params.filterByArea = True
        params.minArea = 50
        params.maxArea = 600
        params.filterByCircularity = True
        params.minCircularity = 0.5
        params.filterByConvexity = True
        params.minConvexity = 0.7

        detector = cv2.SimpleBlobDetector_create(params)
        keypts = detector.detect(mask)

        height, width = mask.shape[:2]
        roi_x_start = width // 2
        roi_x_end = width
        roi_y_start = height // 3
        roi_y_end = 2 * height // 3

        for i in range(len(keypts)):
            self.point_x = int(keypts[i].pt[0])
            self.point_y = int(keypts[i].pt[1])
            if roi_x_start < self.point_x < roi_x_end and roi_y_start < self.point_y < roi_y_end:
                detect_result = True
                self.get_logger().info(f'{color} light detected')
            else:
                detect_result = False

        return detect_result


def main(args=None):
    rclpy.init(args=args)
    node = DetectTrafficLight()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
