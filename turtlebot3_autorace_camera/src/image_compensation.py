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

# Author: Leon Jung

import cv2
from cv_bridge import CvBridge
import numpy as np
from rcl_interfaces.msg import FloatingPointRange
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image


class ImageCompensation(Node):

    def __init__(self):
        super().__init__('image_compensation')
        parameter_descriptor_clip_hist = ParameterDescriptor(
            description='clip hist range.',
            floating_point_range=[FloatingPointRange(
                from_value=0.0,
                to_value=10.0,
                step=0.1)]
        )

        self.declare_parameters(
            namespace='',
            parameters=[
                (
                    'camera.extrinsic_camera_calibration.clip_hist_percent',
                    1.0,
                    parameter_descriptor_clip_hist
                ),
                ('is_extrinsic_camera_calibration_mode', False)
            ]
        )

        self.clip_hist_percent = self.get_parameter(
            'camera.extrinsic_camera_calibration.clip_hist_percent'
        ).value
        self.is_calibration_mode = self.get_parameter('is_extrinsic_camera_calibration_mode').value

        if self.is_calibration_mode:
            self.add_on_set_parameters_callback(self.param_update_callback)

        self.sub_image_type = 'compressed'  # 'compressed' / 'raw'
        self.pub_image_type = 'raw'  # 'compressed' / 'raw'

        if self.sub_image_type == 'compressed':
            self.sub_image_original = self.create_subscription(
                CompressedImage,
                '/camera/image_input/compressed',
                self.cbImageCompensation,
                10
            )
        elif self.sub_image_type == 'raw':
            self.sub_image_original = self.create_subscription(
                Image,
                '/camera/image_input',
                self.cbImageCompensation,
                10
            )

        if self.pub_image_type == 'compressed':
            self.pub_image_compensated = self.create_publisher(
                CompressedImage,
                '/camera/image_output/compressed',
                10
            )
        elif self.pub_image_type == 'raw':
            self.pub_image_compensated = self.create_publisher(
                Image,
                '/camera/image_output',
                10
            )

        self.cvBridge = CvBridge()

    def param_update_callback(self, parameters):
        for param in parameters:
            self.get_logger().info(f'Parameter name: {param.name}')
            self.get_logger().info(f'Parameter value: {param.value}')
            self.get_logger().info(f'Parameter type: {param.type_}')
            if param.name == 'camera.extrinsic_camera_calibration.clip_hist_percent':
                self.clip_hist_percent = param.value
        self.get_logger().info(f'change: {self.clip_hist_percent}')
        return SetParametersResult(successful=True)

    def cbImageCompensation(self, msg_img):
        if self.sub_image_type == 'compressed':
            np_image_original = np.frombuffer(msg_img.data, np.uint8)
            cv_image_original = cv2.imdecode(np_image_original, cv2.IMREAD_COLOR)
        elif self.sub_image_type == 'raw':
            cv_image_original = self.cvBridge.imgmsg_to_cv2(msg_img, 'bgr8')

        cv_image_compensated = np.copy(cv_image_original)

        clip_hist_percent = self.clip_hist_percent
        hist_size = 256
        min_gray = 0
        max_gray = 0
        alpha = 0
        beta = 0

        gray = cv2.cvtColor(cv_image_compensated, cv2.COLOR_BGR2GRAY)

        if clip_hist_percent == 0.0:
            min_gray, max_gray, _, _ = cv2.minMaxLoc(gray)
        else:
            hist = cv2.calcHist([gray], [0], None, [hist_size], [0, hist_size])

            accumulator = np.cumsum(hist)

            total_max = accumulator[hist_size - 1]

            clip_hist_percent *= (total_max / 100.)
            clip_hist_percent /= 2.

            min_gray = 0
            while accumulator[min_gray] < clip_hist_percent:
                min_gray += 1

            max_gray = hist_size - 1
            while accumulator[max_gray] >= (total_max - clip_hist_percent):
                max_gray -= 1

        input_range = max_gray - min_gray

        alpha = (hist_size - 1) / input_range
        beta = -min_gray * alpha

        cv_image_compensated = cv2.convertScaleAbs(cv_image_compensated, -1, alpha, beta)

        if self.pub_image_type == 'compressed':
            self.pub_image_compensated.publish(
                self.cvBridge.cv2_to_compressed_imgmsg(
                    cv_image_compensated, 'jpg'
                )
            )
        elif self.pub_image_type == 'raw':
            self.pub_image_compensated.publish(
                self.cvBridge.cv2_to_imgmsg(
                    cv_image_compensated, 'bgr8'
                )
            )


def main(args=None):
    rclpy.init(args=args)
    node = ImageCompensation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
