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

# Author: Leon Jung, [AuTURBO] Ki Hoon Kim (https://github.com/auturbo), Gilbert

import cv2
from cv_bridge import CvBridge
import numpy as np
from rcl_interfaces.msg import IntegerRange
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image


class ImageProjection(Node):

    def __init__(self):
        super().__init__('image_projection')
        parameter_descriptor_top = ParameterDescriptor(
            description='projection range top.',
            integer_range=[
                IntegerRange(
                    from_value=0,
                    to_value=120,
                    step=1)]
        )
        parameter_descriptor_bottom = ParameterDescriptor(
            description='projection range top.',
            integer_range=[
                IntegerRange(
                    from_value=0,
                    to_value=320,
                    step=1)]
        )

        self.declare_parameters(
            namespace='',
            parameters=[
                ('camera.extrinsic_camera_calibration.top_x', 72, parameter_descriptor_top),
                ('camera.extrinsic_camera_calibration.top_y', 4, parameter_descriptor_top),
                ('camera.extrinsic_camera_calibration.bottom_x', 259, parameter_descriptor_bottom),
                ('camera.extrinsic_camera_calibration.bottom_y', 159, parameter_descriptor_bottom),
                ('is_extrinsic_camera_calibration_mode', False)
            ]
        )

        self.top_x = self.get_parameter(
            'camera.extrinsic_camera_calibration.top_x').get_parameter_value().integer_value
        self.top_y = self.get_parameter(
            'camera.extrinsic_camera_calibration.top_y').get_parameter_value().integer_value
        self.bottom_x = self.get_parameter(
            'camera.extrinsic_camera_calibration.bottom_x').get_parameter_value().integer_value
        self.bottom_y = self.get_parameter(
            'camera.extrinsic_camera_calibration.bottom_y').get_parameter_value().integer_value

        self.is_calibration_mode = self.get_parameter(
            'is_extrinsic_camera_calibration_mode').get_parameter_value().bool_value

        if self.is_calibration_mode:
            self.add_on_set_parameters_callback(self.cbGetImageProjectionParam)

        self.sub_image_type = 'compressed'  # you can choose image type 'compressed', 'raw'
        self.pub_image_type = 'raw'  # you can choose image type 'compressed', 'raw'

        if self.sub_image_type == 'compressed':
            self.sub_image_original = self.create_subscription(
                CompressedImage,
                '/camera/image_input/compressed',
                self.cbImageProjection,
                1
            )
        elif self.sub_image_type == 'raw':
            self.sub_image_original = self.create_subscription(
                Image,
                '/camera/image_input',
                self.cbImageProjection,
                1
            )

        if self.pub_image_type == 'compressed':
            self.pub_image_projected = self.create_publisher(
                CompressedImage,
                '/camera/image_output/compressed',
                1
            )
        elif self.pub_image_type == 'raw':
            self.pub_image_projected = self.create_publisher(Image, '/camera/image_output', 1)

        if self.is_calibration_mode:
            if self.pub_image_type == 'compressed':
                self.pub_image_calib = self.create_publisher(
                    CompressedImage,
                    '/camera/image_calib/compressed',
                    1
                )
            elif self.pub_image_type == 'raw':
                self.pub_image_calib = self.create_publisher(
                    Image,
                    '/camera/image_calib',
                    1
                )

        self.cvBridge = CvBridge()

    def cbGetImageProjectionParam(self, parameters):
        for param in parameters:
            self.get_logger().info(f'Parameter name: {param.name}')
            self.get_logger().info(f'Parameter value: {param.value}')
            self.get_logger().info(f'Parameter type: {param.type_}')
            if param.name == 'camera.extrinsic_camera_calibration.top_x':
                self.top_x = param.value
            if param.name == 'camera.extrinsic_camera_calibration.top_y':
                self.top_y = param.value
            if param.name == 'camera.extrinsic_camera_calibration.bottom_x':
                self.bottom_x = param.value
            if param.name == 'camera.extrinsic_camera_calibration.bottom_y':
                self.bottom_y = param.value
        self.get_logger().info(f'change: {self.top_x}')
        self.get_logger().info(f'change: {self.top_y}')
        self.get_logger().info(f'change: {self.bottom_x}')
        self.get_logger().info(f'change: {self.bottom_y}')
        return SetParametersResult(successful=True)

    def cbImageProjection(self, msg_img):
        if self.sub_image_type == 'compressed':
            np_image_original = np.frombuffer(msg_img.data, np.uint8)
            cv_image_original = cv2.imdecode(np_image_original, cv2.IMREAD_COLOR)
        elif self.sub_image_type == 'raw':
            cv_image_original = self.cvBridge.imgmsg_to_cv2(msg_img, 'bgr8')

        # setting homography variables
        top_x = self.top_x
        top_y = self.top_y
        bottom_x = self.bottom_x
        bottom_y = self.bottom_y

        if self.is_calibration_mode:
            # copy original image to use for calibration
            cv_image_calib = np.copy(cv_image_original)

            # draw lines to help setting homography variables
            cv_image_calib = cv2.line(
                cv_image_calib,
                (160 - top_x, 180 - top_y),
                (160 + top_x, 180 - top_y),
                (0, 0, 255),
                1
            )
            cv_image_calib = cv2.line(
                cv_image_calib,
                (160 - bottom_x, 120 + bottom_y),
                (160 + bottom_x, 120 + bottom_y),
                (0, 0, 255),
                1
            )
            cv_image_calib = cv2.line(
                cv_image_calib,
                (160 + bottom_x, 120 + bottom_y),
                (160 + top_x, 180 - top_y),
                (0, 0, 255),
                1
            )
            cv_image_calib = cv2.line(
                cv_image_calib,
                (160 - bottom_x, 120 + bottom_y),
                (160 - top_x, 180 - top_y),
                (0, 0, 255),
                1
            )

            if self.pub_image_type == 'compressed':
                self.pub_image_calib.publish(
                    self.cvBridge.cv2_to_compressed_imgmsg(
                        cv_image_calib,
                        'jpg'))
            elif self.pub_image_type == 'raw':
                self.pub_image_calib.publish(self.cvBridge.cv2_to_imgmsg(cv_image_calib, 'bgr8'))

        # adding Gaussian blur to the image of original
        cv_image_original = cv2.GaussianBlur(cv_image_original, (5, 5), 0)

        # homography transform process
        # selecting 4 points from the original image
        pts_src = np.array([
            [160 - top_x, 180 - top_y],
            [160 + top_x, 180 - top_y],
            [160 + bottom_x, 120 + bottom_y],
            [160 - bottom_x, 120 + bottom_y]
        ])

        # selecting 4 points from image that will be transformed
        pts_dst = np.array([[200, 0], [800, 0], [800, 600], [200, 600]])

        # finding homography matrix
        h, status = cv2.findHomography(pts_src, pts_dst)

        # homography process
        cv_image_homography = cv2.warpPerspective(cv_image_original, h, (1000, 600))

        # fill the empty space with black triangles on left and right side of bottom
        triangle1 = np.array([[0, 599], [0, 340], [200, 599]], np.int32)
        triangle2 = np.array([[999, 599], [999, 340], [799, 599]], np.int32)
        black = (0, 0, 0)
        cv_image_homography = cv2.fillPoly(cv_image_homography, [triangle1, triangle2], black)

        if self.pub_image_type == 'compressed':
            self.pub_image_projected.publish(
                self.cvBridge.cv2_to_compressed_imgmsg(
                    cv_image_homography, 'jpg'
                )
            )
        elif self.pub_image_type == 'raw':
            self.pub_image_projected.publish(
                self.cvBridge.cv2_to_imgmsg(
                    cv_image_homography, 'bgr8'
                )
            )


def main(args=None):
    rclpy.init(args=args)
    node = ImageProjection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
