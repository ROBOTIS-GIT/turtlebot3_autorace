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
# Author: Leon Jung, Gilbert, Ashe Kim, Jun

from enum import Enum
import os

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8


class DetectSign(Node):

    def __init__(self):
        super().__init__('detect_sign')

        self.sub_image_type = 'raw'  # you can choose image type 'compressed', 'raw'
        self.pub_image_type = 'compressed'  # you can choose image type 'compressed', 'raw'

        if self.sub_image_type == 'compressed':
            self.sub_image_original = self.create_subscription(
                CompressedImage,
                '/detect/image_input/compressed',
                self.cbFindTrafficSign,
                10
            )
        elif self.sub_image_type == 'raw':
            self.sub_image_original = self.create_subscription(
                Image,
                '/detect/image_input',
                self.cbFindTrafficSign,
                10
            )

        self.pub_traffic_sign = self.create_publisher(UInt8, '/detect/traffic_sign', 10)
        if self.pub_image_type == 'compressed':
            self.pub_image_traffic_sign = self.create_publisher(
                CompressedImage,
                '/detect/image_output/compressed', 10
            )
        elif self.pub_image_type == 'raw':
            self.pub_image_traffic_sign = self.create_publisher(
                Image, '/detect/image_output', 10
            )

        self.cvBridge = CvBridge()
        self.TrafficSign = Enum('TrafficSign', 'parking')
        self.counter = 1

        self.fnPreproc()

        self.get_logger().info('DetectSign Node Initialized')

    def fnPreproc(self):
        # Initiate SIFT detector
        self.sift = cv2.SIFT_create()

        dir_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        dir_path = os.path.join(dir_path, 'image')

        self.img_parking = cv2.imread(dir_path + '/parking.png', 0)  # trainImage2
        self.kp_parking, self.des_parking = self.sift.detectAndCompute(self.img_parking, None)

        FLANN_INDEX_KDTREE = 0
        index_params = {
            'algorithm': FLANN_INDEX_KDTREE,
            'trees': 5
        }

        search_params = {
            'checks': 50
        }

        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

    def fnCalcMSE(self, arr1, arr2):
        squared_diff = (arr1 - arr2) ** 2
        total_sum = np.sum(squared_diff)
        num_all = arr1.shape[0] * arr1.shape[1]  # cv_image_input and 2 should have same shape
        err = total_sum / num_all
        return err

    def cbFindTrafficSign(self, image_msg):
        # drop the frame to 1/5 (6fps) because of the processing speed.
        # This is up to your computer's operating power.
        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1

        if self.sub_image_type == 'compressed':
            # converting compressed image to opencv image
            np_arr = np.frombuffer(image_msg.data, np.uint8)
            cv_image_input = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif self.sub_image_type == 'raw':
            cv_image_input = self.cvBridge.imgmsg_to_cv2(image_msg, 'bgr8')

        MIN_MATCH_COUNT = 9
        MIN_MSE_DECISION = 50000

        # find the keypoints and descriptors with SIFT
        kp1, des1 = self.sift.detectAndCompute(cv_image_input, None)

        matches_parking = self.flann.knnMatch(des1, self.des_parking, k=2)
        image_out_num = 1

        good_parking = []
        for m, n in matches_parking:
            if m.distance < 0.7*n.distance:
                good_parking.append(m)

        if len(good_parking) > MIN_MATCH_COUNT:
            src_pts = np.float32([kp1[m.queryIdx].pt for m in good_parking]).reshape(-1, 1, 2)
            dst_pts = np.float32([
                self.kp_parking[m.trainIdx].pt for m in good_parking
            ]).reshape(-1, 1, 2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
            matchesMask_parking = mask.ravel().tolist()

            mse = self.fnCalcMSE(src_pts, dst_pts)
            if mse < MIN_MSE_DECISION:
                msg_sign = UInt8()
                msg_sign.data = self.TrafficSign.parking.value

                self.pub_traffic_sign.publish(msg_sign)

                self.get_logger().info('parking')
                image_out_num = 2

        else:
            matchesMask_parking = None
            # self.get_logger().info('nothing')

        if image_out_num == 1:
            if self.pub_image_type == 'compressed':
                self.pub_image_traffic_sign.publish(
                    self.cvBridge.cv2_to_compressed_imgmsg(
                        cv_image_input, 'jpg'
                    )
                )

            elif self.pub_image_type == 'raw':
                self.pub_image_traffic_sign.publish(
                    self.cvBridge.cv2_to_imgmsg(
                        cv_image_input, 'bgr8'
                    )
                )

        elif image_out_num == 2:
            draw_params = {
                'matchColor': (255, 0, 0),  # draw matches in green color
                'singlePointColor': None,
                'matchesMask': matchesMask_parking,  # draw only inliers
                'flags': 2
            }

            final_parking = cv2.drawMatches(
                cv_image_input,
                kp1,
                self.img_parking,
                self.kp_parking,
                good_parking,
                None,
                **draw_params
            )

            if self.pub_image_type == 'compressed':
                self.pub_image_traffic_sign.publish(
                    self.cvBridge.cv2_to_compressed_imgmsg(
                        final_parking, 'jpg'
                    )
                )

            elif self.pub_image_type == 'raw':
                self.pub_image_traffic_sign.publish(
                    self.cvBridge.cv2_to_imgmsg(
                        final_parking, 'bgr8'
                    )
                )


def main(args=None):
    rclpy.init(args=args)
    node = DetectSign()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
