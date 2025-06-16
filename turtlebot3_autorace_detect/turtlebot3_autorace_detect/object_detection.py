#!/usr/bin/env python3
#
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
#
# Author: YeonSoo Noh

import os

import cv2
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import numpy as np
import rclpy
from collections import deque
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from turtlebot3_autorace_msgs.srv import DetectionResult
from ultralytics import YOLO


class ObjectDetectionNode(LifecycleNode):

    def __init__(self):
        super().__init__('object_detection_node')
        self.declare_parameter('model_path', '')
        self.model_path = self.get_parameter('model_path').value

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("\033[1;34mObject Dectection Node INIT\033[0m")

        self.model = YOLO(self.model_path)
        self.bridge = CvBridge()
        self.result_cli = self.create_client(DetectionResult, 'detection_result')
        while not self.result_cli.wait_for_service(timeout_sec=1.0):
          self.get_logger().warn('Service not available, waiting...')
        self.detection_in_progress = False
        self.class_history = {
            'pizza': deque(maxlen=5),
            'burger': deque(maxlen=5),
            'chicken': deque(maxlen=5),
            '101': deque(maxlen=5),
            '102': deque(maxlen=5),
            '103': deque(maxlen=5)
        }
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("\033[1;34mObject Detection Node ACTIVATE\033[0m")
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("\033[1;34mObject Detection Node DEACTIVATE\033[0m")
        if self.image_sub:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("\033[1;34mObject Detection Node CLEANUP\033[0m")
        self.model = None
        self.bridge = None
        self.result_cli = None
        self.class_history = None
        self.process_detection_results = None
        self.future = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("\033[1;34mObject Detection Node SHUTDOWN\033[0m")
        return TransitionCallbackReturn.SUCCESS

    def image_callback(self, msg):
        if self.detection_in_progress:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
            results = self.model(frame, verbose=False, conf=0.7, iou=0.1)
            detected_labels = set()
            for i, cls_id in enumerate(results[0].boxes.cls.cpu().numpy()):
                label = results[0].names[int(cls_id)]
                detected_labels.add(label)

            for label in self.class_history.keys():
                self.class_history[label].append(label in detected_labels)

            confirmed_stores = []
            confirmed_rooms = []

            for label, history in self.class_history.items():
                if len(history) == 5 and all(history):
                    if label in ['pizza', 'burger', 'chicken']:
                        confirmed_stores.append(label)
                    elif label in ['101', '102', '103']:
                        confirmed_rooms.append(int(label))

            if confirmed_stores or confirmed_rooms:
                self.process_detection_results(results)
            else:
                self.get_logger().info("Waiting for stable detection.")

            annotated_frame = np.array(results[0].plot())
            annotated_frame = cv2.cvtColor(annotated_frame, cv2.COLOR_RGB2BGR)

        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def process_detection_results(self, results):
        labels = results[0].names
        boxes = results[0].boxes
        classes = boxes.cls.cpu().numpy()
        xyxy = boxes.xyxy.cpu().numpy()

        detections = []
        for i, cls_id in enumerate(classes):
            label = labels[int(cls_id)]
            x1, y1, x2, y2 = xyxy[i]
            center_y = (y1 + y2) / 2
            detections.append({'label': label, 'center_y': center_y})

        detections.sort(key=lambda d: d['center_y'])

        stores = []
        rooms = []
        for det in detections:
            if det['label'] in ['pizza', 'chicken', 'burger']:
                stores.append(det['label'])
            elif det['label'] in ['101', '102', '103']:
                rooms.append(det['label'])

        self.get_logger().info(f'Detected stores: {stores}, Detected rooms: {rooms}')

        req = DetectionResult.Request()
        req.stores = stores
        req.rooms = rooms

        self.future = self.result_cli.call_async(req)
        self.detection_in_progress = True
        self.future.add_done_callback(self.future_callback)

    def trigger_callback(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('State change trigger successfully sent.')
            else:
                self.get_logger().warn('State change trigger rejected by TaskManager.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

    def future_callback(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('Detection result successfully sent.')
            else:
                self.detection_in_progress = False
                self.get_logger().warn('Detection result was rejected by TaskManager.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
