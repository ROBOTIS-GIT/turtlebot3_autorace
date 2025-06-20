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
import numpy as np
import rclpy
from collections import deque
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from turtlebot3_autorace_msgs.srv import DetectionResult
from ultralytics import YOLO


class ObjectDetectionNode(LifecycleNode):

    def __init__(self):
        super().__init__('object_detection_node')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring object detection...")
        self.declare_parameter('model_path', '/home/ubuntu/best.pt')
        model_path = os.path.expanduser(
            self.get_parameter('model_path').get_parameter_value().string_value)
        self.model = YOLO(model_path)
        self.trigger_called = False
        self.detection_in_progress = False
        self.class_history = {
            'pizza': deque(maxlen=5),
            'burger': deque(maxlen=5),
            'chicken': deque(maxlen=5),
            '101': deque(maxlen=5),
            '102': deque(maxlen=5),
            '103': deque(maxlen=5)
        }

        self.result_cli = self.create_client(DetectionResult, 'detection_result')
        self.trigger_cli = self.create_client(Trigger, 'state_change_trigger')
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating object detection...")

        self.image_sub = self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed', self.image_callback, 10)
        self.image_pub = self.create_publisher(
            CompressedImage, '/camera/detections/compressed', 10)
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating object detection...")
        if hasattr(self, 'image_sub') and self.image_sub is not None:
            self.image_sub.destroy()
            self.image_sub = None
        if hasattr(self, 'image_pub') and self.image_pub is not None:
            self.image_pub.destroy()
            self.image_pub = None
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up object detection...")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down object detection...")
        return TransitionCallbackReturn.SUCCESS

    def image_callback(self, msg):
        if self.detection_in_progress:
            return

        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            results = self.model(frame)
            detected_labels = set()
            for cls_id in results[0].boxes.cls.cpu().numpy():
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

            if confirmed_stores and confirmed_rooms:
                self.process_detection_results(results)
            else:
                self.get_logger().info("Waiting for stable detection.")

            annotated_frame = np.array(results[0].plot())
            success, encoded_image = cv2.imencode('.jpg', annotated_frame)
            if success:
                compressed_image = CompressedImage()
                compressed_image.header = msg.header
                compressed_image.format = 'jpeg'
                compressed_image.data = encoded_image.tobytes()
                self.image_pub.publish(compressed_image)

        except Exception as e:
            self.get_logger().error(f'Error in image_callback: {e}')

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

        self.detection_in_progress = True
        future = self.result_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        try:
            result = future.result()
            if result.success:
                self.get_logger().info('Detection result successfully sent.')
                if not self.trigger_called:
                    self.call_state_change_trigger()
                    self.trigger_called = True
                rclpy.shutdown()
            else:
                self.get_logger().warn('Detection result was rejected by TaskManager.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

        self.detection_in_progress = False

    def call_state_change_trigger(self):
        req = Trigger.Request()
        future = self.trigger_cli.call_async(req)
        future.add_done_callback(self.trigger_callback)
    
    def trigger_callback(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('State change trigger successfully sent.')
            else:
                self.get_logger().warn('State change trigger rejected by TaskManager.')
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
