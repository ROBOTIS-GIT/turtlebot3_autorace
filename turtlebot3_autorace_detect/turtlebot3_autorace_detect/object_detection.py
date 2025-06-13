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
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from turtlebot3_autorace_msgs.srv import DetectionResult
from ultralytics import YOLO


class ObjectDetectionNode(LifecycleNode):

    def __init__(self):
        super().__init__('object_detection_node')

        self.declare_parameter('use_sim_time', False)
        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value

        self.declare_parameter('model_path', '~/Downloads/best.pt')
        model_path = os.path.expanduser(
            self.get_parameter('model_path').get_parameter_value().string_value)
        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        self.trigger_called = False

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring object detection...")
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating object detection...")
        if self.use_sim_time:
            self.get_logger().info('Using simulated time: subscribing to image topic')
            self.image_sub = self.create_subscription(
                Image, '/camera/image_raw', self.image_callback, 10)
        else:
            self.get_logger().info('Using real time: opening camera device')
            self.cap = cv2.VideoCapture(0)
            self.timer = self.create_timer(0.033, self.camera_timer_callback)

        self.result_cli = self.create_client(DetectionResult, 'detection_result')
        while not self.result_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for detection_result service from TaskManager...')
        self.trigger_cli = self.create_client(Trigger, 'state_change_trigger')
        while not self.trigger_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for state_change_trigger service...')
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating object detection...")
        if self.image_sub is not None:
            self.image_sub.destroy()
            self.image_sub = None
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        if hasattr(self, 'timer') and self.timer:
            self.timer.cancel()
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up object detection...")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down object detection...")
        return TransitionCallbackReturn.SUCCESS
    
    def camera_timer_callback(self):
        if not self.cap.isOpened():
            self.get_logger().error('Camera device not opened')
            return
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture image from camera')
            return
        
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = 'bgr8'
        msg.step = msg.width * 3
        msg.data = frame.tobytes()

        self.image_callback(msg)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
            results = self.model(frame)
            self.process_detection_results(results)
            annotated_frame = np.array(results[0].plot())
            annotated_frame = cv2.cvtColor(annotated_frame, cv2.COLOR_RGB2BGR)
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

        if not self.use_sim_time:
            cv2.imshow('YOLO Detection', annotated_frame)
            cv2.waitKey(1)

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
                rooms.append(int(det['label']))

        self.get_logger().info(f'Detected stores: {stores}, Detected rooms: {rooms}')

        req = DetectionResult.Request()
        req.stores = stores
        req.rooms = rooms
        future = self.result_cli.call_async(req)
        future.add_done_callback(self.detection_result_callback)
    
    def detection_result_callback(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('Detection result successfully sent.')
                if not self.trigger_called:
                    self.call_state_change_trigger()
                    self.trigger_called = True
            else:
                self.get_logger().warn('Detection result was rejected by TaskManager.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

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
