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

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sklearn.cluster import DBSCAN

from turtlebot3_autorace_msgs.msg import Objects
from turtlebot3_autorace_msgs.msg import Tracker


class LidarTracker(Node):
    def __init__(self):
        super().__init__('lidar_node')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        self.publisher = self.create_publisher(
            Objects,
            '/tracked_objects',
            10
        )

        self.timer = self.create_timer(0.03, self.process_func)
        self.previous_objects = []
        self.epsilon = 0.08
        self.tracking_thres = 0.5
        self.points = None
        self.last_id = 0
        self.max_tracking_distance = 0.7

    def lidar_callback(self, msg):
        self.points = self.convert_laserscan_to_points(msg)

    def convert_laserscan_to_points(self, msg):
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)
        valid = (ranges >= msg.range_min) & (ranges <= msg.range_max)
        ranges = ranges[valid]
        angles = angles[valid]
        if len(ranges) == 0:
            return np.array([])
        x = ranges * -np.sin(angles)
        y = ranges * np.cos(angles)
        return np.vstack((x, y)).T

    def process_func(self):
        if self.points is not None:
            clusters = self.detect_objects(self.points)
            tracked = self.track_objects(clusters)
            self.publish_tracked_objects(tracked)
            self.previous_objects = tracked

    def detect_objects(self, points):
        if points.shape[0] == 0:
            return []

        clustering = DBSCAN(eps=self.epsilon, min_samples=3).fit(points)
        labels = clustering.labels_
        clusters = []
        for label in set(labels):
            if label == -1:
                continue

            cluster_points = points[labels == label]
            centroid = np.mean(cluster_points, axis=0)
            if np.linalg.norm(centroid) > self.max_tracking_distance:
                continue

            rect = cv2.minAreaRect(cluster_points.astype(np.float32))
            width, height = rect[1]
            max_size = 0.5
            if width > max_size or height > max_size:
                continue

            clusters.append((centroid[0], centroid[1], cluster_points))
        return clusters

    def track_objects(self, clusters):
        tracked = []
        if len(clusters) == 0:
            self.previous_objects = []
            return []

        if len(self.previous_objects) == 0:
            for obj in clusters:
                self.last_id = (self.last_id + 1) % 256
                tracked.append((obj[0], obj[1], self.last_id, obj[2]))
            return tracked

        used_indices = set()
        for prev in self.previous_objects:
            prev_x, prev_y, prev_id, prev_points = prev
            best_index = None
            best_distance = float('inf')
            for i, cur in enumerate(clusters):
                if i in used_indices:
                    continue
                cur_x, cur_y = cur[0], cur[1]
                dist = np.sqrt((cur_x - prev_x)**2 + (cur_y - prev_y)**2)
                if dist < best_distance:
                    best_distance = dist
                    best_index = i
            if best_distance <= self.tracking_thres and best_index is not None:
                cur = clusters[best_index]
                tracked.append((cur[0], cur[1], prev_id, cur[2]))
                used_indices.add(best_index)
        for i, cur in enumerate(clusters):
            if i not in used_indices:
                self.last_id = (self.last_id + 1) % 256
                tracked.append((cur[0], cur[1], self.last_id, cur[2]))
        return tracked

    def publish_tracked_objects(self, tracked):
        objects = Objects()
        for obj in tracked:
            tracker = Tracker()
            tracker.x = float(obj[0])
            tracker.y = float(obj[1])
            tracker.id = int(obj[2])
            for pt in obj[3]:
                p = Point()
                p.x = float(pt[0])
                p.y = float(pt[1])
                p.z = 0.0
                tracker.raw_points.append(p)
            objects.objects.append(tracker)
        self.publisher.publish(objects)


def main(args=None):
    rclpy.init(args=args)
    node = LidarTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
