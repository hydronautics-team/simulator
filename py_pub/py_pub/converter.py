# Copyright 2016 Open Source Robotics Foundation, Inc.
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

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from vision_msgs.msg import Detection2DArray
from nav_msgs.msg import Odometry
from stingray_interfaces.msg import Bbox
import importlib.resources as pkg_resources

import time

import math
import sys
import os

import yaml
try:
    from yaml import CLoader as Loader
except ImportError:
    from yaml import Loader


class DistanceCalculator:
    # Y, X
    def __init__(self,
                 object_attrs: dict,
                 ):
        self.object_attrs = object_attrs

    def _tg(self, x):
        return math.tan(x/360*2*math.pi)

    def _calcDistance(self, angleSize, realSize):
        if (2 * self._tg(angleSize / 2)) == 0 : return 0
        return realSize / (2 * self._tg(angleSize / 2))

    def calcDistanceAndAngle(self, xyxy, label_name: str, ppx, ppy, fx, fy):

        if label_name in self.object_attrs:
            # camera intrinsics
            # ppx = camera_info.k[2]
            # ppy = camera_info.k[5]
            # fx = camera_info.k[0]
            # fy = camera_info.k[4]

            # distance
            obj_width = xyxy[2] - xyxy[0]
            obj_height = xyxy[3] - xyxy[1]
            obj_center_x = (xyxy[2] + xyxy[0]) / 2
            obj_center_y = (xyxy[3] + xyxy[1]) / 2

            if (obj_width == 0) : obj_width = 1
            if (obj_height == 0) : obj_height = 1

            if self.object_attrs[label_name]["reference_dim"] == "width":
                pos_z = self.object_attrs[label_name]["real_size"] * fx / obj_width
            elif self.object_attrs[label_name]["reference_dim"] == "height":
                pos_z = self.object_attrs[label_name]["real_size"] * fy / obj_height
            else:
                raise ValueError("reference_dim must be width or height")
            
            pos_x = (obj_center_x - ppx) * pos_z / fx
            pos_y = (obj_center_y - ppy) * pos_z / fy

            horizontal_angle = math.atan2(pos_x, pos_z) * 57.3
            vertical_angle = math.atan2(pos_y, pos_z) * 57.3

            return pos_x, pos_y, pos_z, horizontal_angle, vertical_angle
        
        return 1.0, 1.0, 1.0, 1.0, 1.0

def id_2_name(id) :
    if id == 1 : return 'starting_zone'
    if id == 2 : return 'gate'
    if id == 3 : return 'red_flare'
    if id == 4 : return 'yellow_flare'
    if id == 5 : return 'blue_flare'
    if id == 6 : return 'orange_flare'
    if id == 7 : return 'qualification_gate'
    if id == 8 : return 'qualification_gate'
    if id == 9 : return 'qualification_gate'
    if id == 10 : return 'mat'
    if id == 11 : return 'blue_bowl'
    if id == 12 : return 'red_bowl'
    if id == 13 : return 'red_bowl'
    if id == 14 : return 'red_bowl'
    return str(id)

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        # Создаём подписчика
        self.subscription = self.create_subscription(
            Detection2DArray,
            '/cameraaa',     # тема, на которую подписываемся
            self.detect_callback,
            10)                # очередь сообщений
        self.subscription  # предотвращаем удаление подписчика

        self.subscription = self.create_subscription(Odometry, '/model/copter/odometry', self.odometry_callback, 10)
        self.subscription  # предотвращаем удаление подписчика

        self.publisher = self.create_publisher(Bbox, 'Bbox_array', 10)
        self.publisherDepth = self.create_publisher(Float64, '/depth', 10)
        self.publisherDis2Bottom = self.create_publisher(Float64, '/distance_to_bottom', 10)
        self.publisherDis2Start = self.create_publisher(Float64, '/distance_to_start_zone', 10)
        self.publisherDis2Pinger = self.create_publisher(Float64, '/distance_to_pinger', 10)
        self.publisherAngle2Pinger = self.create_publisher(Float64, '/angle_to_pinger', 10)

    def detect_callback(self, msg):
        # Обработчик для сообщений, приходящих с 'input_topic'
        self.get_logger().info('Получено сообщение: Bbox')
        
        bbox_attrs_config_path = 'install/py_pub/resource/bbox_attrs.yaml'
        with open(bbox_attrs_config_path, 'r') as f:
        #with pkg_resources.open_text('py_pub', bbox_attrs_config_path) as f:
            bbox_attrs = yaml.load(f, Loader=Loader)
        dc = DistanceCalculator(object_attrs=bbox_attrs)
        
        detections = msg.detections
        for i in detections :
            id = int(i.results[0].hypothesis.class_id)

            size_x = i.bbox.size_x
            size_y = i.bbox.size_y
            x = i.bbox.center.position.x
            y = i.bbox.center.position.y

            x1 = x - size_x/2
            y1 = y - size_y/2
            x2 = x + size_x/2
            y2 = y + size_y/2
            

            #dc = DistanceCalculator([])
            xyxy = [x1, y1, x2, y2]
            ppx = 800
            ppy = 600
            fx = 1.047
            fy = 1.047
            pos_x, pos_y, pos_z, horizontal_angle, vertical_angle = dc.calcDistanceAndAngle(xyxy, id_2_name(id), ppx, ppy, fx, fy)

            bbox = Bbox()
            bbox.name = id_2_name(id)
            bbox.id = id
            bbox.confidence = 1.0
            bbox.top_left_x = int(x1)
            bbox.top_left_y = int(y1)
            bbox.bottom_right_x = int(x2)
            bbox.bottom_right_y = int(y2)
            bbox.pos_x = pos_x
            bbox.pos_y = pos_y 
            bbox.pos_z = pos_z
            bbox.horizontal_angle = horizontal_angle
            bbox.vertical_angle = vertical_angle

            self.get_logger().info('ID: ' + str(id) + ', POSE: ' + str(x1) + ' ' + str(y1) + ' ' + str(x2) + ' ' + str(y2))

            self.publisher.publish(bbox)


    def calc_distance(self, x1, y1, z1, x2, y2, z2) :
        x = x2 - x1
        y = y2 - y1
        z = z2 - z1

        return (x**2 + y**2 + z**2)**0.5

    def odometry_callback(self, msg):
        self.get_logger().info('Получено сообщение: Odometry')

        h = 2.0
        p_x = 24.0
        p_y = 23.0
        p_z = 0.1
        s_x = 36.0
        s_y = 0.0
        s_z = 2.0

        fx = Float64()
        fy = Float64()
        fz = Float64()
        x = fx.data = msg.pose.pose.position.x
        y = fy.data = msg.pose.pose.position.y
        z = fz.data = msg.pose.pose.position.z

        self.publisherDepth = self.create_publisher(Float64, '/depth', 10)
        self.publisherDis2Bottom = self.create_publisher(Float64, '/distance_to_bottom', 10)
        self.publisherDis2Start = self.create_publisher(Float64, '/distance_to_start_zone', 10)
        self.publisherDis2Pinger = self.create_publisher(Float64, '/distance_to_pinger', 10)
        self.publisherAngle2Pinger = self.create_publisher(Float64, '/angle_to_pinger', 10)

        flt_depth = Float64()
        flt_dis2Bottom = Float64()
        flt_dis2Start = Float64()
        flt_dis2Pinger = Float64()
        flt_angle2Pinger = Float64()
        
        flt_depth.data = h - z
        flt_dis2Bottom.data = z
        flt_dis2Start.data = self.calc_distance(x, y, z, s_x, s_y, s_z)
        flt_dis2Pinger.data = self.calc_distance(x, y, z, p_x, p_y, p_z)
        flt_angle2Pinger.data = 0.0

        self.publisherDepth.publish(flt_depth)
        self.publisherDis2Bottom.publish(flt_dis2Bottom)
        self.publisherDis2Start.publish(flt_dis2Start)
        self.publisherDis2Pinger.publish(flt_dis2Pinger)
        self.publisherAngle2Pinger.publish(flt_angle2Pinger)

    def timer_callback(self):
        msg = Float64()
        msg.data = self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 0.1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
