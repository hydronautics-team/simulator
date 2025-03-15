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
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Float64
from std_msgs.msg import Float32
from vision_msgs.msg import Detection2DArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Imu

from stingray_interfaces.msg import Bbox
from stingray_interfaces.msg import BboxArray
from stingray_core_interfaces.srv import SetTwist
from stingray_core_interfaces.srv import SetStabilization
from stingray_core_interfaces.msg import UVState
from std_srvs.srv import Trigger
import importlib.resources as pkg_resources

import time
import random
import math
import sys
import os
import asyncio

import yaml
try:
    from yaml import CLoader as Loader
except ImportError:
    from yaml import Loader

import math

def quaternion_to_polar_angle(x, y, z, w): 
    # Вычисляем полярный угол на плоскости XY или попросту крен
    angle = math.atan2(2*(w*z + x*y), 1-2*(y**2 + z**2))
    angle = angle / 2 / math.pi * 360
    if angle < 0 : angle += 360
    
    return angle



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

    def calcDistanceAndAngle(self, xyxy, label_name: str, camera_info):

        # self.get_logger().info('AAA1: ' + str(label_name))
        # self.get_logger().info('AAA2: ' + str(self.object_attrs))
        # self.get_logger().info('AAA3: ' + str(label_name in self.object_attrs))

        if label_name in self.object_attrs:
            # camera intrinsics
            ppx = camera_info.k[2]*2
            ppy = camera_info.k[5]*2
            fx = camera_info.k[0]
            fy = camera_info.k[4]

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

        self.delay_bbox = 3
        self.delay_odometry = 1.5

        self.targetDepth = 1.6
        self.targetCourse = -20
        self.lastVector = Twist()
        self.lastImu = Imu()
        self.camera_info = CameraInfo()
        self.bottom_camera_info = CameraInfo()

        self.lastUVState_yaw = 0.0
        self.yawCount = 0

        self.uvstate_buff = []
        self.bbox_buff = []
        

        # self.subscription_detect = self.create_subscription(Detection2DArray, '/cameraaa', self.detect_callback, 10)
        # self.subscription = self.create_subscription(Odometry, '/model/copter/odometry', self.odometry_callback, 10)
        # self.subscription_targetDepth = self.create_subscription(Float64, '/copter/depth', self.targetDepth_callback, 10)
        # self.subscription_targetCourse = self.create_subscription(Float64, '/copter/course', self.targetCourse_callback, 10)
        # self.subscription_vector = self.create_subscription(Twist, '/X3/gazebo/command/twist', self.vector_callback, 10)
        # self.subscription_camera_info = self.create_subscription(CameraInfo, 'camera_info', self.vector_camera_callback, 10)
        

        self.subscription = self.create_subscription(Odometry, '/model/copter/odometry', self.odometry_callback, 1)
        

        # изменено
        self.subscription_detect = self.create_subscription(Detection2DArray, '/stingray/topics/front_camera', self.detect_callback, 1)
        self.subscription_detect = self.create_subscription(Detection2DArray, '/stingray/topics/bottom_camera/bottom_camera', self.bottom_detect_callback, 1)
        self.srv_setTwist = self.create_service(SetTwist, '/stingray/services/set_twist', self.setTwist_callback)
        #self.publisher = self.create_publisher(Bbox, '/stingray/topics/front_camera/bbox_array', 10)
        self.publisher2 = self.create_publisher(BboxArray, '/stingray/topics/camera/front/bbox_array', 1)
        self.publisher3 = self.create_publisher(BboxArray, '/stingray/topics/bottom_camera/bbox_array', 1)
        self.publisher_camera_info = self.create_publisher(CameraInfo, '/stingray/topics/front_camera/camera_info', 1)
        #self.publisherDepth = self.create_publisher(Float64, '/stingray/topics/zbar', 10)
        self.publisherAngle2Pinger = self.create_publisher(Float32, '/stingray/topics/angle_hydroacoustic', 1)
        self.publisher_uv_state = self.create_publisher(UVState, '/stingray/topics/uv_state', 1)
        
        # не нужно изменять
        self.subscription_targetDepth = self.create_subscription(Float64, '/copter/depth', self.targetDepth_callback, 1)
        self.subscription_targetCourse = self.create_subscription(Float64, '/copter/course', self.targetCourse_callback, 1)
        #self.subscription_vector = self.create_subscription(Twist, '/X3/gazebo/command/twist', self.vector_callback, 10)
        self.publisherVector = self.create_publisher(Twist, '/X3/gazebo/command/twist', 1)
        self.subscription_camera_info = self.create_subscription(CameraInfo, '/stingray/topics/camera_info', self.vector_camera_callback, 1)
        self.subscription_camera_info = self.create_subscription(CameraInfo, '/stingray/topics/camera_info/bottom_camera', self.bottom_camera_callback, 1)
        self.subscription_imu = self.create_subscription(Imu, '/imu', self.imu_callback, 1)

        # сервисы
        self.srv_resetIMU = self.create_service(Trigger, '/stingray/services/reset_imu', self.resetIMU_callback)
        self.srv_setStabilization = self.create_service(SetStabilization, '/stingray/services/set_stabilization', self.setStabilization_callback)
        self.srv_enable_thrusters = self.create_service(Trigger, '/stingray/services/enable_thrusters', self.resetIMU_callback)
        
        # self.publisher = self.create_publisher(Bbox, 'Bbox_array', 10)
        # self.publisherDepth = self.create_publisher(Float64, '/depth', 10)
        # self.publisherDis2Bottom = self.create_publisher(Float64, '/distance_to_bottom', 10)
        # self.publisherDis2Start = self.create_publisher(Float64, '/distance_to_start_zone', 10)
        # self.publisherDis2Pinger = self.create_publisher(Float64, '/distance_to_pinger', 10)
        # self.publisherAngle2Pinger = self.create_publisher(Float64, '/angle_to_pinger', 10)
        
        self.publisherDis2Bottom = self.create_publisher(Float64, '/distance_to_bottom', 1)
        self.publisherDis2Start = self.create_publisher(Float64, '/distance_to_start_zone', 1)
        self.publisherDis2Pinger = self.create_publisher(Float64, '/distance_to_pinger', 1)

        self.publisherDepth = self.create_publisher(Float64, '/depth', 1)
        #self.publisherDis2Bottom = self.create_publisher(Float64, '/distance_to_bottom', 1)
        #self.publisherDis2Start = self.create_publisher(Float64, '/distance_to_start_zone', 1)
        #self.publisherDis2Pinger = self.create_publisher(Float64, '/distance_to_pinger', 1)
        #self.publisherAngle2Pinger = self.create_publisher(Float64, '/angle_to_pinger', 1)

        # предотвращаем удаление подписчиков
        self.subscription_detect 
        self.subscription 
        self.subscription_targetDepth 
        self.subscription_targetCourse 
        #self.subscription_vector 
        self.subscription_camera_info  

        self.bbox_attrs = None
        bbox_attrs_config_path = 'install/py_pub/resource/bbox_attrs.yaml'
        with open(bbox_attrs_config_path, 'r') as f:
        #with pkg_resources.open_text('py_pub', bbox_attrs_config_path) as f:
            self.bbox_attrs = yaml.load(f, Loader=Loader)

        #self.timer = self.create_timer(2, self.timer_callback)

    def resetIMU_callback(self, request, response):
        response.success = True
        response.message = "It's just a simulation bro..."

        return response
    
    def setStabilization_callback(self, request, response):
        response.success = True
        response.message = "It's just a simulation bro..."

        self.get_logger().info('AAAAAAAAAAAAAAAA service')

        return response
    
    def imu_callback(self, msg):
        self.lastImu = msg

    def setTwist_callback(self, request, response):
        response.success = True
        response.message = ""

        surge = request.surge
        sway = request.sway
        depth = request.depth
        roll = request.roll
        pitch = request.pitch
        yaw = request.yaw

        # self.get_logger().info('Получено сообщение: setTwist')
        # self.get_logger().info('surge: ' + str(surge))
        # self.get_logger().info('sway: ' + str(sway))
        # self.get_logger().info('depth: ' + str(depth))
        # self.get_logger().info('roll: ' + str(roll))
        # self.get_logger().info('pitch: ' + str(pitch))
        # self.get_logger().info('yaw: ' + str(yaw))


        # self.get_logger().info('Last Twist Vector: setTwist')
        # self.get_logger().info('x: ' + str(self.lastVector.linear.x))
        # self.get_logger().info('y: ' + str(self.lastVector.linear.y))
        # self.get_logger().info('z: ' + str(self.lastVector.linear.z))
        # self.get_logger().info('yaw: ' + str(self.lastVector.angular.z))

        self.targetDepth = 2 - depth
        #if self.targetDepth > 2.2 or self.targetDepth < -0.2 : self.targetDepth = -999

        self.targetCourse = 360 - ((yaw + 0) % 360)

        vector = Twist()
        vector.linear.x = min(surge / 100 * 1.5, 1.5)
        vector.linear.y = -min(sway / 100 * 0.7, 0.7)

        self.lastVector.linear.x = vector.linear.x
        self.lastVector.linear.y = vector.linear.y
        self.publisherVector.publish(self.lastVector)

        self.get_logger().info('Incoming request!')

        # self.get_logger().info('Last Twist Vector: setTwist')
        # self.get_logger().info('x: ' + str(self.lastVector.linear.x))
        # self.get_logger().info('y: ' + str(self.lastVector.linear.y))
        # self.get_logger().info('z: ' + str(self.lastVector.linear.z))
        # self.get_logger().info('ax: ' + str(self.lastVector.angular.x))
        # self.get_logger().info('ay: ' + str(self.lastVector.angular.y))
        # self.get_logger().info('az: ' + str(self.lastVector.angular.z))

        return response

    def bottom_detect_callback(self, msg):
        # Обработчик для сообщений, приходящих с 'input_topic'
        #self.get_logger().info('Получено сообщение: Bbox')
        
        

        #self.get_logger().info('AAA: ' + str(bbox_attrs))

        dc = DistanceCalculator(object_attrs=self.bbox_attrs)
        
        bboxes = BboxArray()
        detections = msg.detections
        for i in detections :
            id = int(i.results[0].hypothesis.class_id)

            # self.get_logger().info('AAA1: ' + id_2_name(id))
            # self.get_logger().info('AAA2: ' + str(self.bbox_attrs))
            # self.get_logger().info('AAA3: ' + str(id_2_name(id) in self.bbox_attrs))

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
            pos_x, pos_y, pos_z, horizontal_angle, vertical_angle = dc.calcDistanceAndAngle(xyxy, id_2_name(id), self.bottom_camera_info)

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

            # self.get_logger().info('ID: ' + id_2_name(id) + ', POSE: ' + str(pos_x) + ' ' + str(pos_y) + ' ' + str(pos_z))
            # self.get_logger().info('ID: ' + id_2_name(id) + ', ANGLE: ' + str(vertical_angle) + ' ' + str(horizontal_angle))

            bboxes.bboxes.append(bbox)

            #self.publisher2.publish(bbox)
        self.publisher3.publish(bboxes)

    def detect_callback(self, msg):
        # Обработчик для сообщений, приходящих с 'input_topic'
        #self.get_logger().info('Получено сообщение: Bbox')
        
        

        #self.get_logger().info('AAA: ' + str(bbox_attrs))

        dc = DistanceCalculator(object_attrs=self.bbox_attrs)
        
        bboxes = BboxArray()
        detections = msg.detections
        for i in detections :
            id = int(i.results[0].hypothesis.class_id)

            # self.get_logger().info('AAA1: ' + id_2_name(id))
            # self.get_logger().info('AAA2: ' + str(self.bbox_attrs))
            # self.get_logger().info('AAA3: ' + str(id_2_name(id) in self.bbox_attrs))

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
            pos_x, pos_y, pos_z, horizontal_angle, vertical_angle = dc.calcDistanceAndAngle(xyxy, id_2_name(id), self.camera_info)

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

            bbox2 = Bbox()
            bbox2.name = id_2_name(id)
            bbox2.id = id
            bbox2.confidence = 0.5
            bbox2.top_left_x = int(x1)
            bbox2.top_left_y = int(y1)
            bbox2.bottom_right_x = int(x2)
            bbox2.bottom_right_y = int(y2)
            bbox2.pos_x = -0.21
            bbox2.pos_y = pos_y/2
            bbox2.pos_z = pos_z/2
            bbox2.horizontal_angle = horizontal_angle/2
            bbox2.vertical_angle = vertical_angle/2

            #self.get_logger().info('ID: ' + id_2_name(id) + ', POSE: ' + str(pos_x) + ' ' + str(pos_y) + ' ' + str(pos_z))
            #self.get_logger().info('ID: ' + id_2_name(id) + ', ANGLE: ' + str(vertical_angle) + ' ' + str(horizontal_angle))

            bboxes.bboxes.append(bbox)
            #bboxes.bboxes.append(bbox2)

            #self.publisher2.publish(bbox)
        #self.get_logger().info('Time 1: ' + str(time.time()))
        #self.get_logger().info('Time 2: ' + str(time.time()))
        if len(self.bbox_buff) > 0 :
            if self.bbox_buff[-1][1] - self.delay_bbox + 0.5 < time.time() :
                self.bbox_buff.append([bboxes, time.time() + self.delay_bbox, time.time()])
        else : self.bbox_buff.append([bboxes, time.time() + self.delay_bbox, time.time()])

        if self.bbox_buff[0][1] <= time.time() :
            m = self.bbox_buff.pop(0)
            self.publisher2.publish(m[0])
            self.get_logger().info('bbox (buff_len): ' + str(len(self.bbox_buff)) + ", (delayed time)" + str(m[1]-m[2]))
        #time.sleep(self.delay_bbox)
        # q = 0
        # for i in range(10000000) :
        #     q += i
        # print(q)
        #self.publisher2.publish(bboxes)

    def targetDepth_callback(self, msg):
        # Обработчик для сообщений
        #self.get_logger().info('Получено сообщение: Depth')
        self.targetDepth = msg.data + 2
        #if self.targetDepth > 2.2 or self.targetDepth < -0.2 : self.targetDepth = -999

    def vector_callback(self, msg):
        # Обработчик для сообщений
        #self.get_logger().info('Получено сообщение: Vector')
        #msg.linear.y = 0.0
        self.lastVector = msg

        self.get_logger().info('Last Twist Vector: setTwist')
        self.get_logger().info('x: ' + str(self.lastVector.linear.x))
        self.get_logger().info('y: ' + str(self.lastVector.linear.y))
        self.get_logger().info('z: ' + str(self.lastVector.linear.z))
        self.get_logger().info('ax: ' + str(self.lastVector.angular.x))
        self.get_logger().info('ay: ' + str(self.lastVector.angular.y))
        self.get_logger().info('az: ' + str(self.lastVector.angular.z))

    def vector_camera_callback(self, msg):
        # Обработчик для сообщений
        # self.get_logger().info('Получено сообщение: Camera_info ' + str(msg.height))
        if msg.height == 360 :
            #self.get_logger().info('+Получено сообщение: Camera_info ' + str(msg.width))
            self.camera_info = msg
            self.publisher_camera_info.publish(msg)

    def bottom_camera_callback(self, msg):
        # Обработчик для сообщений
        # self.get_logger().info('Получено сообщение: Camera_info ' + str(msg.height))
        if msg.height == 480 :
            #self.get_logger().info('+Получено сообщение: Camera_info ' + str(msg.width))
            self.bottom_camera_info = msg
            #self.publisher_camera_info.publish(msg)

    def targetCourse_callback(self, msg):
        # Обработчик для сообщений
        #self.get_logger().info('Получено сообщение: Cource')
        self.targetCourse = msg.data % 360

    def calc_distance(self, x1, y1, z1, x2, y2, z2) :
        x = x2 - x1
        y = y2 - y1
        z = z2 - z1

        return (x**2 + y**2 + z**2)**0.5

    def odometry_callback(self, msg):
        #self.get_logger().info('Получено сообщение: Odometry')

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

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        polarAngle = quaternion_to_polar_angle(qx, qy, qz, qw)
        

        flt_depth = Float64()
        flt_dis2Bottom = Float64()
        flt_dis2Start = Float64()
        flt_dis2Pinger = Float64()
        flt_angle2Pinger = Float32()

        vpx = x - p_x
        vpy = y - p_y
        pinger_polarAngle = math.atan2(vpy, vpx)/2/math.pi * 360
        pinger_deltaAngle = (pinger_polarAngle- polarAngle + 180) % 360 - 180
        pinger_deltaAngle = (pinger_deltaAngle % 360 + 180) % 360
        if pinger_deltaAngle > 180 :
            pinger_deltaAngle = -(-pinger_deltaAngle % 180)
        # теперь, если deltaAngle положительно, движемся по часовой стрелке
        # отрицательно - против
        if pinger_deltaAngle > 38.75 and pinger_deltaAngle < 180 : pinger_deltaAngle = random.randint(0, 179) + 0.00001
        #if pinger_deltaAngle > 135 : pinger_deltaAngle = 180 - pinger_deltaAngle
        if pinger_deltaAngle < -38.75 and pinger_deltaAngle > -180 : pinger_deltaAngle = random.randint(0, 179) + 0.00001
        #if pinger_deltaAngle < -135 : pinger_deltaAngle = -180 - pinger_deltaAngle
        pinger_deltaAngle = -pinger_deltaAngle

        # self.get_logger().info('Полярный угол: ' + str(polarAngle))
        # self.get_logger().info('pinger_deltaAngle: ' + str(pinger_deltaAngle))
        
        
        flt_depth.data = h - z
        flt_dis2Bottom.data = z
        flt_dis2Start.data = self.calc_distance(x, y, z, s_x, s_y, s_z)
        flt_dis2Pinger.data = self.calc_distance(x, y, z, p_x, p_y, p_z)
        flt_angle2Pinger.data = pinger_deltaAngle

        # self.publisherDepth.publish(flt_depth)
        # self.publisherDis2Bottom.publish(flt_dis2Bottom)
        # self.publisherDis2Start.publish(flt_dis2Start)
        # self.publisherDis2Pinger.publish(flt_dis2Pinger)
        # self.publisherAngle2Pinger.publish(flt_angle2Pinger)
        
        

        isNeedToPublish = False

        if self.targetCourse != -999 :
            targetPolarAngle = (self.targetCourse + 90) % 360
            isNeedToPublish = True

            deltaAngle = (targetPolarAngle - polarAngle + 180) % 360 - 180
            deltaAngle = -deltaAngle
            # теперь, если deltaAngle положительно, движемся по часовой стрелке
            # отрицательно - против
            #self.get_logger().info('deltaAngle: ' + str(deltaAngle))

            if abs(deltaAngle) > 2 :
                self.rotate(deltaAngle)
                #if deltaAngle > 0 : self.rotateRight(deltaAngle)
                #else : self.rotateLeft(deltaAngle)
            else :
                #self.get_logger().info('Stop')
                self.lastVector.angular.z = 0.0
                #self.publisherVector.publish(self.lastVector)

        if self.targetDepth != -999 :
            isNeedToPublish = True

            if abs(z - self.targetDepth) > 0.1 :
                self.lastVector.linear.z = self.depth(self.targetDepth - z)
            else :
                self.lastVector.linear.z = 0.0

        if isNeedToPublish or True :
            self.publisherVector.publish(self.lastVector)

        self.publisherDepth.publish(flt_depth)
        self.publisherDis2Bottom.publish(flt_dis2Bottom)
        self.publisherDis2Start.publish(flt_dis2Start)
        self.publisherDis2Pinger.publish(flt_dis2Pinger)
        self.publisherAngle2Pinger.publish(flt_angle2Pinger)


        flt32_depth = Float32()
        flt32_dis2Bottom = Float32()
        flt32_dis2Start = Float32()
        flt32_dis2Pinger = Float32()
        flt32_angle2Pinger = Float32()
        flt32_depth.data = flt_depth.data
        flt32_dis2Bottom.data = flt_dis2Bottom.data
        flt32_dis2Start.data = flt_dis2Start.data
        flt32_dis2Pinger.data = flt_dis2Pinger.data
        flt32_angle2Pinger.data = flt_angle2Pinger.data

        flt32_surge_accel = Float32()
        flt32_sway_accel = Float32()
        flt32_surge_accel.data = self.lastImu.linear_acceleration.x
        flt32_sway_accel.data = self.lastImu.linear_acceleration.y

        yaw = 360 - (polarAngle - 90) % 360
        if (yaw > 180) : yaw = yaw - 360
        YawForVlad = 0.0

        if (self.lastUVState_yaw < -150 and yaw > 150) : self.yawCount -= 1
        elif (self.lastUVState_yaw > 150 and yaw < -150) : self.yawCount += 1
        YawForVlad = self.yawCount * 360 + yaw

        m = UVState()
        m.roll = 0.0
        m.pitch = 0.0
        #m.yaw = 360 - (polarAngle - 90) % 360
        m.yaw = YawForVlad
        m.surge_accel = flt32_surge_accel.data
        m.sway_accel = flt32_sway_accel.data
        m.depth = flt32_depth.data
        m.distance_from_bottom = flt32_dis2Bottom.data
        m.distance_from_start = flt32_dis2Start.data
        #m.flare_seq
        m.depth_stabilization = True
        m.roll_stabilization = True
        m.pitch_stabilization = True
        m.yaw_stabilization = True
        #await asyncio.sleep(self.delay_odometry)
        #time.sleep(self.delay_odometry)
        
        if len(self.uvstate_buff) > 0 :
            if self.uvstate_buff[-1][1] - self.delay_odometry + 0.10 < time.time() :
                self.uvstate_buff.append([m, time.time() + self.delay_odometry, time.time()])
        else : self.uvstate_buff.append([m, time.time() + self.delay_odometry, time.time()])

        if self.uvstate_buff[0][1] <= time.time() :
            m = self.uvstate_buff.pop(0)
            self.publisher_uv_state.publish(m[0])
            self.get_logger().info('UVState (buff_len): ' + str(len(self.uvstate_buff)) + ", (delayed time): " + str(m[1]-m[2]))
            #self.get_logger().info('UVState (time): ' + str(self.uvstate_buff))
        #self.publisher_uv_state.publish(m)

        self.lastUVState_yaw = yaw

    def depth(self, deltaDepth) :
        speed = min(abs(deltaDepth)**2 / 4 + 0.2, 0.5)

        return speed * deltaDepth / abs(deltaDepth)

    def rotate(self, angle) :
        #self.get_logger().info('Rotate')

        x = abs(angle)
        speed = min(math.e**x/math.e**40 + x / 200 + 0.02, 1.0)
        #speed = min(math.e**x/math.e**40 + x / 150 + 0.04, 1.0)

        #if deltaAngle > 0 : self.get_logger().info('ToRight')
        #else : self.get_logger().info('ToLeft')

        self.lastVector.angular.z = -speed * angle / abs(angle)
        #self.publisherVector.publish(self.lastVector)

    def timer_callback(self):

        self.lastVector.linear.x = 1
        self.publisherVector.publish(self.lastVector)

        self.get_logger().info('Last Twist Vector: setTwist')
        self.get_logger().info('x: ' + str(self.lastVector.linear.x))
        self.get_logger().info('y: ' + str(self.lastVector.linear.y))
        self.get_logger().info('z: ' + str(self.lastVector.linear.z))
        self.get_logger().info('ax: ' + str(self.lastVector.angular.x))
        self.get_logger().info('ay: ' + str(self.lastVector.angular.y))
        self.get_logger().info('az: ' + str(self.lastVector.angular.z))


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    executor = MultiThreadedExecutor()
    executor.add_node(minimal_publisher)

    executor.spin()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
