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

from std_msgs.msg import Float64
from std_msgs.msg import Float32
from vision_msgs.msg import Detection2DArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Imu
from actuator_msgs.msg import Actuators

from stingray_interfaces.msg import Bbox
from stingray_interfaces.msg import BboxArray
from stingray_interfaces.srv import SetTwist
from stingray_interfaces.srv import SetStabilization
from stingray_interfaces.msg import UVState
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory
import importlib.resources as pkg_resources

import random
import math

import yaml
try:
    from yaml import CLoader as Loader
except ImportError:
    from yaml import Loader

from .utils.DistanceCalculator import DistanceCalculator


def quaternion_to_polar_angle(x, y, z, w): 
    # Вычисляем полярный угол на плоскости XY или попросту крен
    angle = math.atan2(2*(w*z + x*y), 1-2*(y**2 + z**2))
    angle = angle / 2 / math.pi * 360
    if angle < 0 : angle += 360
    
    return angle



ID_TO_NAME = {
    1: 'starting_zone',
    2: 'gate',
    3: 'red_flare',
    4: 'yellow_flare',
    5: 'blue_flare',
    6: 'orange_flare',
    7: 'qualification_gate',
    8: 'qualification_gate',
    9: 'qualification_gate',
    10: 'mat',
    11: 'blue_bowl',
    12: 'red_bowl',
    13: 'red_bowl',
    14: 'red_bowl',
}


def id_2_name(id_value):
    return ID_TO_NAME.get(id_value, str(id_value))

class SimulatorPerceptionNode(Node):
    """Основной узел адаптации данных симуляции в интерфейсы Stingray.

    Задачи узла:
    - принимать детекции/камеры/одометрию/IMU из симуляции;
    - преобразовывать данные в сообщения пакета `stingray_interfaces`;
    - обслуживать сервисы управления и стабилизации;
    - публиковать команды/состояние для остального стека.
    """

    def __init__(self):
        super().__init__('simulator_perception_converter')

        self.max_speed_surge = 1.7
        self.max_speed_lag = 0.7

        self.delay_bbox = 0
        self.delay_odometry = 0

        self.targetDepth = 1.6
        self.targetCourse = 0
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
        self.subscription_detect_front = self.create_subscription(Detection2DArray, '/stingray/topics/front_camera', self.detect_callback, 1)
        self.subscription_detect_bottom = self.create_subscription(Detection2DArray, '/stingray/topics/bottom_camera/bottom_camera', self.bottom_detect_callback, 1)
        self.srv_setTwist = self.create_service(SetTwist, '/stingray/services/set_twist', self.setTwist_callback)
        #self.publisher = self.create_publisher(Bbox, '/stingray/topics/front_camera/bbox_array', 10)
        self.publisher2 = self.create_publisher(BboxArray, '/stingray/topics/camera/front/bbox_array', 1)
        self.publisher3 = self.create_publisher(BboxArray, '/stingray/topics/camera/bottom/bbox_array', 1)
        self.publisher_camera_info = self.create_publisher(CameraInfo, '/stingray/topics/front_camera/camera_info', 1)
        #self.publisherDepth = self.create_publisher(Float64, '/stingray/topics/zbar', 10)
        self.publisherAngle2Pinger = self.create_publisher(Float32, '/stingray/topics/angle_hydroacoustic', 1)
        self.publisher_uv_state = self.create_publisher(UVState, '/stingray/topics/uv_state', 1)
        
        # не нужно изменять
        self.subscription_targetDepth = self.create_subscription(Float64, '/copter/depth', self.targetDepth_callback, 1)
        self.subscription_targetCourse = self.create_subscription(Float64, '/copter/course', self.targetCourse_callback, 1)
        #self.subscription_vector = self.create_subscription(Twist, '/X3/gazebo/command/twist', self.vector_callback, 10)
        self.publisherVector = self.create_publisher(Twist, '/NULINA/gazebo/command/twist0', 1)

        self.publisherNulina = self.create_publisher(Actuators, '/NULINA/gazebo/command/motor_speed', 1)

        self.subscription_camera_info = self.create_subscription(CameraInfo, '/stingray/topics/camera_info', self.vector_camera_callback, 1)
        self.subscription_camera_info2 = self.create_subscription(CameraInfo, '/stingray/topics/bottom_camera/camera_info', self.bottom_camera_callback, 1)
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
        self.subscription_detect_front
        self.subscription_detect_bottom
        self.subscription 
        self.subscription_targetDepth 
        self.subscription_targetCourse 
        #self.subscription_vector 
        self.subscription_camera_info  
        self.subscription_camera_info2

        self.bbox_attrs = self._load_bbox_attrs()
        self.distance_calculator = DistanceCalculator(object_attrs=self.bbox_attrs)

        #self.timer = self.create_timer(2, self.timer_callback)

    def publish_msg(self):
        msg = Actuators()
        msg.velocity = [100.0, 100.0, 100.0, 100.0, 0.0, 700.0]
        #self.get_logger().info('Опубликованно новое сообщение')
        #self.publisherNulina.publish(msg)

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
        self.publish_msg()

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
        vector.linear.x = min(surge / 100 * self.max_speed_surge, self.max_speed_surge)
        vector.linear.y = -min(sway / 100 * self.max_speed_lag, self.max_speed_lag)

        self.lastVector.linear.x = vector.linear.x
        self.lastVector.linear.y = vector.linear.y
        self.publisherVector.publish(self.lastVector)

        #self.get_logger().info('Incoming request!')

        # self.get_logger().info('Last Twist Vector: setTwist')
        # self.get_logger().info('x: ' + str(self.lastVector.linear.x))
        # self.get_logger().info('y: ' + str(self.lastVector.linear.y))
        # self.get_logger().info('z: ' + str(self.lastVector.linear.z))
        # self.get_logger().info('ax: ' + str(self.lastVector.angular.x))
        # self.get_logger().info('ay: ' + str(self.lastVector.angular.y))
        # self.get_logger().info('az: ' + str(self.lastVector.angular.z))

        return response

    def _load_bbox_attrs(self):
        """Загружает bbox_attrs.yaml из package resources или из share-пути."""
        try:
            with pkg_resources.open_text('simulator_perception', 'bbox_attrs.yaml') as f:
                return yaml.load(f, Loader=Loader)
        except FileNotFoundError:
            share_file = get_package_share_directory('simulator_perception') + '/bbox_attrs.yaml'
            with open(share_file, 'r', encoding='utf-8') as f:
                return yaml.load(f, Loader=Loader)

    def _extract_xyxy(self, detection):
        """Преобразует Detection2D в формат [x1, y1, x2, y2]."""
        size_x = detection.bbox.size_x
        size_y = detection.bbox.size_y
        x = detection.bbox.center.position.x
        y = detection.bbox.center.position.y
        return [x - size_x / 2, y - size_y / 2, x + size_x / 2, y + size_y / 2]

    def _make_bbox(self, object_id, xyxy, camera_info):
        """Строит сообщение Bbox с оценкой позиции и углов объекта."""
        label_name = id_2_name(object_id)
        pos_x, pos_y, pos_z, horizontal_angle, vertical_angle = self.distance_calculator.calcDistanceAndAngle(
            xyxy, label_name, camera_info)

        bbox = Bbox()
        bbox.name = label_name
        bbox.id = object_id
        bbox.confidence = 1.0
        bbox.top_left_x = int(xyxy[0])
        bbox.top_left_y = int(xyxy[1])
        bbox.bottom_right_x = int(xyxy[2])
        bbox.bottom_right_y = int(xyxy[3])
        bbox.pos_x = pos_x
        bbox.pos_y = pos_y
        bbox.pos_z = pos_z
        bbox.horizontal_angle = horizontal_angle
        bbox.vertical_angle = vertical_angle
        return bbox

    def _process_detections(self, detections, camera_info, debug_log=False):
        """Обрабатывает массив детекций и возвращает BboxArray."""
        bboxes = BboxArray()

        for detection in detections:
            object_id = int(detection.results[0].hypothesis.class_id)
            xyxy = self._extract_xyxy(detection)
            bbox = self._make_bbox(object_id, xyxy, camera_info)

            if debug_log:
                self.get_logger().info(
                    'ID: ' + bbox.name + ', POSE: ' + str(bbox.pos_x) + ' ' + str(bbox.pos_y) + ' ' + str(bbox.pos_z))
                self.get_logger().info(
                    'ID: ' + bbox.name + ', ANGLE: ' + str(bbox.vertical_angle) + ' ' + str(bbox.horizontal_angle))

            bboxes.bboxes.append(bbox)

        return bboxes

    def bottom_detect_callback(self, msg):
        #self.get_logger().info('Получено сообщение: Bbox')
        bboxes = self._process_detections(msg.detections, self.bottom_camera_info, debug_log=False)
        self.publisher3.publish(bboxes)

    def detect_callback(self, msg):
        bboxes = self._process_detections(msg.detections, self.camera_info)
        self.publisher2.publish(bboxes)

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

        # self.get_logger().info('Last Twist Vector: setTwist')
        # self.get_logger().info('x: ' + str(self.lastVector.linear.x))
        # self.get_logger().info('y: ' + str(self.lastVector.linear.y))
        # self.get_logger().info('z: ' + str(self.lastVector.linear.z))
        # self.get_logger().info('ax: ' + str(self.lastVector.angular.x))
        # self.get_logger().info('ay: ' + str(self.lastVector.angular.y))
        # self.get_logger().info('az: ' + str(self.lastVector.angular.z))

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
            # self.get_logger().info('+Получено сообщение: Camera_info ' + str(msg.width))
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
        # NOTE:
        # UVState message definition can differ between stacks/versions.
        # Some versions include distance_from_bottom / distance_from_start fields,
        # others don't. To keep the simulator converter compatible, set these
        # fields only if they exist.
        if hasattr(m, 'distance_from_bottom'):
            m.distance_from_bottom = flt32_dis2Bottom.data
        if hasattr(m, 'distance_from_start'):
            m.distance_from_start = flt32_dis2Start.data
        #m.flare_seq
        m.depth_stabilization = True
        m.roll_stabilization = True
        m.pitch_stabilization = True
        m.yaw_stabilization = True
        #await asyncio.sleep(self.delay_odometry)
        #time.sleep(self.delay_odometry)
        
        # заддержка отправки сообщений
        # if len(self.uvstate_buff) > 0 :
        #     if self.uvstate_buff[-1][1] - self.delay_odometry + 0.10 < time.time() :
        #         self.uvstate_buff.append([m, time.time() + self.delay_odometry, time.time()])
        # else : self.uvstate_buff.append([m, time.time() + self.delay_odometry, time.time()])

        # if self.uvstate_buff[0][1] <= time.time() :
        #     m = self.uvstate_buff.pop(0)
        #     self.publisher_uv_state.publish(m[0])
            # self.get_logger().info('UVState (buff_len): ' + str(len(self.uvstate_buff)) + ", (delayed time): " + str(m[1]-m[2]))
            #self.get_logger().info('UVState (time): ' + str(self.uvstate_buff))
        
        self.publisher_uv_state.publish(m)

        self.lastUVState_yaw = yaw

    def depth(self, deltaDepth) :
        speed = min(abs(deltaDepth)**2 / 4 + 0.2, 0.5)

        return speed * deltaDepth / abs(deltaDepth)

    def rotate(self, angle) :
        #self.get_logger().info('Rotate')

        x = abs(angle)
        #speed = min(math.e**x/math.e**40 + x / 200 + 0.02, 1.0)
        speed = min(math.e**x/math.e**40 + x / 100 + 0.08, 1.0)

        #if deltaAngle > 0 : self.get_logger().info('ToRight')
        #else : self.get_logger().info('ToLeft')

        self.lastVector.angular.z = -speed * angle / abs(angle)
        #self.publisherVector.publish(self.lastVector)

    def timer_callback(self):

        self.lastVector.linear.x = 1
        self.publisherVector.publish(self.lastVector)

        self.publish_msg()

        self.get_logger().info('Last Twist Vector: setTwist')
        self.get_logger().info('x: ' + str(self.lastVector.linear.x))
        self.get_logger().info('y: ' + str(self.lastVector.linear.y))
        self.get_logger().info('z: ' + str(self.lastVector.linear.z))
        self.get_logger().info('ax: ' + str(self.lastVector.angular.x))
        self.get_logger().info('ay: ' + str(self.lastVector.angular.y))
        self.get_logger().info('az: ' + str(self.lastVector.angular.z))


def main(args=None):
    rclpy.init(args=args)

    perception_node = SimulatorPerceptionNode()

    executor = MultiThreadedExecutor()
    executor.add_node(perception_node)

    executor.spin()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    perception_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
