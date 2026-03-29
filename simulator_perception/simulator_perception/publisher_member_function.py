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

import time


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        # Создаём подписчика
        self.subscription = self.create_subscription(
            Float64,
            'hand_cmd',     # тема, на которую подписываемся
            self.listener_callback,
            10)                # очередь сообщений
        self.subscription  # предотвращаем удаление подписчика

        self.subscription2 = self.create_subscription(
            Float64,
            'val_spd',     # тема, на которую подписываемся
            self.listener_callback2,
            10)                # очередь сообщений
        self.subscription2  # предотвращаем удаление подписчика

        self.publisher_ = self.create_publisher(Float64, 'val_speed', 10)
        #timer_period = 1.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.i = 0.0

        self.publisher_l = self.create_publisher(Float64, 'rotor_cmd2', 10)
        #timer_period = 1.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.i = 0.0

        self.publisher_r = self.create_publisher(Float64, 'rotor_cmd3', 10)

        time.sleep(5)
        new_msg_l = Float64()
        new_msg_l.data = -1.0
        new_msg_r = Float64()
        new_msg_r.data = 1.0

        self.publisher_l.publish(new_msg_l)
        self.publisher_r.publish(new_msg_r)
        #timer_period = 1.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.i = 0.0

    def listener_callback(self, msg):
        # Обработчик для сообщений, приходящих с 'input_topic'
        self.get_logger().info('Получено сообщение: hand ' + str(msg.data))
        
        # Пример публикации полученного сообщения с изменениями
        

        angle = msg.data
        if angle != 0.0 and angle != 1.0 : return
        # if angle < -1.0 : angle = -1.0
        # if angle > 1.0 : angle = 1.0
        if angle == 0.0 : angle = -1.0

        new_msg_l = Float64()
        new_msg_l.data = angle
        new_msg_r = Float64()
        new_msg_r.data = -angle

        self.publisher_l.publish(new_msg_l)
        self.publisher_r.publish(new_msg_r)
        #self.get_logger().info(f'Отправлено сообщение: "{new_msg.data}"')

    def listener_callback2(self, msg):
        # Обработчик для сообщений, приходящих с 'input_topic'
        self.get_logger().info('Получено сообщение: val ' + str(msg.data))
        
        # Пример публикации полученного сообщения с изменениями
        

        angle = msg.data
        new_msg = Float64()

        if angle < -5.0 : angle = -5.0
        if angle > 5.0 : angle = 5.0

        new_msg.data = angle

        self.publisher_.publish(new_msg)
        #self.get_logger().info(f'Отправлено сообщение: "{new_msg.data}"')

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
