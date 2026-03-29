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

"""Manual thruster bridge node.

Нода принимает простые ручные команды и публикует команды в топики тяги/скорости
для симуляции.
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class ManualThrusterBridgeNode(Node):
    """Bridge между ручными командами и командами движителей.

    Подписки:
    - `hand_cmd`: переключение направления боковых роторов (0/1).
    - `val_spd`: команда скорости, ограниченная диапазоном [-5, 5].

    Публикации:
    - `val_speed`
    - `rotor_cmd2`
    - `rotor_cmd3`
    """

    def __init__(self):
        super().__init__('manual_thruster_bridge_node')

        self.hand_cmd_sub = self.create_subscription(
            Float64,
            'hand_cmd',
            self._on_hand_cmd,
            10,
        )
        self.val_spd_sub = self.create_subscription(
            Float64,
            'val_spd',
            self._on_val_speed,
            10,
        )

        self.val_speed_pub = self.create_publisher(Float64, 'val_speed', 10)
        self.left_rotor_pub = self.create_publisher(Float64, 'rotor_cmd2', 10)
        self.right_rotor_pub = self.create_publisher(Float64, 'rotor_cmd3', 10)

        # Первичная инициализация роторов после старта симуляции.
        time.sleep(5)
        self._publish_rotors(-1.0, 1.0)

    def _publish_rotors(self, left_value: float, right_value: float) -> None:
        """Публикует значения для левого и правого роторов."""
        left_msg = Float64()
        left_msg.data = left_value

        right_msg = Float64()
        right_msg.data = right_value

        self.left_rotor_pub.publish(left_msg)
        self.right_rotor_pub.publish(right_msg)

    def _on_hand_cmd(self, msg: Float64) -> None:
        """Обрабатывает ручную команду 0/1 и публикует парные команды роторов."""
        self.get_logger().info('Получено сообщение: hand ' + str(msg.data))

        angle = msg.data
        if angle not in (0.0, 1.0):
            return

        # Историческая логика: 0.0 трактуется как -1.0.
        if angle == 0.0:
            angle = -1.0

        self._publish_rotors(angle, -angle)

    def _on_val_speed(self, msg: Float64) -> None:
        """Обрабатывает целевую скорость и ограничивает её диапазоном [-5, 5]."""
        self.get_logger().info('Получено сообщение: val ' + str(msg.data))

        speed = msg.data
        if speed < -5.0:
            speed = -5.0
        if speed > 5.0:
            speed = 5.0

        out = Float64()
        out.data = speed
        self.val_speed_pub.publish(out)


def main(args=None):
    """Entry point узла."""
    rclpy.init(args=args)

    node = ManualThrusterBridgeNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()