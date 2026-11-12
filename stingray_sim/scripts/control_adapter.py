#!/usr/bin/env python3
"""Small closed-loop plant adapter implementing the Stingray control API."""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64, UInt8


def clamp(value, limit):
    return max(-limit, min(limit, value))


def angle_error(target, actual):
    return (target - actual + math.pi) % (2.0 * math.pi) - math.pi


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ControlAdapter(Node):
    AUTOMATIC = 2

    def __init__(self):
        super().__init__('stingray_sim_control')
        self.declare_parameter('robot_name', 'stingray')
        self.declare_parameter('max_rotor_speed', 300.0)
        self.declare_parameter('position_gain', 90.0)
        self.declare_parameter('velocity_gain', 120.0)
        self.declare_parameter('yaw_gain', 140.0)
        self.declare_parameter('yaw_rate_gain', 80.0)
        self.declare_parameter('command_timeout', 2.0)
        name = self.get_parameter('robot_name').value
        self.thruster_publishers = [self.create_publisher(
            Float64, f'/{name}/thrusters/id_{i}/input', 10) for i in range(6)]
        self.pose = None
        self.target = None
        self.last_command = None
        self.last_command_time = None
        self.mode = self.AUTOMATIC
        self.loop_flags = 0x3f
        self.create_subscription(
            Odometry, '/core/state/odometry', self.on_odometry,
            qos_profile_sensor_data)
        self.create_subscription(Twist, '/control/data', self.on_command, 10)
        self.create_subscription(UInt8, '/control/mode', self.on_mode, 10)
        self.create_subscription(
            UInt8, '/control/loop_flags', self.on_loop_flags, 10)
        self.create_timer(0.02, self.update)

    def on_odometry(self, msg):
        self.pose = msg
        if self.target is None:
            self.capture_hold()

    def capture_hold(self):
        if self.pose is None:
            return
        p = self.pose.pose.pose.position
        self.target = [p.x, p.y, p.z, yaw_of(
            self.pose.pose.pose.orientation)]

    def on_mode(self, msg):
        self.mode = int(msg.data)
        self.capture_hold()

    def on_loop_flags(self, msg):
        self.loop_flags = int(msg.data)

    def on_command(self, msg):
        if self.pose is None or self.mode != self.AUTOMATIC:
            return
        values = (msg.linear.x, msg.linear.y, msg.linear.z,
                  msg.angular.z)
        changed = self.last_command is None or any(
            abs(a - b) > 1e-6 for a, b in zip(values, self.last_command))
        if changed:
            p = self.pose.pose.pose.position
            yaw = yaw_of(self.pose.pose.pose.orientation)
            c, s = math.cos(yaw), math.sin(yaw)
            # Stingray uses +Y to starboard (right), while Gazebo / REP-103
            # uses +Y to port (left).
            self.target = [
                p.x + c * msg.linear.x + s * msg.linear.y,
                p.y + s * msg.linear.x - c * msg.linear.y,
                p.z - msg.linear.z,
                math.radians(msg.angular.z),
            ]
            self.last_command = values
        self.last_command_time = self.get_clock().now()

    def rotor_speed(self, force):
        # The Gazebo plugin uses F = 0.01 * |w| * w.
        speed = math.copysign(math.sqrt(abs(force) / 0.01), force) \
            if abs(force) > 1e-6 else 0.0
        return clamp(speed, self.get_parameter('max_rotor_speed').value)

    def update(self):
        if self.pose is None or self.target is None:
            return
        if self.mode != self.AUTOMATIC:
            self.publish([0.0] * 6)
            return
        p = self.pose.pose.pose.position
        twist = self.pose.twist.twist
        yaw = yaw_of(self.pose.pose.pose.orientation)
        dx, dy = self.target[0] - p.x, self.target[1] - p.y
        c, s = math.cos(yaw), math.sin(yaw)
        body_x = c * dx + s * dy
        body_right = s * dx - c * dy
        kp = self.get_parameter('position_gain').value
        kd = self.get_parameter('velocity_gain').value
        surge = kp * body_x - kd * twist.linear.x
        # The measured twist is +Y left, hence both signs are inverted for
        # the external +Y-right control convention.
        sway_right = kp * body_right + kd * twist.linear.y
        heave = kp * (self.target[2] - p.z) - kd * twist.linear.z
        yaw_force = self.get_parameter('yaw_gain').value * angle_error(
            self.target[3], yaw) - self.get_parameter('yaw_rate_gain').value \
            * twist.angular.z
        forces = [surge - yaw_force, surge + yaw_force,
                  -sway_right, -sway_right, heave, heave]
        self.publish([self.rotor_speed(force) for force in forces])

    def publish(self, values):
        for publisher, value in zip(self.thruster_publishers, values):
            msg = Float64()
            msg.data = float(value)
            publisher.publish(msg)


def main():
    rclpy.init()
    node = ControlAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
