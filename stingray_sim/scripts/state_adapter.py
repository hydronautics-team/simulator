#!/usr/bin/env python3
"""Turn Gazebo ground truth into the state topics consumed by Stingray."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class StateAdapter(Node):
    def __init__(self):
        super().__init__('stingray_sim_state')
        self.declare_parameter('ground_truth_topic', '/sim/ground_truth/pose')
        self.declare_parameter('zero_at_start', True)
        self.origin = None
        self.previous = None
        # stingray_missions uses the default reliable subscription for state;
        # a reliable publisher remains compatible with SensorDataQoS clients
        # such as the planner and the synthetic detector.
        self.odom_pub = self.create_publisher(
            Odometry, '/core/state/odometry', 10)
        self.dvl_pub = self.create_publisher(
            Odometry, '/stingray_core/sensors/dvl/odometry', 10)
        self.tf_pub = TransformBroadcaster(self)
        static_pub = StaticTransformBroadcaster(self)
        identity = TransformStamped()
        identity.header.stamp = self.get_clock().now().to_msg()
        identity.header.frame_id = 'map'
        identity.child_frame_id = 'odom'
        identity.transform.rotation.w = 1.0
        static_pub.sendTransform(identity)
        self.static_pub = static_pub
        self.create_subscription(
            PoseStamped,
            self.get_parameter('ground_truth_topic').value,
            self.on_pose,
            qos_profile_sensor_data)

    def on_pose(self, msg):
        now = self.get_clock().now()
        p = msg.pose.position
        if self.origin is None:
            self.origin = (p.x, p.y, p.z) if self.get_parameter(
                'zero_at_start').value else (0.0, 0.0, 0.0)
            self.get_logger().info(
                'Map origin fixed at simulator position '
                f'({self.origin[0]:.2f}, {self.origin[1]:.2f}, '
                f'{self.origin[2]:.2f})')

        position = (p.x - self.origin[0], p.y - self.origin[1],
                    p.z - self.origin[2])
        linear_world = (0.0, 0.0, 0.0)
        angular_z = 0.0
        if self.previous is not None:
            old_time, old_position, old_yaw = self.previous
            dt = (now - old_time).nanoseconds * 1e-9
            if dt > 1e-4:
                linear_world = tuple(
                    (position[i] - old_position[i]) / dt for i in range(3))
                dyaw = (yaw_of(msg.pose.orientation) - old_yaw + math.pi) \
                    % (2.0 * math.pi) - math.pi
                angular_z = dyaw / dt
        current_yaw = yaw_of(msg.pose.orientation)
        self.previous = (now, position, current_yaw)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose = msg.pose
        odom.pose.pose.position.x = position[0]
        odom.pose.pose.position.y = position[1]
        odom.pose.pose.position.z = position[2]
        c, s = math.cos(current_yaw), math.sin(current_yaw)
        odom.twist.twist.linear.x = c * linear_world[0] + s * linear_world[1]
        odom.twist.twist.linear.y = -s * linear_world[0] + c * linear_world[1]
        odom.twist.twist.linear.z = linear_world[2]
        odom.twist.twist.angular.z = angular_z
        self.odom_pub.publish(odom)
        self.dvl_pub.publish(odom)

        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = position[0]
        transform.transform.translation.y = position[1]
        transform.transform.translation.z = position[2]
        transform.transform.rotation = msg.pose.orientation
        self.tf_pub.sendTransform(transform)


def main():
    rclpy.init()
    node = StateAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
