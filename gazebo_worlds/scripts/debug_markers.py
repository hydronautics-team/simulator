#!/usr/bin/env python3
"""Debug markers for the robot (started only with debug:=true).

Publishes in-scene markers for Gazebo Sim (rendered by the built-in
MarkerManager):

  * an arrow from the body centre along the RESULTANT thruster force, i.e.
    the sum of the two ThrusterPlugin world-frame thrust vectors (line strip
    shaft + cone head, because the MarkerManager of gz sim does not support
    the ARROW marker type);

The markers are published on the debug topic /<name>/debug/marker and are
mirrored to /marker, because the built-in MarkerManager subscribes to /marker
only (its topic is hardcoded in gz sim).

Subscriptions (ROS 2, all already bridged by the spawn launch):

  /<name>/debug/pose             geometry_msgs/PoseStamped  ground truth pose
  /<name>/thrusters/id_<N>/thrust  geometry_msgs/Vector3   world frame force

Numeric debug values (depth, speeds, ...) are plotted by PlotJuggler from
their own topics instead of being drawn as scene text (the MarkerManager of
gz sim 10 does not apply the text field of TEXT markers).

Parameters:

  name         robot namespace (default 'ball')
  force_scale  arrow length in metres per newton (default 0.001)
  update_rate  marker refresh rate in Hz (default 10.0)
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3

import gz.msgs.marker_pb2 as marker_pb2
import gz.transport as gz_transport


def _quaternion_from_z_to(direction):
    """Quaternion (x, y, z, w) rotating the +z axis onto `direction`."""
    norm = math.sqrt(sum(c * c for c in direction))
    if norm < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    x, y, z = (c / norm for c in direction)

    dot = max(-1.0, min(1.0, z))
    if dot > 1.0 - 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    if dot < -1.0 + 1e-9:
        return (1.0, 0.0, 0.0, 0.0)  # 180 deg about x

    # rotation axis = z x direction
    ax, ay, az = (-y, x, 0.0)
    axis_norm = math.sqrt(ax * ax + ay * ay + az * az)
    ax, ay, az = ax / axis_norm, ay / axis_norm, az / axis_norm
    angle = math.acos(dot)
    s = math.sin(angle / 2.0)
    return (ax * s, ay * s, az * s, math.cos(angle / 2.0))


class DebugMarkers(Node):
    def __init__(self):
        super().__init__('debug_markers')

        self.declare_parameter('name', 'ball')
        self.declare_parameter('force_scale', 0.001)
        self.declare_parameter('update_rate', 10.0)

        self.name = self.get_parameter('name').value
        self.force_scale = float(self.get_parameter('force_scale').value)
        update_rate = float(self.get_parameter('update_rate').value)

        self.pose = None
        self.forces = [Vector3(), Vector3()]

        prefix = '/%s' % self.name
        self.create_subscription(
            PoseStamped, prefix + '/debug/pose', self._on_pose, 10)
        self.create_subscription(
            Vector3, prefix + '/thrusters/id_0/thrust',
            self._on_thrust_0, 10)
        self.create_subscription(
            Vector3, prefix + '/thrusters/id_1/thrust',
            self._on_thrust_1, 10)

        self.gz_node = gz_transport.Node()
        self.debug_pub = self.gz_node.advertise(
            prefix + '/debug/marker', marker_pb2.Marker)
        self.scene_pub = self.gz_node.advertise('/marker', marker_pb2.Marker)

        self.create_timer(1.0 / max(update_rate, 1.0), self._publish_markers)
        self.get_logger().info(
            'debug markers for %s: %s/debug/marker (+ /marker), '
            'force_scale %.4f m/N' % (self.name, prefix, self.force_scale))

    def _on_pose(self, msg):
        self.pose = msg

    def _on_thrust_0(self, msg):
        self.forces[0] = msg

    def _on_thrust_1(self, msg):
        self.forces[1] = msg

    def _publish(self, marker):
        self.debug_pub.publish(marker)
        self.scene_pub.publish(marker)

    def _resultant_force(self):
        fx = self.forces[0].x + self.forces[1].x
        fy = self.forces[0].y + self.forces[1].y
        fz = self.forces[0].z + self.forces[1].z
        return (fx, fy, fz)

    def _publish_markers(self):
        if self.pose is None:
            return

        origin = self.pose.pose.position
        force = self._resultant_force()

        # Shaft: line strip from the body centre along the resultant force.
        shaft = marker_pb2.Marker()
        shaft.ns = self.name
        shaft.id = 1
        shaft.action = marker_pb2.Marker.ADD_MODIFY
        shaft.type = marker_pb2.Marker.LINE_STRIP
        shaft.lifetime.sec = 1
        shaft.material.diffuse.r = 0.1
        shaft.material.diffuse.g = 1.0
        shaft.material.diffuse.b = 0.1
        shaft.material.diffuse.a = 1.0
        start = (origin.x, origin.y, origin.z)
        tip = tuple(start[i] + force[i] * self.force_scale for i in range(3))
        shaft.point.add(x=start[0], y=start[1], z=start[2])
        shaft.point.add(x=tip[0], y=tip[1], z=tip[2])
        self._publish(shaft)

        # Head: cone at the tip, oriented along the resultant force.
        head = marker_pb2.Marker()
        head.ns = self.name
        head.id = 2
        head.action = marker_pb2.Marker.ADD_MODIFY
        head.type = marker_pb2.Marker.CONE
        head.lifetime.sec = 1
        head.material.diffuse.r = 0.1
        head.material.diffuse.g = 0.8
        head.material.diffuse.b = 0.1
        head.material.diffuse.a = 1.0
        # The cone origin is its centre, so offset it by half its height
        # along the force direction.
        norm = math.sqrt(sum(c * c for c in force))
        direction = tuple(c / norm for c in force) if norm > 1e-9 else (0.0, 0.0, 1.0)
        head_height = 0.5
        head.pose.position.x = tip[0] + direction[0] * head_height / 2.0
        head.pose.position.y = tip[1] + direction[1] * head_height / 2.0
        head.pose.position.z = tip[2] + direction[2] * head_height / 2.0
        qx, qy, qz, qw = _quaternion_from_z_to(direction)
        head.pose.orientation.x = qx
        head.pose.orientation.y = qy
        head.pose.orientation.z = qz
        head.pose.orientation.w = qw
        head.scale.x = 0.15
        head.scale.y = 0.15
        head.scale.z = head_height
        self._publish(head)


def main():
    rclpy.init()
    node = DebugMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
