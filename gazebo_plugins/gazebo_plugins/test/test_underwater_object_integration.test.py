#!/usr/bin/env python3
# Copyright (c) 2026 simulator authors.
#
# Integration test for the UnderwaterObjectPlugin:
#   * loads the underwater world with the vehicle (ball);
#   * validates that the vehicle is present and neutrally buoyant;
#   * validates that the depth stays stable (no commands: the plugin reads
#     the velocity from the link components only).
#
# The body pose is observed through the gz pose topic bridged into ROS as
# tf2_msgs/TFMessage.

import os
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import rclpy
from ament_index_python.packages import (get_package_prefix,
                                         get_package_share_directory)
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from tf2_msgs.msg import TFMessage

MODEL_NAME = 'ball'
WORLD_NAME = 'empty_underwater'
POSE_TOPIC = '/world/' + WORLD_NAME + '/dynamic_pose/info'

READY_TIMEOUT_S = 90.0

# Default spawn height of the vehicle (see gazebo_worlds/default.world)
SPAWN_Z = -20.0


def generate_test_description():
    plugin_lib = os.path.join(get_package_prefix('gazebo_plugins'), 'lib')
    descriptions_models = os.path.join(
        get_package_share_directory('descriptions'), 'models')
    worlds_models = os.path.join(
        get_package_share_directory('gazebo_worlds'), 'models')
    world_file = os.path.join(
        get_package_share_directory('gazebo_worlds'), 'worlds',
        'default.world')
    resource_path = os.pathsep.join([descriptions_models, worlds_models])

    # The vehicle is included in the world file so that its system plugin is
    # loaded together with the world.
    simulator = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', '-v', '3', world_file],
        output='screen')

    bridges = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            POSE_TOPIC + '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        output='screen')

    return launch.LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_lib),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        simulator,
        bridges,
        launch_testing.actions.ReadyToTest(),
    ])


class TestUnderwaterObjectIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('underwater_object_integration_test')
        cls.poses = {}
        cls.subscription = cls.node.create_subscription(
            TFMessage, POSE_TOPIC, cls._on_pose, 50)

    @classmethod
    def _on_pose(cls, message):
        for transform in message.transforms:
            name = transform.child_frame_id.lstrip('/')
            cls.poses[name] = transform.transform.translation

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    # -- helpers ------------------------------------------------------------

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait_for_pose(self, name, timeout=READY_TIMEOUT_S):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if name in self.poses:
                return self.poses[name]
        return None

    def get_pose(self, name):
        rclpy.spin_once(self.node, timeout_sec=0.05)
        return self.poses.get(name)

    # -- tests (names enforce the execution order) ---------------------------

    def test_01_model_is_loaded_and_neutrally_buoyant(self):
        pose = self.wait_for_pose(MODEL_NAME)
        self.assertIsNotNone(pose, 'model %s was not loaded' % MODEL_NAME)
        self.assertAlmostEqual(pose.z, SPAWN_Z, delta=1.5,
                               msg='unexpected initial depth: %s' % pose.z)

    def test_02_depth_is_stable_without_commands(self):
        z_values = []
        for _ in range(10):
            pose = self.get_pose(MODEL_NAME)
            if pose:
                z_values.append(pose.z)
            self.spin_for(0.2)
        self.assertGreater(len(z_values), 5, 'no pose updates received')
        spread = max(z_values) - min(z_values)
        self.assertLess(spread, 0.5,
                        'depth drifted by %.3f m without commands' % spread)


