#!/usr/bin/env python3
# Copyright (c) 2026 buoyancy_test authors.
#
# Integration test for the ThrusterPlugin (facade over the propeller dynamics
# and the thrust conversion):
#   * loads a gravity free world with four instrumented probes, each driven by
#     a thruster with a different geometry / limit configuration;
#   * validates that a rotor speed command produces the expected thrust force;
#   * validates that the thrust also produces the moment of that force about
#     the centre of gravity of the vehicle, i.e. tau = (application point -
#     centre of gravity) x force (the legacy classic plugin produced the force
#     only);
#   * validates the input (rotor speed) and output (thrust) limits;
#   * validates that a zero command removes the wrench completely.
#
# Every probe has mass 10 kg and Izz = 1 kg m^2, its thruster pushes along +y
# with rotorConstant = 1 (F = |w| w) and the world has no gravity and no
# damping, so the probes coast after the command is zeroed.
#
# The poses are observed through the gz pose topic bridged into ROS as
# tf2_msgs/TFMessage, the commands are sent as std_msgs/Float64 into the
# gz input topics.

import collections
import math
import os
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_prefix
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage

WORLD_NAME = 'thruster_probe'
POSE_TOPIC = '/world/' + WORLD_NAME + '/dynamic_pose/info'

PROBE_NO_ARM = 'probe_no_arm'
PROBE_ARM = 'probe_arm'
PROBE_ARM_COG = 'probe_arm_cog'
PROBE_CLAMPED = 'probe_clamped'
PROBES = [PROBE_NO_ARM, PROBE_ARM, PROBE_ARM_COG, PROBE_CLAMPED]

# Command sent to every thruster, and the clamp applied by probe_clamped.
COMMAND = 1.0
CLAMP_MAX = 0.5
# thrust = rotorConstant * |w| * w with rotorConstant = 1.
THRUST_RATIO_CLAMPED = (CLAMP_MAX / COMMAND) ** 2  # 0.25

# Arm geometry, mass and inertia of the probes
# (see test/worlds/thruster_test.world).
ARM = 0.5
COG = 0.25
MOMENT_RATIO_COG = (ARM - COG) / ARM  # 0.5
MASS = 10.0
INERTIA_ZZ = 1.0

READY_TIMEOUT_S = 90.0
# Time given to the probes to accelerate after a command is sent.
COMMAND_SETTLE_S = 2.0
# Interval between two consecutive pose snapshots.
SAMPLE_INTERVAL_S = 0.5

# Pose of one probe in one pose message: geometry_msgs transform plus the yaw
# angle unwrapped over the whole run (a quaternion alone wraps at +-pi).
ProbePose = collections.namedtuple('ProbePose', 'transform yaw')


def input_topic(probe):
    return '/' + probe + '/thrusters/id_0/input'


def wrap_to_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_of(rotation):
    """Yaw angle of a geometry_msgs quaternion."""
    x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def generate_test_description():
    plugin_lib = os.path.join(get_package_prefix('gazebo_plugins'), 'lib')
    world_file = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 'worlds',
        'thruster_test.world')

    # The probes are part of the world file, so their system plugins are loaded
    # together with the world.
    simulator = ExecuteProcess(
        cmd=['ign', 'gazebo', '-s', '-r', '-v', '3', world_file],
        output='screen')

    bridges = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            POSE_TOPIC + '@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        ] + [
            input_topic(probe) + '@std_msgs/msg/Float64]ignition.msgs.Double'
            for probe in PROBES
        ],
        output='screen')

    return launch.LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_lib),
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', plugin_lib),
        simulator,
        bridges,
        launch_testing.actions.ReadyToTest(),
    ])


class TestThrusterIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('thruster_integration_test')
        cls.poses = {}
        cls.accumulated_yaw = {}
        cls.messages = 0
        cls.initial = {}
        cls.subscription = cls.node.create_subscription(
            TFMessage, POSE_TOPIC, cls._on_pose, 50)
        cls.command_publishers = {
            probe: cls.node.create_publisher(Float64, input_topic(probe), 10)
            for probe in PROBES
        }

    @classmethod
    def _on_pose(cls, message):
        for transform in message.transforms:
            name = transform.child_frame_id.lstrip('/')
            cls.poses[name] = transform.transform
            yaw = yaw_of(transform.transform.rotation)
            if name in cls.accumulated_yaw:
                yaw = cls.accumulated_yaw[name] + wrap_to_pi(
                    yaw - cls.accumulated_yaw[name])
            cls.accumulated_yaw[name] = yaw
        cls.messages += 1

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    # -- helpers ------------------------------------------------------------

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def snapshot_of_all_probes(self, timeout=READY_TIMEOUT_S):
        """Waits for a fresh pose message holding every probe."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            seen = self.messages
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.messages > seen and all(p in self.poses for p in PROBES):
                return {
                    probe: ProbePose(self.poses[probe],
                                     self.accumulated_yaw[probe])
                    for probe in PROBES
                }
        return None

    def set_command(self, value):
        """Publishes a rotor speed command to every thruster."""
        message = Float64()
        message.data = value
        for _ in range(10):
            for publisher in self.command_publishers.values():
                publisher.publish(message)
            self.spin_for(0.05)

    def position(self, probe, snapshot):
        """Position of a probe relative to its spawn pose."""
        now = snapshot[probe].transform.translation
        start = TestThrusterIntegration.initial[probe].transform.translation
        return (now.x - start.x, now.y - start.y, now.z - start.z)

    def speed(self, probe, first, second):
        """Mean speed of a probe between two snapshots."""
        a = self.position(probe, first)
        b = self.position(probe, second)
        distance = math.sqrt(sum((b[i] - a[i]) ** 2 for i in range(3)))
        return distance / SAMPLE_INTERVAL_S

    def yaw_rate(self, probe, first, second):
        return (second[probe].yaw - first[probe].yaw) / SAMPLE_INTERVAL_S

    # -- tests (names enforce the execution order) ---------------------------

    def test_01_probes_are_loaded_and_at_rest(self):
        snapshot = self.snapshot_of_all_probes()
        self.assertIsNotNone(snapshot, 'the probes were not reported by gz')

        for probe in PROBES:
            self.assertAlmostEqual(
                snapshot[probe].yaw, 0.0, delta=math.radians(1.0),
                msg='%s is not aligned before the command' % probe)

        # Kept on the class: unittest creates a new instance per test method.
        TestThrusterIntegration.initial = snapshot
        self.assertGreaterEqual(self.messages, 1, 'no pose updates received')

    def test_02_command_produces_force_and_moment(self):
        self.set_command(COMMAND)
        self.spin_for(COMMAND_SETTLE_S)

        first = self.snapshot_of_all_probes()
        self.assertIsNotNone(first, 'no pose updates after the command')

        # -- force: the thrust pushes the probe along +y, and the clamped
        #    thruster (clamp_max^2 / command^2 of the thrust) moves a quarter
        #    as far.
        x_no_arm, y_no_arm, _ = self.position(PROBE_NO_ARM, first)
        _, y_clamped, _ = self.position(PROBE_CLAMPED, first)

        self.assertGreater(y_no_arm, 0.02,
                           'the thruster did not move probe_no_arm')
        self.assertAlmostEqual(x_no_arm, 0.0, delta=1e-3,
                               msg='the thrust is not aligned with +y')
        self.assertAlmostEqual(
            y_clamped, THRUST_RATIO_CLAMPED * y_no_arm, delta=0.02,
            msg='the clamped thruster did not produce a quarter of the '
                'displacement (%.4f vs %.4f)' % (y_clamped, y_no_arm))

        # -- moment: without an arm there is no rotation at all, with an arm
        #    the probe rotates about +z, and moving the centre of gravity to
        #    0.25 m halves the moment (and therefore the angle).
        yaw_no_arm = first[PROBE_NO_ARM].yaw
        yaw_arm = first[PROBE_ARM].yaw
        yaw_arm_cog = first[PROBE_ARM_COG].yaw

        self.assertAlmostEqual(yaw_no_arm, 0.0, delta=math.radians(1.0),
                               msg='a thrust through the CoG must not rotate '
                                   'the body')
        self.assertGreater(math.degrees(yaw_arm), 20.0,
                           'the moment about the CoG is missing')
        self.assertAlmostEqual(
            yaw_arm_cog, MOMENT_RATIO_COG * yaw_arm, delta=0.05,
            msg='the centre of gravity offset was not applied: %.2f deg vs '
                '%.2f deg' % (math.degrees(yaw_arm_cog),
                              math.degrees(yaw_arm)))

        # -- the moment must not change the force. Both probes start together
        #    and share one clock, and with a constant thrust the straight
        #    probe satisfies y = F t^2 / (2 m) while the rotating one satisfies
        #    yaw = a F t^2 / (2 Izz), so
        #        y / yaw = Izz / (m * a)
        #    holds at any time after the command: the force cancels out and
        #    only the moment arm of probe_arm survives.
        expected_ratio = INERTIA_ZZ / (MASS * ARM)
        self.assertAlmostEqual(
            y_no_arm / yaw_arm, expected_ratio, delta=0.02 * expected_ratio,
            msg='the force and the moment are inconsistent: %.4f vs %.4f'
                % (y_no_arm / yaw_arm, expected_ratio))

    def test_03_zero_command_removes_the_wrench(self):
        self.set_command(0.0)
        self.spin_for(SAMPLE_INTERVAL_S)

        first = self.snapshot_of_all_probes()
        self.assertIsNotNone(first, 'no pose updates after stopping')
        self.spin_for(SAMPLE_INTERVAL_S)
        second = self.snapshot_of_all_probes()
        self.assertIsNotNone(second, 'no pose updates after stopping')
        self.spin_for(SAMPLE_INTERVAL_S)
        third = self.snapshot_of_all_probes()
        self.assertIsNotNone(third, 'no pose updates after stopping')

        # No gravity and no damping: a body without a wrench keeps its
        # velocity, so two consecutive intervals must be identical.
        speed_first = self.speed(PROBE_NO_ARM, first, second)
        speed_second = self.speed(PROBE_NO_ARM, second, third)
        self.assertGreater(speed_first, 0.01,
                           'probe_no_arm stopped before the measured interval')
        self.assertAlmostEqual(
            speed_second / speed_first, 1.0, delta=0.1,
            msg='the thruster still applies a force after a zero command')

        rate_first = self.yaw_rate(PROBE_ARM, first, second)
        rate_second = self.yaw_rate(PROBE_ARM, second, third)
        self.assertGreater(abs(rate_first), 0.1,
                           'probe_arm is not rotating any more')
        self.assertAlmostEqual(
            rate_second / rate_first, 1.0, delta=0.1,
            msg='the thruster still applies a moment after a zero command')
