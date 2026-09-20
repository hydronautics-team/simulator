#!/usr/bin/env python3
# Copyright (c) 2026 simulator authors.
#
# Spawn the buoyancy test robot into a running Gazebo Sim world.
#
# The robot is described by ../robots/ball.xacro (a sphere with two cube
# thrusters) and that xacro is the single source of truth for the model: it is
# expanded when the launch file runs and the resulting URDF is handed to the
# `create` node of ros_gz_sim, which converts it to SDF. Link names and plugin
# blocks survive the conversion, so the spawn always matches the current
# versions of libunderwater_object.so and libthruster.so.
#
# The world must already be running, e.g.:
#   ros2 launch gazebo_worlds empty_underwater_world.launch.py
#
# Control: libthruster.so listens on the gz topics
#   /<name>/thrusters/id_0/input   (left thruster,  rotor speed in rad/s)
#   /<name>/thrusters/id_1/input   (right thruster, rotor speed in rad/s)
# (gz.msgs.Double; thrust = rotorConstant * |w| * w) and publishes the
# world frame thrust on /<name>/thrusters/id_<id>/thrust. With thrusters:=true
# those topics are bridged to ROS 2, so the robot can be driven with e.g.
#   ros2 topic pub -r 10 /ball/thrusters/id_0/input \
#       std_msgs/msg/Float64 "{data: 100.0}"
# Equal rotor speeds on both thrusters move the robot forward, opposite speeds
# turn it, because each thruster pushes at y = +-0.6 m from the centre of
# gravity (application_point / center_of_mass in the xacro).
#
# Usage:
#   ros2 launch descriptions upload_rexrov_default.launch.py                # ball
#   ros2 launch descriptions upload_rexrov_default.launch.py name:=rov z:=-30
#
# teleop:=true (default) also starts the keyboard teleop node in this terminal:
# w/s/a/d or the arrow keys drive the robot, space stops it and q quits. Every
# key event adds step_rpm to the involved thrusters, so the robot keeps its
# throttle after the keys are released and the opposite key removes it again.
# The node publishes the resulting rotor speeds on the bridged thruster topics,
# so it needs thrusters:=true as well; its own defaults are used unless
# max_rpm / step_rpm are given here.
#
# Spawn only, drive from a second terminal:
#   ros2 launch descriptions upload_rexrov_default.launch.py teleop:=false
#   ros2 run descriptions ball_teleop.py --ros-args -p name:=ball

import pathlib

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.actions import OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration as Lc
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Robot description and default entity name (works from source and installed
# share trees: <package>/launch/<this file> -> <package>)
PKG_DIR = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_XACRO = str(PKG_DIR / 'robots' / 'ball.xacro')

# Thrusters described by the xacro, used to build the bridge topics
THRUSTER_IDS = (0, 1)


def to_bool(value: str):
    if isinstance(value, bool):
        return value
    if not isinstance(value, str):
        raise ValueError('String to bool, invalid value: ' + str(value))
    valid = {'true': True, '1': True, 'false': False, '0': False}
    if value.lower() in valid:
        return valid[value.lower()]
    raise ValueError('String to bool, invalid value: %s' % value)


def launch_setup(context, *args, **kwargs):
    world = Lc('world').perform(context)
    xacro_file = Lc('xacro').perform(context)
    name = Lc('name').perform(context)
    # The create node declares the pose parameters as doubles, so convert the
    # launch arguments here (a string would be written to the params file as a
    # YAML string and rejected with InvalidParameterTypeException).
    x = float(Lc('x').perform(context))
    y = float(Lc('y').perform(context))
    z = float(Lc('z').perform(context))
    roll = float(Lc('roll').perform(context))
    pitch = float(Lc('pitch').perform(context))
    yaw = float(Lc('yaw').perform(context))
    verbose = to_bool(Lc('verbose').perform(context))
    debug = to_bool(Lc('debug').perform(context))
    thrusters = to_bool(Lc('thrusters').perform(context))
    teleop = to_bool(Lc('teleop').perform(context))
    # Empty means "keep the default of the teleop node itself".
    max_rpm = Lc('max_rpm').perform(context).strip()
    step_rpm = Lc('step_rpm').perform(context).strip()

    # Expand the xacro now; the same name is used as the link prefix inside the
    # model and as the entity name in the world.
    model = Command(['xacro ', xacro_file, ' namespace:=', name])

    if debug:
        gzLogVerbosity = '4'
    elif verbose:
        gzLogVerbosity = '3'
    else:
        gzLogVerbosity = '1'

    actions = []
    if verbose or debug:
        actions.append(LogInfo(
            msg='spawning %s from %s in world %s' % (name, xacro_file, world)))

    actions.append(SetEnvironmentVariable('GZ_LOG_VERBOSITY', gzLogVerbosity))
    # The create node of ros_gz_sim converts the xacro-expanded URDF into SDF
    # and calls the /world/<world>/create service; plugin blocks survive the
    # conversion. String parameters are wrapped in ParameterValue so that the
    # URDF is not parsed as YAML (it has XML characters YAML cannot read).
    # (The GzSpawnModel launch action is not used because it forwards None for
    # every argument it was not given, which makes launch raise
    # "'NoneType' object is not iterable".)
    actions.append(Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        parameters=[{
            'world': ParameterValue(world, value_type=str),
            'file': ParameterValue('', value_type=str),
            'string': ParameterValue(model, value_type=str),
            'topic': ParameterValue('', value_type=str),
            'name': ParameterValue(name, value_type=str),
            'allow_renaming': False,
            'x': x,
            'y': y,
            'z': z,
            'R': roll,
            'P': pitch,
            'Y': yaw,
        }],
    ))

    # ROS 2 <-> gz bridge for the thrusters: rotor speed commands in, thrust out
    if thrusters:
        arguments = []
        for thruster_id in THRUSTER_IDS:
            prefix = '/%s/thrusters/id_%d' % (name, thruster_id)
            arguments.append(
                prefix + '/input@std_msgs/msg/Float64]gz.msgs.Double')
            arguments.append(
                prefix + '/thrust@geometry_msgs/msg/Vector3[gz.msgs.Vector3d')
        actions.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=arguments,
            output='screen',
        ))

    # Keyboard teleop: reads the terminal of the launch process and drives the
    # thrusters through the bridge above. It can also be started on its own:
    #   ros2 run descriptions ball_teleop.py --ros-args -p name:=<name>
    if teleop:
        if not thrusters:
            actions.append(LogInfo(
                msg='teleop:=true needs thrusters:=true: the teleop node '
                    'publishes on the bridged thruster topics'))
        teleop_parameters = {'name': name}
        if max_rpm:
            teleop_parameters['max_rpm'] = float(max_rpm)
        if step_rpm:
            teleop_parameters['step_rpm'] = float(step_rpm)
        actions.append(Node(
            package='descriptions',
            executable='ball_teleop.py',
            name='ball_teleop',
            parameters=[teleop_parameters],
            output='screen',
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='empty_underwater',
            description='Name of the running Gazebo Sim world'),
        DeclareLaunchArgument(
            'xacro', default_value=DEFAULT_XACRO,
            description='Robot description (xacro) to spawn'),
        DeclareLaunchArgument(
            'name', default_value='ball',
            description='Name of the spawned entity, also used as the link '
                        'prefix inside the model'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='-20.0'),
        DeclareLaunchArgument('roll', default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('thrusters', default_value='true',
                              description='Bridge the thruster command / '
                                          'thrust topics to ROS 2'),
        DeclareLaunchArgument('teleop', default_value='true',
                              description='Start the keyboard teleop node '
                                          '(needs a terminal and needs '
                                          'thrusters:=true); set teleop:=false '
                                          'to spawn only and run the node '
                                          'separately with `ros2 run '
                                          'descriptions ball_teleop.py`'),
        DeclareLaunchArgument('max_rpm', default_value='',
                              description='Full throttle of one thruster, in '
                                          'rpm; empty keeps the default of '
                                          'ball_teleop.py'),
        DeclareLaunchArgument('step_rpm', default_value='',
                              description='Rotor speed added by every key press, '
                                          'in rpm; empty keeps the default of '
                                          'ball_teleop.py'),
        DeclareLaunchArgument('verbose', default_value='false',
                              description='Verbose gz transport output'),
        DeclareLaunchArgument('debug', default_value='false',
                              description='Debug gz transport output '
                                          '(implies verbose)'),
        OpaqueFunction(function=launch_setup),
    ])
