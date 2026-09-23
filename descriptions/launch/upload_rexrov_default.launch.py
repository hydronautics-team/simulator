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
# The built-in IMU of the robot (see ball.xacro) is published on the gz topic
# /<name>/sensors/imu and is bridged to ROS 2 as sensor_msgs/Imu; the front
# camera is published on /<name>/sensors/camera/front (sensor_msgs/Image) with
# the calibration on /<name>/sensors/camera/front/camera_info
# (sensor_msgs/CameraInfo); the water pressure sensor (libwater_pressure.so) is
# published on /<name>/sensors/pressure as sensor_msgs/FluidPressure.
#
# Usage:
#   ros2 launch descriptions upload_rexrov_default.launch.py                # ball
#   ros2 launch descriptions upload_rexrov_default.launch.py name:=rov z:=-30
#
# The keyboard teleop is not part of this launch: start it in its own terminal
# (it needs thrusters:=true, which is the default):
#   ros2 run descriptions ball_teleop.py --ros-args -p name:=ball
# w/s/a/d or the arrow keys drive the robot, space stops it and q quits; every
# key event adds a step to the rotor speeds, so the robot keeps its throttle
# after the keys are released.
#
# Debug tooling (only with debug:=true):
#   * ground truth pose of the model on /<name>/debug/pose (PoseStamped,
#     published by the debug-only PosePublisher plugin in ball.xacro);
#   * in-scene marker arrow with the resultant thruster force
#     (gazebo_worlds/scripts/debug_markers.py, rendered on /marker);
#   * live matplotlib windows (gazebo_worlds/scripts/debug_plot.py, one
#     subplot per topic) for the requested perspectives, e.g.
#       ros2 launch descriptions upload_rexrov_default.launch.py \
#           debug:=true perspectives:=world_model,odometry
#     The plot configs live in descriptions/config/plots/*.yaml.

import os
import pathlib

import yaml
from ament_index_python.packages import get_package_share_directory
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
    # Comma separated plot configs, e.g. perspectives:=world_model,odometry
    perspectives = Lc('perspectives').perform(context).strip()

    # Expand the xacro now; the same name is used as the link prefix inside the
    # model and as the entity name in the world. The debug argument also gates
    # the debug-only plugins inside the xacro (ground truth pose).
    model = Command([
        'xacro ', xacro_file, ' namespace:=', name,
        ' debug:=', 'true' if debug else 'false',
    ])

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

    # ROS 2 <-> gz bridge: the robot IMU, front camera and water pressure
    # sensor are always bridged; the thruster command / thrust topics are
    # added when thrusters:=true.
    arguments = [
        '/%s/sensors/imu@sensor_msgs/msg/Imu[gz.msgs.IMU' % name,
        '/%s/sensors/camera/front@sensor_msgs/msg/Image[gz.msgs.Image' % name,
        '/%s/sensors/camera/front/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        % name,
        '/%s/sensors/pressure@sensor_msgs/msg/FluidPressure[gz.msgs.FluidPressure'
        % name,
    ]
    if thrusters:
        for thruster_id in THRUSTER_IDS:
            prefix = '/%s/thrusters/id_%d' % (name, thruster_id)
            arguments.append(
                prefix + '/input@std_msgs/msg/Float64]gz.msgs.Double')
            arguments.append(
                prefix + '/thrust@geometry_msgs/msg/Vector3[gz.msgs.Vector3d')
    if debug:
        # Ground truth pose published by the debug-only PosePublisher plugin
        # (see ball.xacro); used by the debug markers and odometry plots.
        arguments.append(
            '/%s/debug/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose' % name)
    actions.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=arguments,
        output='screen',
    ))

    # The keyboard teleop is deliberately not started here: run it separately
    # (ros2 run descriptions ball_teleop.py --ros-args -p name:=<name>), so
    # there is only one publisher of the thruster commands per robot.

    # Debug tooling: everything below runs only with debug:=true.
    #   * in-scene markers (resultant thrust arrow) from gazebo_worlds;
    #   * live matplotlib windows for the requested perspectives, e.g.
    #     perspectives:=world_model,odometry. Each perspective is a YAML file
    #     in descriptions/config/plots/<perspective>.yaml with:
    #       window_title, window_seconds, update_rate, topics: [topic/field...]
    #     ({name} in the topic list is replaced by the robot name; every entry
    #     gets its own subplot).
    if debug:
        actions.append(Node(
            package='gazebo_worlds',
            executable='debug_markers.py',
            parameters=[{'name': name}],
            output='screen',
        ))

        plots_dir = os.path.join(
            get_package_share_directory('descriptions'), 'config', 'plots')
        for perspective in [p.strip() for p in perspectives.split(',')]:
            if not perspective:
                continue
            config_file = os.path.join(plots_dir, perspective + '.yaml')
            if not os.path.isfile(config_file):
                actions.append(LogInfo(
                    msg='perspectives: no plot config %s, skipping'
                        % config_file))
                continue
            with open(config_file, 'r') as handle:
                config = yaml.safe_load(handle) or {}
            topics = [str(topic).format(name=name)
                      for topic in config.get('topics', [])]
            if not topics:
                actions.append(LogInfo(
                    msg='perspectives: %s has no topics, skipping'
                        % config_file))
                continue
            actions.append(Node(
                package='gazebo_worlds',
                executable='debug_plot.py',
                name='debug_plot_%s' % perspective,
                parameters=[{
                    'topics': topics,
                    'window_title': str(config.get('window_title',
                                                   perspective)),
                    'window_seconds': float(config.get('window_seconds', 60.0)),
                    'update_rate': float(config.get('update_rate', 10.0)),
                }],
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
        DeclareLaunchArgument('verbose', default_value='false',
                              description='Verbose gz transport output'),
        DeclareLaunchArgument('debug', default_value='false',
                              description='Start the debug tooling: gz debug '
                                          'output, the ground truth pose '
                                          'bridge, in-scene markers and the '
                                          'matplotlib plot windows requested '
                                          'with perspectives'),
        DeclareLaunchArgument(
            'perspectives', default_value='',
            description='Comma separated plot configs to open when '
                        'debug:=true, e.g. '
                        'perspectives:=world_model,odometry (configs in '
                        'descriptions/config/plots: world_model, odometry, '
                        'mission)'),
        OpaqueFunction(function=launch_setup),
    ])
