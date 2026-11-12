#!/usr/bin/env python3
"""Start the SAUVC world and expose the robot through the Stingray API."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_worlds = get_package_share_directory('gazebo_worlds')
    descriptions = get_package_share_directory('descriptions')
    world_launch = os.path.join(
        gazebo_worlds, 'launch', 'empty_underwater_world.launch.py')
    spawn_launch = os.path.join(
        descriptions, 'launch', 'upload_rexrov_default.launch.py')
    world_file = PathJoinSubstitution(
        [gazebo_worlds, 'worlds', 'sauvc_2027.world'])

    sim_parameters = {'use_sim_time': True}
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('debug', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(world_launch),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
                'paused': LaunchConfiguration('paused'),
                'debug': LaunchConfiguration('debug'),
                'world': world_file,
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn_launch),
            launch_arguments={
                'world': 'sauvc_2027',
                'name': 'stingray',
                'x': '0.0', 'y': '0.0', 'z': '-2.0',
                'yaw': '0.0',
                'thrusters': 'true',
                'debug': LaunchConfiguration('debug'),
                'stingray_contract': 'true',
            }.items()),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'),
        Node(
            package='stingray_sim', executable='state_adapter.py',
            parameters=[sim_parameters, {'zero_at_start': True}],
            output='screen'),
        Node(
            package='stingray_sim', executable='control_adapter.py',
            parameters=[sim_parameters], output='screen'),
    ])
