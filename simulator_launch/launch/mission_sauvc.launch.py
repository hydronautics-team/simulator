# Copyright 2022 Open Source Robotics Foundation, Inc.
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import TextSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    pkg_project_gazebo = get_package_share_directory('simulator_simulation')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    default_world = PathJoinSubstitution([
        pkg_project_gazebo,
        'worlds',
        'SAUVC_WORLD.sdf',
    ])

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Path to Gazebo world file',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=TextSubstitution(text='true'),
        description='Use simulation time for ROS nodes',
    )

    bridge_arguments = [
        '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        '/X3/gazebo/command/twist@geometry_msgs/msg/Twist]gz.msgs.Twist',
        '/model/copter/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/camera/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image',
        '/camera/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        '/stingray/topics/bottom_camera/bottom_camera@vision_msgs/msg/Detection2DArray[gz.msgs.AnnotatedAxisAligned2DBox_V',
        '/stingray/topics/bottom_camera/bottom_camera_image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/stingray/topics/bottom_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        '/stingray/topics/front_camera@vision_msgs/msg/Detection2DArray[gz.msgs.AnnotatedAxisAligned2DBox_V',
        '/stingray/topics/front_camera_image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/stingray/topics/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
    ]

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': LaunchConfiguration('world')}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_arguments,
        output='screen',
    )

    converter = Node(
        package='simulator_perception',
        executable='simulator_perception_node',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        gz_sim,
        bridge,
        converter,
    ])