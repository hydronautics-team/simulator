#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_simulation = get_package_share_directory('simulator_simulation')
    
    world_path = os.path.join(pkg_simulation, 'worlds', 'aruco_simple.world')
    models_path = os.path.join(pkg_simulation, 'models')
    
    # Устанавливаем путь к моделям
    gz_resource_path = f"{models_path}:$GZ_SIM_RESOURCE_PATH"
    
    # Запуск Gazebo Fortress
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen',
        additional_env={'GZ_SIM_RESOURCE_PATH': gz_resource_path}
    )
    
    # Мост для камеры
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        arguments=['/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'],
        output='screen'
    )
    
    return LaunchDescription([
        gz_sim,
        camera_bridge
    ])
