from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import TextSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('simulator_simulation'),
            'worlds',
            'SAUVC_WORLD.sdf',
        ]),
        description='Path to Gazebo world file',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=TextSubstitution(text='true'),
        description='Use simulation time',
    )

    stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('simulator_launch'),
            'launch',
            'mission_sauvc.launch.py',
        ])),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        stack_launch,
    ])
