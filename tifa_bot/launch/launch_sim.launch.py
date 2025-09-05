import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess ,DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node


def generate_launch_description():
    package_name='tifa_bot'
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch','rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control' : 'true'}.items()
    )

    default_world = PathJoinSubstitution([
        FindPackageShare(package_name), 'worlds', 'obstacles.world'
    ])

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Path file .world (SDF)'
    )
    world = LaunchConfiguration('world')

    gazebo_params_file = os.path.join(get_package_share_directory(package_name), 'config','gazebo_params.yaml')

    gz = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world,
            '--ros-args', '--params-file', gazebo_params_file
        ],
        output='screen'
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic','robot_description',
            '-entity','tifa_bot'
        ], 
        output='screen',
    )   

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"]
    )

    joint_state_broadcsater_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"]
    )


    return LaunchDescription([
        world_arg,
        rsp,
        gz,
        spawn_entity,
        diff_drive_spawner,
        joint_state_broadcsater_spawner
    ])