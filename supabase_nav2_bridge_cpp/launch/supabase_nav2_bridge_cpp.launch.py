from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('db_url', default_value=''),
        DeclareLaunchArgument('table', default_value='navigation_goals'),
        DeclareLaunchArgument('default_frame_id', default_value='map'),
        DeclareLaunchArgument('poll_hz', default_value='2.0'),
        DeclareLaunchArgument('nav2_namespace', default_value=''),
        DeclareLaunchArgument('goal_timeout_sec', default_value='0.0'),

        Node(
            package='supabase_nav2_bridge_cpp',
            executable='bridge_node',
            name='supabase_nav2_bridge_cpp',
            output='screen',
            parameters=[{
                'db_url': LaunchConfiguration('db_url'),
                'table': LaunchConfiguration('table'),
                'default_frame_id': LaunchConfiguration('default_frame_id'),
                'poll_hz': LaunchConfiguration('poll_hz'),
                'nav2_namespace': LaunchConfiguration('nav2_namespace'),
                'goal_timeout_sec': LaunchConfiguration('goal_timeout_sec'),
            }],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
    ])
