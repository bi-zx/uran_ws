from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('log_level', default_value='info'),
        Node(
            package='uran_autotask',
            executable='uran_autotask_node',
            name='uran_autotask_node',
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        ),
    ])
