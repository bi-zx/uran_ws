from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('uran_move')

    return LaunchDescription([
        DeclareLaunchArgument('log_level', default_value='info'),
        Node(
            package='uran_move',
            executable='uran_move_node',
            name='uran_move_node',
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        ),
    ])
