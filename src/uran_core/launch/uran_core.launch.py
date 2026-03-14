from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('uran_core')

    return LaunchDescription([
        DeclareLaunchArgument('log_level', default_value='info'),
        Node(
            package='uran_core',
            executable='uran_core_node',
            name='uran_core_node',
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        ),
    ])
