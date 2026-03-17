from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('uran_media')
    config = os.path.join(share, 'config', 'media.yaml')

    return LaunchDescription([
        Node(
            package='uran_media',
            executable='uran_media_node',
            name='uran_media_node',
            output='screen',
            parameters=[config],
        ),
    ])
