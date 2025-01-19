from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision_cameras',
            namespace='vision_cameras',
            executable='calibration',
            name='calibration'
        )
    ])
