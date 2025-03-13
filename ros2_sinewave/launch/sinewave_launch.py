#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_sinewave',
            executable='sinewave_publisher',
            name='sinewave_publisher',
            output='screen'

        ),
        Node(
            package='ros2_sinewave',
            executable='sinewave_subscriber',
            name='sinewave_subscriber',
            output='screen'

        )
    ])
    
if __name__ == '__main__':
    generate_launch_description()