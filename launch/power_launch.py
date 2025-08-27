from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='power_nodes',
            executable='power',
            name='power_coordinator',
            output='screen'
        ),
        Node(
            package='power_nodes',
            executable='serial_bridge',
            name='power_bridge',
            output='screen'
        )
    ])
