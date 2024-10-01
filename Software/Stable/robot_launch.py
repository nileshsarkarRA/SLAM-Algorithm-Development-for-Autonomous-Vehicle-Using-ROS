from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='runs',
            executable='lidar_node',
            name='lidar_node',
            output='screen'
        ),
        Node(
            package='runs',
            executable='motor_control',
            name='motor_control_node',
            output='screen'
        ),
        Node(
            package='runs',
            executable='camera_node',
            name='camera_node',
            output='screen'
        )
    ])
