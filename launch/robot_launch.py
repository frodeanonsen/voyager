from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the obstacle avoider node
        Node(
            package='robot_control',
            executable='obstacle_avoider',
            name='obstacle_avoider',
            output='screen'
        ),
        # Launch the range sensor node
        Node(
            package='robot_control',
            executable='range_sensor',
            name='range_sensor',
            output='screen'
        ),
        # Launch the tread control node
        Node(
            package='robot_control',
            executable='tread_control',
            name='tread_control',
            output='screen'
        ),
    ])

