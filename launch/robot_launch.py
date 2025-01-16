from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the game control node
        # Node(
        #     package='voyager',
        #     executable='game_control',
        #     name='game_control',
        #     output='screen'
        # ),
        # Launch the obstacle avoider node
        Node(
            package='voyager',
            executable='obstacle_avoider',
            name='obstacle_avoider',
            output='screen'
        ),
        # Launch the range sensor node
        Node(
            package='voyager',
            executable='range_sensor',
            name='range_sensor',
            output='screen'
        ),
        # Launch the tread control node
        Node(
            package='voyager',
            executable='tread_control',
            name='tread_control',
            output='screen'
        ),
        # Node(
        #     package='teleop_twist_joy',
        #     executable='teleop_node',
        #     name='teleop_twist_joy',
        #     parameters=[{'joy_config': 'p'}]
        # ),
        # Launch the robot mode node
        Node(
            package='voyager',
            executable='robot_mode',
            name='robot_mode',
            output='screen'
        ),
        Node(
            package='voyager',
            executable='emotion_control',
            name='emotion_control',
            output='screen'
        ),
    ])

