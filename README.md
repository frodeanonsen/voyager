# Voyager Robot Project

This project is a ROS2-based robot control system for the Voyager robot. The system includes various nodes for controlling different aspects of the robot, such as motion, emotion display, obstacle avoidance, and more.

## Project Structure

```
launch/
    robot_display.py
    robot_launch.py
test/
voyager/
    emotion_control.py
    eyes_control.py
    obstacle_avoider.py
    range_sensor.py
    robot_mode.py
    tread_control.py
```

## Nodes

### Emotion Control Node

This node controls the emotional state of the robot based on sensor input. It publishes the current emotion to the `/emotion` topic.

### Eyes Control Node

This node displays the robot's eyes on a screen using Pygame. It subscribes to the `/emotion` topic to update the eye display based on the robot's emotional state.

### Obstacle Avoider Node

This node controls the robot's movement to avoid obstacles. It subscribes to the `/ultrasonic_range` and `/robot_mode` topics and publishes movement commands to the `/cmd_vel` topic.

### Range Sensor Node

This node reads data from an ultrasonic range sensor and publishes the distance measurements to the `/ultrasonic_range` topic.

### Robot Mode Node

This node toggles the robot's mode between manual and autonomous based on input from a joystick. It subscribes to the `/joy` topic and publishes the current mode to the `/robot_mode` topic.

### Tread Control Node

This node controls the robot's treads using GPIO pins on a Raspberry Pi. It subscribes to the `/cmd_vel` topic to receive velocity commands and translates them into motor control signals.

## Launch Files

### Robot Display Launch

This launch file starts the `robot_state_publisher` and `rviz2` nodes to visualize the robot's state and display.

### Robot Launch

This launch file starts various nodes required for the robot's operation, including `obstacle_avoider`, `range_sensor`, `tread_control`, `robot_mode`, `emotion_control`, and `eyes_control`.

## Setup

To set up the project, follow the ROS2 Jazzy instructions.

## Running the Project

To run the project, use the following command:

```sh
ros2 launch voyager robot_launch.py
```

## Testing

The project includes tests for code style and compliance with PEP257 and Flake8 standards. To run the tests, use the following command:

```sh
colcon test
```

## License

This project is licensed under the MIT License. See the LICENSE file for details.
