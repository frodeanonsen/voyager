import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
import random


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

        # Subscribe to ultrasonic range data and robot_mode changes
        self.sub_mode = self.create_subscription(Bool, '/robot_mode', self.mode_callback, 10)

        # Publisher for movement commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Parameters
        self.safe_distance = 0.3  # Minimum distance to stop
        self.forward_speed = 0.2  # Speed for moving forward
        self.reverse_speed = -0.2 # Speed for reversing
        self.turn_speed = 0.25    # Speed for turning
        self.forward_time = 5.0   # Max time to move forward (seconds)
        self.reverse_time = 1.0   # Time to reverse (seconds)
        self.turn_time = 0.5      # Time to turn (seconds)

        # State
        self.state = 'forward'  # Current state: 'forward' or 'turn'
        self.last_state_change = self.get_clock().now()
        self.turn_direction = 1  # 1 for right, -1 for left (default to right)

        self.autonomy_enabled = False

        self.get_logger().info('Initialized ObstacleAvoider')


    def range_callback(self, msg):
        """Check for obstacles and update behavior."""
        if self.state == 'forward' and msg.range <= self.safe_distance:
            self.get_logger().info(f'Too close: {msg.range}. Reversing and turning.')
            self.state = 'reverse'
            self.last_state_change = self.get_clock().now()
            self.turn_direction = random.choice([-1, 1])  # Randomly choose left or right

    def mode_callback(self, new_mode: Bool):
        self.autonomy_enabled = new_mode.data
        if self.autonomy_enabled:
            self.get_logger().info('Autonomy mode enabled')
            # Subscription to ultrasonic_range changes
            self.sub_range = self.create_subscription(Range, '/ultrasonic_range', self.range_callback, 10)
            # Timer for state management
            self.timer = self.create_timer(0.1, self.state_machine)
        else:
            self.get_logger().info('Autonomy mode disabled')
            if self.timer is not None:
                self.timer.destroy()
            if self.sub_range is not None:
                self.sub_range.destroy()

    def state_machine(self):
        """State machine to control robot behavior."""

        twist = Twist()
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.last_state_change).nanoseconds / 1e9

        if self.state == 'forward':
            self.get_logger().info('forward')
            # Drive forward for a maximum of forward_time seconds
            if elapsed_time < self.forward_time:
                twist.linear.x = self.forward_speed
                twist.angular.z = 0.0
            else:
                self.state = 'turn'
                self.last_state_change = current_time
                self.turn_direction = random.choice([-1, 1])  # Randomly choose left or right

        elif self.state == 'reverse':
            # Reverse for reverse_time seconds
            if elapsed_time < self.reverse_time:
                twist.linear.x = self.reverse_speed
                twist.angular.z = 0.0
            else:
                self.state = 'turn'
                self.last_state_change = current_time
                self.turn_direction = random.choice([-1, 1])  # Randomly choose left or right

        elif self.state == 'turn':
            self.get_logger().info(f'turn {"left" if self.turn_direction < 0 else "right"}')
            # Turn for turn_time seconds
            if elapsed_time < self.turn_time:
                twist.linear.x = 0.0
                twist.angular.z = self.turn_speed * self.turn_direction
            else:
                self.state = 'forward'
                self.last_state_change = current_time

        # Publish the movement command
        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

