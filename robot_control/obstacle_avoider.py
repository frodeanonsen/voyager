import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        # Subscribe to ultrasonic range data
        self.subscriber = self.create_subscription(Range, '/ultrasonic_range', self.range_callback, 10)
        # Publisher for movement commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.safe_distance = 0.2  # Minimum distance in meters
        self.turning = False
        self.turn_timer = self.create_timer(0.1, self.turn_control)
        self.stop_turning_after = 2.0  # seconds
        self.turn_start_time = None
        self.get_logger().info('Initialized ObstacleAvoider')

    def range_callback(self, msg):
        self.get_logger().info(f'range_callback: {msg}')
        twist = Twist()
        if not self.turning:
            if msg.range > self.safe_distance:
                twist.linear.x = 0.3  # Move forward
                twist.angular.z = 0.0  # No turning
            else:
                twist.linear.x = 0.0  # Stop
                self.turning = True
                self.turn_start_time = self.get_clock().now()
        self.publisher.publish(twist)

    def turn_control(self):
        """Handle turning logic when in 'turning' state."""
        if self.turning and self.turn_start_time:
            twist = Twist()
            elapsed_time = (self.get_clock().now() - self.turn_start_time).nanoseconds / 1e9
            if elapsed_time < self.stop_turning_after:
                twist.angular.z = 0.3  # Turn at a fixed rate
                twist.linear.x = 0.0
                self.publisher.publish(twist)
            else:
                self.turning = False  # Stop turning after the duration

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

