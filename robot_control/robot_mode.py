import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

class RobotMode(Node):
    def __init__(self):
        super().__init__('robot_mode')
        
        # Subscribe to the /joy topic
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Publisher for your robot mode
        self.publisher = self.create_publisher(Bool, 'robot_mode', 10)

        # Track the current mode (False or True)
        self.current_mode = False

        # Common index for the X button on some Xbox controllers
        # Adjust if your X button index is different
        self.x_button_idx = 2
        
        self.get_logger().info('Initialized RobotMode')


    def joy_callback(self, msg: Joy):
        # Check if the X button is pressed
        if msg.buttons[self.x_button_idx] == 1:
            # Toggle the robot mode
            self.current_mode = not self.current_mode
            mode_msg = Bool(data=self.current_mode)
            self.publisher.publish(mode_msg)
            self.get_logger().info(f"Robot mode toggled to: {self.current_mode}")


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    node = RobotMode()
    # Spin so this node will process callbacks
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
