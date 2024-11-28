import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
from time import time

class TreadControlNode(Node):
    """
    Class for controlling robot tread motion using ROS2 and Raspberry Pi GPIO.
    This class creates a ROS2 node to control the movement of a tank-like robot
    by listening to velocity commands and translating them into GPIO signals
    for motor control.
    """

    def __init__(self):
        """
        Initialize the TreadControl node, setup GPIO pins, and create ROS2 subscriptions/timers.
        This function sets up GPIO for motor control, starts PWM, and creates a ROS2 subscription
        to listen to velocity commands. It also initializes a timer to handle inactivity.
        """
        super().__init__('tread_control')
        self.init_motor_pins()

        self.left_track_correction = 1 # Adjust this to < 1 if the tank is turning right while attempting to go in a straight line Try .95 then .90 ect. 
        self.right_track_correction = 1
        self.max_linear_speed = 1.27 # Maximum linear speed in m/s - How fast our tank can actually move- I ran the linear test at full speed for 1 second to see how far the tank traveled.  This probably should be adjusted based on 10 meters or 20 meters vs 1. But it's a start. 
        self.linear_speed_adjusted = 1.0
        # To calculate a new linear speed correction factor:
        # New Correction Factor = Current Correction Factor * (Commanded Distance / Actual Distance Travelled)
        # For example, if the tank is initially set with a correction factor of 0.79,
        # and it moves 1.1 meters when commanded to move 1 meter, the new correction factor is calculated as:
        # new_linear_speed = 0.79 * (1 / 1.1) = .72
        # divide first and then multiply
        
        self.expo_linear = 2
        self.expo_angular = 2
        self.angular_amp = 0.5 # 1.5 # Factor to amplify angular speed- Adjust this if the tank isn't turning like you'd expect

        # Subscribe to the /cmd_vel topic with the message type Twist
        # This subscription will receive velocity commands for the robot
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.tread_callback,
            10)

        # Create a timer to run the callback function at 10Hz
        # This is used to check for inactivity and stop the motors if needed
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Initialize the time of the last message received
        self.last_msg_time = time()


    def init_motor_pins(self):
        self.get_logger().debug("init motor pins")
        # forward, back, pwm (en_a)
        self.left_motor_pins = [23, 24, 18]
        # forward, back, pwm (en_b)
        self.right_motor_pins = [20, 21, 19]

        try:
            GPIO.setmode(GPIO.BCM)
            for pin in self.left_motor_pins + self.right_motor_pins:
                GPIO.setup(pin, GPIO.OUT)

            self.left_pwm = GPIO.PWM(self.left_motor_pins[2], 1000)  # PWM at 1000Hz
            self.right_pwm = GPIO.PWM(self.right_motor_pins[2], 1000)
            self.left_pwm.start(0)
            self.right_pwm.start(0)
            self.get_logger().info("GPIO and PWM successfully initialized.")
        except Exception as e:
            self.get_logger().error(f'Critical error: Failed to setup GPIO pins: {e}')
            raise Exception(f'Failed to load configuration: {e}')

    def tread_callback(self, msg):
        """
        Callback to handle incoming ROS2 messages and control the tank motion.

        Parameters:
        - msg (Twist): The incoming ROS2 message containing the tank's desired motion parameters.
        """

        # Apply an exponential curve to both linear and angular velocities
        self.linear_x = self.apply_exponential_curve(msg.linear.x, exponent=self.expo_linear)
        self.angular_z = self.apply_exponential_curve(msg.angular.z, exponent=self.expo_angular)

        # Limit the linear speed to the maximum speed of the robot
        msg.linear.x = min(msg.linear.x, self.max_linear_speed)  # Ensuring robot doesn't exceed max speed

        # Extract linear and angular velocities from the message
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

        # Apply a correction factor to linear velocity for real-world adjustments
        corrected_linear_x = self.linear_x * self.linear_speed_adjusted

        # Calculate left and right wheel speeds based on linear and angular velocities
        angular_speed_amplified = self.angular_z * self.angular_amp # Enhance the effect of angular velocity
        if corrected_linear_x >= 0:  # Condition for moving forwards or staying stationary
            left_speed = corrected_linear_x - angular_speed_amplified
            right_speed = corrected_linear_x + angular_speed_amplified
        else:  # Condition for moving backwards
            left_speed = corrected_linear_x + angular_speed_amplified
            right_speed = corrected_linear_x - angular_speed_amplified

        # Normalize speeds to be within -100 to 100 range
        left_speed = self.map_range(left_speed, -1.0, 1.0, -100, 100)  # Map to motor speed range
        right_speed = self.map_range(right_speed, -1.0, 1.0, -100, 100)

        # Control the left motor based on calculated speed
        if left_speed > 0:  # Forward condition
            self.drive(self.left_motor_pins, True, False, abs(left_speed))
            self.get_logger().info(f'Left motor driving forward at speed: {abs(left_speed)}')
        elif left_speed < 0:  # Reverse condition
            self.drive(self.left_motor_pins, False, True, abs(left_speed))
            self.get_logger().info(f'Left motor driving backward at speed: {abs(left_speed)}')
        else:  # Stop condition
            self.stop_motors(self.left_motor_pins)
            self.get_logger().info('Left motor stopped')

        # Similarly, control the right motor
        if right_speed > 0:
            self.drive(self.right_motor_pins, True, False, abs(right_speed))
            self.get_logger().info(f'Right motor driving forward at speed: {abs(right_speed)}')
        elif right_speed < 0:
            self.drive(self.right_motor_pins, False, True, abs(right_speed))
            self.get_logger().info(f'Right motor driving backward at speed: {abs(right_speed)}')
        else:
            self.stop_motors(self.right_motor_pins)
            self.get_logger().info('Right motor stopped')

        # Update the time of the last message received
        self.last_msg_time = time()

    def timer_callback(self):
        """
        Callback function executed periodically to stop the motors if no command is received.
        This function checks if a set time has passed since the last command was received.
        If the time exceeds a threshold, it stops the motors to prevent unwanted movement.
        """
        # Calculate elapsed time since the last message was received
        elapsed_time = time() - self.last_msg_time
        # self.get_logger().info(f'Elapsed time since last command: {elapsed_time} seconds')

        # Stop motors if the time since last command exceeds a threshold (0.1 seconds in this case)
        if elapsed_time >= 0.1:
            self.stop_motors(self.left_motor_pins)
            self.stop_motors(self.right_motor_pins)
            # self.get_logger().info('Motors stopped due to inactivity')

    def drive(self, pins, fwd, rev, speed):
        """
        Set motor direction and speed with correction factors applied.

        Parameters:
        - pins: The GPIO pins associated with the motor.
        - fwd: Boolean indicating whether to drive forward.
        - rev: Boolean indicating whether to drive reverse.
        - speed: The speed to drive the motor (0 to 100).
        """
        # Ensure speed is within 0-100 range
        speed = max(min(speed, 100), 0)

        # Apply correction factor based on the motor being controlled
        if pins == self.left_motor_pins:
            speed *= self.left_track_correction
        elif pins == self.right_motor_pins:
            speed *= self.right_track_correction

        # Clamp the speed to the range 0-100 after applying the correction
        # Half speed
        speed = max(min(speed, 100), 0) / 2

        GPIO.output(pins[0], fwd)
        GPIO.output(pins[1], rev)
        
        # Set PWM duty cycle based on corrected speed
        if pins == self.left_motor_pins:
            self.left_pwm.ChangeDutyCycle(speed)
        elif pins == self.right_motor_pins:
            self.right_pwm.ChangeDutyCycle(speed)

        # Log the motor direction and speed
        self.get_logger().info(f'Motor direction: {"forward" if fwd else "backward"}')
        self.get_logger().info(f'Motor speed set to: {speed}')


    def stop_motors(self, pins):
        """
        Stop the motors associated with the given GPIO pins.

        Parameters:
        - pins: The GPIO pins associated with the motor to be stopped.
        
        This function stops a motor by disabling the GPIO outputs and setting the PWM duty cycle to 0.
        """
        # Disable both forward and reverse controls
        GPIO.output(pins[0], False)
        GPIO.output(pins[1], False)

        # Set duty cycle to 0 to stop the motor
        if pins == self.left_motor_pins:
            self.left_pwm.ChangeDutyCycle(0)
        elif pins == self.right_motor_pins:
            self.right_pwm.ChangeDutyCycle(0)

    def set_motor_speed(self, pwm, speed):
        duty_cycle = max(0, min(100, abs(speed) * 30)) # 30% vs 100%
        pwm.ChangeDutyCycle(duty_cycle)

    def map_range(self, value, in_min, in_max, out_min, out_max):
        """
        Map a value from one range to another.

        Parameters:
        - value: The value to be mapped.
        - in_min: Minimum value of the input range.
        - in_max: Maximum value of the input range.
        - out_min: Minimum value of the output range.
        - out_max: Maximum value of the output range.
        
        Returns:
        - The mapped value, scaled to the output range.
        """
        # Perform linear mapping from one range to another
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    @staticmethod
    def apply_exponential_curve(value, exponent):
        """
        Apply an exponential curve to the input value.

        Parameters:
        - value: The input value to be modified.
        - exponent: The exponent to apply to the curve.

        Returns:
        - The value after applying the exponential curve.
        """
        sign = 1 if value >= 0 else -1  # Preserve the sign of the original value
        return sign * (abs(value) ** exponent)

    def __del__(self):
        """
        Destructor to clean up GPIO pins when the object is deleted.
        This method is called when the instance is being destroyed to ensure proper cleanup.
        """
        self.get_logger().info('Cleaning up GPIO pins.')
        GPIO.cleanup()  # Release all GPIO resources

def main(args=None):
    """
    Main function to initialize the ROS2 node and run the TreadControlNode class.
    This function sets up ROS2, creates an instance of the TreadControlNode class, and keeps it running.
    """
    rclpy.init(args=args)  # Initialize ROS 2
    node = TreadControlNode()  # Create an instance of the TreadControlNode class
    rclpy.spin(node)  # Keep the node active

    # Destroy the node explicitly (optional but recommended for clean exit)
    node.destroy_node()
    rclpy.shutdown()  # Shutdown ROS 2

if __name__ == '__main__':
    main()

