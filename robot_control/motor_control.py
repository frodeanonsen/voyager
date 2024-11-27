import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.motor_callback,
            10)
        self.init_motor_pins()

    def init_motor_pins(self):
        self.get_logger().debug("init motor pins")
        self.left_pwm_pin = 18 # en_a
        self.right_pwm_pin = 19 # en_b
        self.left_motor_pins = [23, 24]
        self.right_motor_pins = [20, 21]
        GPIO.setmode(GPIO.BCM)
        for pin in self.left_motor_pins + self.right_motor_pins:
            GPIO.setup(pin, GPIO.OUT)
        GPIO.setup(self.left_pwm_pin, GPIO.OUT)
        GPIO.setup(self.right_pwm_pin, GPIO.OUT)

        self.left_pwm = GPIO.PWM(self.left_pwm_pin, 100)  # PWM at 100Hz
        self.right_pwm = GPIO.PWM(self.right_pwm_pin, 100)
        self.left_pwm.start(0)
        self.right_pwm.start(0)

    def motor_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        self.get_logger().debug(f"motor_callback: linear={linear} angular={angular}")
        # Simple differential drive logic
        left_speed = linear - angular
        right_speed = linear + angular
        self.set_motor_speed(self.left_pwm, left_speed)
        self.set_motor_speed(self.right_pwm, right_speed)

    def set_motor_speed(self, pwm, speed):
        duty_cycle = max(0, min(100, abs(speed) * 30)) # 30% vs 100%
        pwm.ChangeDutyCycle(duty_cycle)

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

