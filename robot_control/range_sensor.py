import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import RPi.GPIO as GPIO
import time

class RangeSensorNode(Node):
    def __init__(self):
        super().__init__('range_sensor')
        self.publisher = self.create_publisher(Range, 'ultrasonic_range', 10)
        self.timer = self.create_timer(0.1, self.read_sensor)
        self.init_sensor_pins()

    def init_sensor_pins(self):
        self.trig_pin = 13
        self.echo_pin = 6
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trig_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)
        GPIO.output(self.trig_pin, False)

    def read_sensor(self):
        GPIO.output(self.trig_pin, True)
        time.sleep(0.00001)
        GPIO.output(self.trig_pin, False)

        pulse_start = 0
        pulse_end = 0
        while GPIO.input(self.echo_pin) == 0:
            pulse_start = time.time()
        while GPIO.input(self.echo_pin) == 1:
            pulse_end = time.time()

        duration = pulse_end - pulse_start
        distance = duration * 34300 / 2  # Speed of sound: 343 m/s
        range_msg = Range()
        range_msg.header.stamp = self.get_clock().now().to_msg()
        range_msg.range = distance / 100.0  # Convert to meters
        self.publisher.publish(range_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RangeSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

