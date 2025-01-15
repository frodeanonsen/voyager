import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from robot_control.msg import Emotion
import time

class EmotionNode(Node):
    def __init__(self):
        super().__init__('emotion_control')
        self.publisher = self.create_publisher(Emotion, 'emotion', 10)
        self.sub_range = self.create_subscription(
            Range, "/ultrasonic_range", self.range_callback, 10
        )
        self.timer = self.create_timer(60, self.sleep)
        self.last_emote = self.get_clock().now()
        self.safe_distance = 0.3
        self.current_emotion = Emotion.NEUTRAL
        self.publish_emotion()
    
    def sleep(self):
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.last_state_change).nanoseconds / 1e9
        if elapsed_time > 30:
            self.current_emotion = Emotion.SLEEP
            self.publish_emotion()

    def publish_emotion(self):
        msg = Emotion()
        msg.emotion = self.current_emotion
        self.publisher.publish(msg)
        self.get_logger().info(f"Published emotion message {msg}")
        self.last_state_change = self.get_clock().now()

    def range_callback(self, msg):
        self.get_logger().info(f"range: {msg}")
        if msg.range <= self.safe_distance:
            self.current_emotion = Emotion.ANGRY
            self.publish_emotion()




def main(args=None):
    rclpy.init(args=args)
    node = EmotionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


