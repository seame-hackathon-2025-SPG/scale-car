import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from piracer.vehicles import PiRacerPro

steer_alignment = -0.2
throttle_threshold = 0.4

class SubscriberObject(Node):
    def __init__(self):
        super().__init__('subscriber')

        self.piracer = PiRacerPro();

        self.create_subscription(Float32, 'steer', self.steer_callback, 10)
        self.create_subscription(Float32, 'throttle', self.throttle_callback, 10)

    def steer_callback(self, steer_msg):
        steer_value = steer_msg.data + steer_alignment
        self.get_logger().info(f'steer: {steer_value}')
        self.piracer.set_steering_percent(steer_value)

    def throttle_callback(self, throttle_msg):
        throttle_value = throttle_msg.data * throttle_threshold
        self.get_logger().info(f'throttle: {throttle_value}')
        self.piracer.set_throttle_percent(throttle_value)

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberObject()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()