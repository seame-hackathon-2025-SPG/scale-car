import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import time

class StopLineDetectorNode(Node):
    def __init__(self):
        super().__init__('stop_line_detector_node')

        self.subscriber = self.create_subscription(
            Bool,
            '/camera/trigger_stop_line',
            self.trigger_callback,
            10
        )

        # motor_cmd publisher (String type)
        self.motor_publisher = self.create_publisher(String, 'motor_cmd', 10)
        self.get_logger().info("StopLineDetectorNode is ready.")

    def trigger_callback(self, msg):
        if msg.data:
            self.get_logger().info('Trigger received: stopping vehicle indefinitely.')
            stop_msg = String()
            stop_msg.data = "speed:0.0"
            self.motor_publisher.publish(stop_msg)
            # 더 이상 resume_callback이나 타이머를 사용하지 않음

def main(args=None):
    rclpy.init(args=args)
    node = StopLineDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
