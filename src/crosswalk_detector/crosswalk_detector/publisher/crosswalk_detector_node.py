import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import time

class CrosswalkDetectorNode(Node):
    def __init__(self):
        super().__init__('crosswalk_detector_node')

        self.subscriber = self.create_subscription(
            Bool,
            '/camera/trigger_crosswalk',
            self.trigger_callback,
            10
        )

        # motor_cmd publisher (String type)
        self.motor_publisher = self.create_publisher(String, 'motor_cmd', 10)
        self.timer_running = False
        self.get_logger().info("CrosswalkDetectorNode is ready.")

    def trigger_callback(self, msg):
        if msg.data and not self.timer_running:
            self.get_logger().info('Trigger received: stopping vehicle for 5 seconds.')
            # Publish speed:0.0 to motor_cmd
            stop_msg = String()
            stop_msg.data = "speed:0.0"
            self.motor_publisher.publish(stop_msg)
            self.timer_running = True
            # Wait 5 seconds, then publish speed:1.0
            def resume_callback():
                resume_msg = String()
                resume_msg.data = "speed:1.0"
                self.motor_publisher.publish(resume_msg)
                self.get_logger().info('Stop complete: resuming movement.')
                self.timer_running = False
            self.create_timer(5.0, resume_callback)

def main(args=None):
    rclpy.init(args=args)
    node = CrosswalkDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
