import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class RedZoneDetectorNode(Node):
    def __init__(self):
        super().__init__('red_zone_detector_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Float32, '/cmd/speed_adjust', 10)
        self.create_subscription(
            Bool,
            '/child_zone_trigger',
            self.child_zone_callback,
            10)
        self.motor_publisher = self.create_publisher(String, 'motor_cmd', 10)
        self.get_logger().info("RedZoneDetectorNode is ready.")

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 70, 50])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = mask1 | mask2

        red_area = cv2.countNonZero(red_mask)
        total_area = frame.shape[0] * frame.shape[1]
        red_ratio = red_area / total_area

        speed = 1.0
        if red_ratio > 0.1:
            speed = 0.5

        self.publisher.publish(Float32(data=speed))

    def child_zone_callback(self, msg):
        if msg.data:
            speed_msg = String()
            speed_msg.data = "speed:0.5"
            self.motor_publisher.publish(speed_msg)
            self.get_logger().info('Child zone detected: speed reduced to 0.5.')
        # False일 때 동작을 추가하려면 아래에 작성
        # else:
        #     pass

def main(args=None):
    rclpy.init(args=args)
    node = RedZoneDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
