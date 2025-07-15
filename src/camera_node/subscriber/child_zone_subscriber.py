import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class ChildZoneSubscriber(Node):
    def __init__(self):
        super().__init__('child_zone_subscriber')

        # Subscribe: 카메라 이미지
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        # Publish: 어린이보호구역 감지 여부
        self.publisher_ = self.create_publisher(Bool, '/child_zone_detected', 10)

        self.br = CvBridge()
        self.get_logger().info('🚸 Child Zone Subscriber Node Started!')

    def image_callback(self, data):
        # ROS Image → OpenCV BGR 이미지
        frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # ROI: 아래쪽 1/2만 사용 (필요 없으면 삭제 가능)
        height = frame.shape[0]
        roi = frame[int(height * 0.5):, :]

        # BGR → HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # 빨간색 범위 (두 구간으로 나눠서 잡기)
        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 70, 50])
        upper_red2 = np.array([180, 255, 255])

        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        # 빨간색 비율 계산
        red_pixels = np.sum(mask_red > 0)
        total_pixels = mask_red.size
        red_ratio = red_pixels / total_pixels

        # 감지 기준 (예: 30% 이상이면 어린이보호구역)
        is_child_zone = red_ratio > 0.3

        # Publish
        msg = Bool()
        msg.data = is_child_zone
        self.publisher_.publish(msg)

        self.get_logger().info(f'Red Ratio: {red_ratio:.3f}, Child Zone: {is_child_zone}')

        # (선택) 시각화
        cv2.imshow('Child Zone Detection', roi)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ChildZoneSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C detected, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
