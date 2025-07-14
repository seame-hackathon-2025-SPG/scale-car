import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class StopLineDetector(Node):
    def __init__(self):
        super().__init__('stop_line_detector')

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.publisher_ = self.create_publisher(Bool, '/stop_line_detected', 10)
        self.br = CvBridge()

        self.get_logger().info('🚦 Stop Line Detector Node Started')

    def image_callback(self, data):
        # BGR 이미지로 변환
        frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        height, width, _ = frame.shape

        # ROI: 아래쪽 1/3 영역
        roi = frame[int(height * 2/3):, :]

        # Grayscale, Blur, Canny
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)

        # HoughLinesP로 직선 검출
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, minLineLength=50, maxLineGap=10)

        stop_line_detected = False

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = (y2 - y1) / (x2 - x1 + 1e-6)  # 1e-6: ZeroDivision 방지
                # 수평선 (slope ≈ 0) 판별
                if abs(slope) < 0.1:
                    stop_line_detected = True
                    break  # 하나만 발견해도 True로

        # 퍼블리시
        msg = Bool()
        msg.data = stop_line_detected
        self.publisher_.publish(msg)

        # (옵션) 디버그용 화면 출력
        cv2.imshow('Stop Line ROI', edges)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = StopLineDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C로 종료됩니다.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
