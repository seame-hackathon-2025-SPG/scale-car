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

        self.get_logger().info('🚦 Stop Line Detector Node (HSV) Started')

    def image_callback(self, data):
        # BGR 이미지로 변환
        frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        height, width, _ = frame.shape

        # ROI: 아래쪽 1/2 영역
        roi = frame[int(height * 0.5):, :]

        # BGR → HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # HSV에서 흰색 범위 지정 (Hue: 전체, Saturation: 낮음, Value: 높음)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 50, 255])

        # 흰색 마스크
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        # ⚡ Canny edge (흰색 부분만)
        edges = cv2.Canny(mask_white, 50, 150)

        # ⚡ HoughLinesP로 수평선 찾기
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, minLineLength=50, maxLineGap=10)
        stop_line_detected = False

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = (y2 - y1) / (x2 - x1 + 1e-6)  # ZeroDivision 방지
                if abs(slope) < 0.1:  # 수평선만
                    stop_line_detected = True
                    break

        # 퍼블리시
        msg = Bool()
        msg.data = stop_line_detected
        self.publisher_.publish(msg)

        # (옵션) 디버그용 시각화
        cv2.imshow('HSV White Mask', mask_white)  # 흰색만 남은 화면 (나머지는 검정)
        cv2.imshow('Stop Line Edges', edges)      # 흰색에서 추출한 에지
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
