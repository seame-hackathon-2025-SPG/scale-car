import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.publisher_ = self.create_publisher(
            Float32MultiArray,  # [center_x, angle_deg]
            '/lane_info',
            10)

        self.br = CvBridge()
        self.get_logger().info('🚗 Lane Detection Node Started')

        # ✅ fallback용 이전 값 저장 변수
        self.last_center_x = -1
        self.last_angle_deg = 0

    def image_callback(self, data):
        frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        height, width, _ = frame.shape
        self.get_logger().info(f'height: {height}, width: {width}')

        # 1️⃣ ROI: 아래쪽 절반
        roi = frame[int(height * 1/2):, :]

        # 2️⃣ grayscale → blur → Canny edge
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 45, 120)

        # 3️⃣ HoughLinesP
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 30, minLineLength=80, maxLineGap=100)
        left_slopes, right_slopes, left_x, right_x = [], [], [], []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = (y2 - y1) / (x2 - x1 + 1e-6)  # prevent division by zero
                if abs(slope) < 0.5:  # 수평선 무시
                    continue
                if x1 < width / 2 and x2 < width / 2:
                    left_slopes.append(slope)
                    left_x.extend([x1, x2])
                elif x1 > width / 2 and x2 > width / 2:
                    right_slopes.append(slope)
                    right_x.extend([x1, x2])

        if left_x and right_x:
            left_mean_x = np.mean(left_x)
            right_mean_x = np.mean(right_x)
            center_x = (left_mean_x + right_mean_x) / 2
            mean_slope = (np.mean(left_slopes) + np.mean(right_slopes)) / 2
            angle_deg = np.degrees(np.arctan(mean_slope))

            # ✅ fallback 값 업데이트
            self.last_center_x = center_x
            self.last_angle_deg = angle_deg

            self.get_logger().info(f'Center X: {center_x:.2f}, Angle: {angle_deg:.2f}')
        else:
            # ✅ fallback 값 사용
            center_x = self.last_center_x
            angle_deg = self.last_angle_deg
            self.get_logger().warn('차선을 찾지 못했습니다. 이전 값 유지합니다.')

        # 4️⃣ Publish [center_x, angle_deg]
        msg = Float32MultiArray()
        msg.data = [float(center_x), float(angle_deg)]
        self.publisher_.publish(msg)

        # (선택) 시각화
        debug = roi.copy()
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = (y2 - y1) / (x2 - x1 + 1e-6)

                if abs(slope) < 0.5:
                    continue
                elif abs(slope) >= 0.5:
                    if x1 < width / 2 and x2 < width / 2:
                        color = (255, 0, 0)  # 파란색 (왼쪽 차선)
                        cv2.line(debug, (x1, y1), (x2, y2), color, 2)
                    elif x1 > width / 2 and x2 > width / 2:
                        color = (0, 0, 255)  # 빨간색 (오른쪽 차선)
                        cv2.line(debug, (x1, y1), (x2, y2), color, 2)

        cv2.imshow('Hough', debug)
        cv2.imshow('Lane Detection', edges)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C 감지, 종료합니다.')
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
