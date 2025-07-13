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

    def image_callback(self, data):
        frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        height, width, _ = frame.shape

        # 1️⃣ ROI: 아래쪽 절반
        roi = frame[height//2:, :]

        # 2️⃣ grayscale → blur → Canny edge
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)

        # 3️⃣ HoughLinesP
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, minLineLength=50, maxLineGap=50)
        left_slopes, right_slopes, left_x, right_x = [], [], [], []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = (y2 - y1) / (x2 - x1 + 1e-6)  # prevent division by zero
                if abs(slope) < 0.5:  # 수평선 무시
                    continue
                if slope > 0 and x1 < width / 2 and x2 < width / 2:
                    left_slopes.append(slope)
                    left_x.extend([x1, x2])
                elif slope < 0 and x1 > width / 2 and x2 > width / 2:
                    right_slopes.append(slope)
                    right_x.extend([x1, x2])

        if left_x and right_x:
            left_mean_x = np.mean(left_x)
            right_mean_x = np.mean(right_x)
            center_x = (left_mean_x + right_mean_x) / 2
            mean_slope = (np.mean(left_slopes) + np.mean(right_slopes)) / 2
            angle_deg = np.degrees(np.arctan(mean_slope))
            self.get_logger().info(f'Center X: {center_x:.2f}, Angle: {angle_deg:.2f}')
        else:
            center_x, angle_deg = -1, 0  # 실패시 기본값
            self.get_logger().warn('차선을 찾지 못했습니다.')

        # 4️⃣ Publish [center_x, angle_deg]
        msg = Float32MultiArray()
        msg.data = [float(center_x), float(angle_deg)]
        self.publisher_.publish(msg)

        # (선택) 시각화
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
