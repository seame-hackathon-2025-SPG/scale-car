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
        edges = cv2.Canny(blur, 70, 140)

        # 3️⃣ HoughLinesP
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 30, minLineLength=80, maxLineGap=100)
        left_slopes, right_slopes, left_x, right_x = [], [], [], []

        # all_x = []
        # if lines is not None:
        #     for line in lines:
        #         x1, y1, x2, y2 = line[0]
        #         slope = (y2 - y1) / (x2 - x1 + 1e-6)
        #         if abs(slope) < 0.5:
        #             continue
        #         all_x.extend([x1, x2])

        #     if all_x:
        #         all_mean_x = np.mean(all_x)
        #     else:
        #         all_mean_x = width / 2  # fallback

        #     for line in lines:
        #         x1, y1, x2, y2 = line[0]
        #         slope = (y2 - y1) / (x2 - x1 + 1e-6)
        #         if abs(slope) < 0.5:
        #             continue

        #         if x1 < all_mean_x and x2 < all_mean_x:
        #             left_slopes.append(slope)
        #             left_x.extend([x1, x2])
        #         elif x1 > all_mean_x and x2 > all_mean_x:
        #             right_slopes.append(slope)
        #             right_x.extend([x1, x2])

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
            # 왼/오 모두 인식됨 → 정상 계산
            left_mean_x = np.mean(left_x)
            right_mean_x = np.mean(right_x)
            center_x = (left_mean_x + right_mean_x) / 2
            mean_slope = (np.mean(left_slopes) + np.mean(right_slopes)) / 2
            angle_deg = np.degrees(np.arctan(mean_slope))
            
            self.get_logger().info(
                f'🟢 양쪽 차선 인식 | Left mean X: {left_mean_x:.2f}, Right mean X: {right_mean_x:.2f}, '
                f'Center X: {center_x:.2f}, Angle: {angle_deg:.2f}'
            )
            self.last_center_x = center_x
            self.last_angle_deg = angle_deg

        elif left_x and not right_x:
            # 왼쪽만 인식 → 오른쪽으로 틀어주기
            center_x = np.mean(left_x) + 100  # 오른쪽으로 임의 offset
            angle_deg = 5.0  # 오른쪽으로 5도
            
            self.get_logger().info(
                f'🟡 오른쪽 차선 미인식 → 오른쪽으로 틀기 | Left mean X: {np.mean(left_x):.2f}, '
                f'Adjusted Center X: {center_x:.2f}, Angle: {angle_deg:.2f}'
            )
            self.last_center_x = center_x
            self.last_angle_deg = angle_deg

        elif right_x and not left_x:
            # 오른쪽만 인식 → 왼쪽으로 틀어주기
            center_x = np.mean(right_x) - 100  # 왼쪽으로 임의 offset
            angle_deg = -5.0  # 왼쪽으로 5도
            
            self.get_logger().info(
                f'🟡 왼쪽 차선 미인식 → 왼쪽으로 틀기 | Right mean X: {np.mean(right_x):.2f}, '
                f'Adjusted Center X: {center_x:.2f}, Angle: {angle_deg:.2f}'
            )
            self.last_center_x = center_x
            self.last_angle_deg = angle_deg

        else:
            # 둘 다 없음 → fallback 값
            center_x = self.last_center_x
            angle_deg = self.last_angle_deg
            
            self.get_logger().info('🔴 양쪽 차선 모두 미인식 → 이전 값 유지')


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
