import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # 퍼블리셔 생성
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.br = CvBridge()

        # 타이머 생성 (예: 0.1초마다 콜백 → 10 FPS)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # OpenCV 카메라 캡처 (0번 카메라)
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

        if not self.cap.isOpened():
            self.get_logger().error('카메라를 열 수 없습니다.')
            exit(1)
        self.get_logger().info('카메라 노드 시작됨!')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('카메라 프레임을 읽을 수 없습니다.')
            return
        
        # 프레임 크기 출력
        height, width, _ = frame.shape
        self.get_logger().info(f'이미지 퍼블리시 중... (Width: {width}, Height: {height})')

        # 색상 반전
        inverted_frame = cv2.bitwise_not(frame)

        # 반전된 OpenCV BGR 이미지를 ROS Image 메시지로 변환
        img_msg = self.br.cv2_to_imgmsg(inverted_frame, encoding='bgr8')

        # 퍼블리시
        self.publisher_.publish(img_msg)

    def destroy_node(self):
        self.cap.release()
        self.get_logger().info('카메라 종료됨.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        camera_node.get_logger().info('Ctrl+C 감지, 종료합니다.')
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        camera_node.destroy_node()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
