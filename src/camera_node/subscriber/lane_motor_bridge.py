import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray

class LaneToMotorBridgeNode(Node):
    def __init__(self):
        super().__init__('lane_to_motor_bridge_node')

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/lane_info',
            self.lane_callback,
            10)

        self.throttle_pub = self.create_publisher(Float32, 'throttle', 10)
        self.steer_pub = self.create_publisher(Float32, 'steer', 10)

        self.get_logger().info('🚀 Lane → Motor Bridge Node Started')

    def lane_callback(self, msg):
        if len(msg.data) != 2:
            self.get_logger().warn('Invalid lane_info data.')
            return

        center_x, angle_deg = msg.data

        # 🚗 Steering 계산
        max_steering_angle = 30.0  # 최대 ±30도
        steering = max(min(angle_deg / max_steering_angle, 1.0), -1.0)

        frame_center_x = 320  # 카메라 해상도 (예: 640px) 중앙
        if center_x != -1:
            error_x = (center_x - frame_center_x) / frame_center_x  # -1 ~ +1
            steering -= error_x * 0.5  # ✅ 중심 보정 (오른쪽 치우치면 왼쪽으로)

        # 🚀 Throttle 계산
        throttle = 0.5 if center_x != -1 else 0.0  # 차선 없으면 정지

        # 퍼블리시: throttle
        throttle_msg = Float32()
        throttle_msg.data = float(throttle)
        self.throttle_pub.publish(throttle_msg)
        self.get_logger().info(f'Published throttle: {throttle:.2f}')

        # 퍼블리시: steering
        steer_msg = Float32()
        steer_msg.data = float(steering)
        self.steer_pub.publish(steer_msg)
        self.get_logger().info(f'Published steering: {steering:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = LaneToMotorBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C 감지, 종료합니다.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
