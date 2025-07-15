import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Bool

class LaneMotorBridgeNode(Node):
    def __init__(self):
        super().__init__('lane_motor_bridge_node')

        # Subscribe
        self.lane_sub = self.create_subscription(
            Float32MultiArray,
            '/lane_info',
            self.lane_callback,
            10)
        self.child_zone_sub = self.create_subscription(
            Bool,
            '/child_zone_detected',
            self.child_zone_callback,
            10)
        self.stop_line_sub = self.create_subscription(
            Bool,
            '/stop_line_detected',
            self.stop_line_callback,
            10)

        # Publishers
        self.throttle_pub = self.create_publisher(Float32, 'throttle', 10)
        self.steer_pub = self.create_publisher(Float32, 'steer', 10)

        # 상태 저장
        self.child_zone = False
        self.stop_line = False
        self.get_logger().info('🚀 Lane + Child Zone + Stop Line → Motor Bridge Node Started')

    def child_zone_callback(self, msg):
        self.child_zone = msg.data
        self.get_logger().info(f'Child Zone Detected: {self.child_zone}')

    def stop_line_callback(self, msg):
        self.stop_line = msg.data
        self.get_logger().info(f'Stop Line Detected: {self.stop_line}')

    def lane_callback(self, msg):
        if len(msg.data) != 2:
            self.get_logger().warn('Invalid lane_info data.')
            return

        center_x, angle_deg = msg.data

        # Steering 계산
        max_steering_angle = 30.0  # 최대 ±30도
        steering = max(min(angle_deg / max_steering_angle, 1.0), -1.0)

        frame_center_x = 320  # 예: 640px / 2
        if center_x != -1:
            error_x = (center_x - frame_center_x) / frame_center_x  # -1 ~ +1
            steering -= error_x * 0.5  # 중심 보정

        # Throttle 결정
        if self.stop_line:
            throttle = 0.0
            reason = "stop_line"
        elif center_x == -1:
            throttle = 0.0
            reason = "no_lane"
        elif self.child_zone:
            throttle = 0.1
            reason = "child_zone"
        else:
            throttle = 0.5
            reason = "normal"

        # ✅ 이유 출력 (로깅)
        self.get_logger().info(f"Throttle reason: {reason}, value: {throttle:.2f}")

        # Publish: throttle
        throttle_msg = Float32()
        throttle_msg.data = float(throttle)
        self.throttle_pub.publish(throttle_msg)
        self.get_logger().info(f'Published throttle: {throttle:.2f}')

        # Publish: steering
        steer_msg = Float32()
        steer_msg.data = float(steering)
        self.steer_pub.publish(steer_msg)
        self.get_logger().info(f'Published steering: {steering:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = LaneMotorBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C 감지, 종료합니다.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
