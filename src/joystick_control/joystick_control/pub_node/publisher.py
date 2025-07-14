import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from piracer.gamepads import ShanWanGamepad

class PublisherObject(Node):
    def __init__(self):
        super().__init__('publisher')

        self.gamepad = ShanWanGamepad()

        self.steer_msg = self.create_publisher(Float32, 'steer', 10)
        self.throttle_msg = self.create_publisher(Float32, 'throttle', 10)
        self.timer = self.create_timer(0.125, self.timer_callback)
    
    
    def timer_callback(self):
        throttle_msg = Float32()
        steer_msg = Float32()    

        throttle_msg.data = self.gamepad.analog_stick_right.y
        steer_msg.data = self.gamepad.analog_stick_left.x

        self.get_logger().info(f'Publishing steer: {steer_msg.data}, throttle: {throttle_msg.data}')
        self.steer_msg.publish(throttle_msg)
        self.throttle_msg.publish(throttle_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PublisherObject()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()