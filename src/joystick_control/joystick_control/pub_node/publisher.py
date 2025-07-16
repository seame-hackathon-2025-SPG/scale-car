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
        self.timer = self.create_timer(0.2, self.timer_callback)
    
    
    def timer_callback(self):
        gamepad_input = self.gamepad.read_data()

        throttle_msg = Float32()
        steer_msg = Float32()    

        throttle_msg.data = gamepad_input.analog_stick_right.y
        steer_msg.data = gamepad_input.analog_stick_left.x

        self.steer_msg.publish(steer_msg)
        self.throttle_msg.publish(throttle_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PublisherObject()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()