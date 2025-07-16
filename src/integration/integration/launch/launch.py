from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_controller',
            executable='motor_controller_sub_node',
            name='motor_controller_subscriber',
            output='screen'
        ),
        Node(
            package='camera_node',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen'
        ),
        Node(
            package='camera_node',
            executable='lane_motor_bridge',
            name='lane_motor_bridge',
            output='screen'
        ),
    ])