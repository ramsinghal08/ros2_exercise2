import rclpy
from rclpy.node import Node
from custom_msg.msg import RoverOdometry
from geometry_msgs.msg import Twist
import random

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        self.publisher_ = self.create_publisher(RoverOdometry, 'rover_odo', 10)
        self.timer = self.create_timer(1.0, self.publish_odometry)
        self.rover_id = 0
        self.current_orientation = 0.0

    def publish_odometry(self):
        msg = RoverOdometry()
        msg.rover_id = self.rover_id
        msg.linear_velocity = Twist()
        msg.linear_velocity.linear.x = random.uniform(0.0, 10.0)
        msg.angular_velocity = random.uniform(-5.0, 5.0)
        self.current_orientation = random.uniform(-3.14, 3.14)
        msg.orientation = self.current_orientation
        self.publisher_.publish(msg)
        self.get_logger().info(f'ID: {msg.rover_id}, Vel: {msg.linear_velocity.linear.x}')

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()