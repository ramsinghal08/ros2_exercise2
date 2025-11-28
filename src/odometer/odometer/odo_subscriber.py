import rclpy
from rclpy.node import Node
from custom_msg.msg import RoverOdometry
import math

class OdometrySubscriber(Node):
    def __init__(self):
        super().__init__('odometry_subscriber')
        self.subscription = self.create_subscription(RoverOdometry, 'rover_odo', self.listener_callback, 10)
        self.x = 0.0
        self.y = 0.0

    def listener_callback(self, msg):
        v = msg.linear_velocity.linear.x
        omega = msg.angular_velocity
        theta = msg.orientation
        dt = 1.0
        self.x += v * math.cos(theta) * dt
        self.y += v * math.sin(theta) * dt
        if v > 3.0:
            self.get_logger().warn(f"Warning : Speed limit exceeded")
        self.get_logger().info(
            f"Rover {msg.rover_id}\n"
            f"Position (X, Y): ({self.x}, {self.y})\n"
            f"Orientation : {theta} rad\n"
            f"Linear Vel : {v} m/s\n"
            f"Angular Vel : {omega} rad/s\n"
        )

def main(args=None):
    rclpy.init(args=args)
    node = OdometrySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()