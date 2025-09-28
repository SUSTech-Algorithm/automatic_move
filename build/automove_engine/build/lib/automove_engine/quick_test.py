import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class QuickTest(Node):
    def __init__(self):
        super().__init__('quick_test')
        self.get_logger().info('Quick test node started')
        self.publisher_ = self.create_publisher(Twist, '/simple_robot/cmd_vel', 10)

    def publish_twist(self):
        twist = Twist()
        twist.linear.x = 1
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = QuickTest()
    rclpy.spin(node)
    rclpy.shutdown()
