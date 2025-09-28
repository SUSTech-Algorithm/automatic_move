#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SimpleRobotController(Node):
    def __init__(self):
        super().__init__('simple_robot_controller')
        self.publisher_ = self.create_publisher(Twist, '/simple_robot/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.move_count = 0

    def timer_callback(self):
        msg = Twist()

        # 根据计数器发送不同的移动命令
        if self.move_count == 0:
            # 前进
            msg.linear.x = 0.5
            self.get_logger().info('Moving forward')
        elif self.move_count == 1:
            # 停止
            msg.linear.x = 0.0
            self.get_logger().info('Stopping')
        elif self.move_count == 2:
            # 后退
            msg.linear.x = -0.5
            self.get_logger().info('Moving backward')
        elif self.move_count == 3:
            # 停止
            msg.linear.x = 0.0
            self.get_logger().info('Stopping')
        elif self.move_count == 4:
            # 左移
            msg.linear.y = 0.5
            self.get_logger().info('Moving left')
        elif self.move_count == 5:
            # 停止
            msg.linear.y = 0.0
            self.get_logger().info('Stopping')
        elif self.move_count == 6:
            # 右移
            msg.linear.y = -0.5
            self.get_logger().info('Moving right')
        elif self.move_count == 7:
            # 停止
            msg.linear.y = 0.0
            self.get_logger().info('Stopping')
            # 重置计数器以重复循环
            self.move_count = -1

        self.publisher_.publish(msg)
        self.move_count += 1

def main(args=None):
    rclpy.init(args=args)
    robot_controller = SimpleRobotController()

    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        pass
    finally:
        # 发送停止命令
        stop_msg = Twist()
        robot_controller.publisher_.publish(stop_msg)
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()