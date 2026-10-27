def main():
    print('Hi from teleop_control.')

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, tty, termios

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist = Twist()
        self.get_logger().info("键盘控制已启动！按 W/S/A/D 移动，按 Q 退出。")

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def run(self):
        while rclpy.ok():
            key = self.get_key()
            if key == 'w':
                self.twist.linear.x = 0.5
                self.twist.angular.z = 0.0
            elif key == 's':
                self.twist.linear.x = -0.5
                self.twist.angular.z = 0.0
            elif key == 'a':
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.5
            elif key == 'd':
                self.twist.linear.x = 0.0
                self.twist.angular.z = -0.5
            elif key == 'q':
                break
            else:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
            
            self.publisher_.publish(self.twist)
            self.get_logger().info(f'发布: 线速度={self.twist.linear.x:.2f}, 角速度={self.twist.angular.z:.2f}', throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
