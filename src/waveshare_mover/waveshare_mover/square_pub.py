import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class SquarePublisher(Node):
    def __init__(self):
        super().__init__('square_publisher')
        self.publisher_ = self.create_publisher(Twist, 'waveshare_real_controller/cmd_vel_unstamped', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    # drive forward for 3 seconds
    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.linear.x)
        self.i += 1
        if self.i == 3:
            self.i = 0
            self.timer.cancel()
            self.timer = self.create_timer(0.5, self.timer_callback2)
    
    # rotate for 2 seconds
    def timer_callback2(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 1.0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.angular.z)
        self.i += 1
        if self.i == 2:
            self.i = 0
            self.timer.cancel()
            self.timer = self.create_timer(0.5, self.timer_callback)

def main(args=None):
    rclpy.init(args=args)
    square_publisher = SquarePublisher()
    rclpy.spin(square_publisher)
    square_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()