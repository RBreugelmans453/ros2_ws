import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class MqttHandler(Node):
    def __init__(self):
        super().__init__('mqtt_handler')
        self.create_subscription(String, 'mqtt', self.mqtt_callback, 10) #change topics to the ones from the mqtt client
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10) #change topics to the ones to the mqtt client
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)

    def publish1(self, msg):
        # add message type
        self.publisher_.publish(msg)

    def publish2(self, msg):
        # add message type
        self.publisher_.publish(msg)
    
    def mqtt_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        #add conversion to the correct message type and publish it to the correct topic
    
    def odom_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        #add conversion to the correct message type and publish it to the correct topic

def main(args=None):
    rclpy.init(args=args)

    mqtt_handler = MqttHandler()

    rclpy.spin(mqtt_handler)

    mqtt_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        
