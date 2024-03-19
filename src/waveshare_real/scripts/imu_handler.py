#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import serial
import sensor_msgs.msg

class ImuHandler(Node):
    def __init__(self):
        super().__init__('imu_handler')
        self.publisher_ = self.create_publisher(sensor_msgs.msg.Imu, 'imu', 10)
        timer_period = 0.5 #change to the period that coincides with the IMU's update rate
        self.timer = self.create_timer(timer_period, self.publish_imu)
        self.i = 0
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

    def publish_imu(self):
        # read the serial data and parse the imu data into it's 9 components
        # imu message "imu,acceX,acceY,acceX,gyroX,gyroY,gyroZ,magX,magY,magZ"
        imu_data = self.ser.readline().decode('utf-8').split(',')
        # create the imu message
        msg = sensor_msgs.msg.Imu()
        # fill in the imu message
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.linear_acceleration.x = float(imu_data[1])
        msg.linear_acceleration.y = float(imu_data[2])
        msg.linear_acceleration.z = float(imu_data[3])
        msg.angular_velocity.x = float(imu_data[4])
        msg.angular_velocity.y = float(imu_data[5])
        msg.angular_velocity.z = float(imu_data[6])

        # publish the message
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    imu_handler = ImuHandler()
    rclpy.spin(imu_handler)
    imu_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()