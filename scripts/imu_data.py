#!/usr/bin/env python3

import rclpy
import smbus
from imusensor.MPU9250 import MPU9250
from sensor_msgs.msg import Imu
from rclpy.node import Node

class ImuData(Node):
    def __init__(self):
        super().__init__('imu_data')
        self.declare_parameters(namespace='',
                                parameters=[('queue',10),
                                            ('imu_address',None),
                                            ('imu_frequency',None)])
        # Initialize imu
        self.imuInit()
        # Create publishers
        queue = self.get_parameter('queue').get_parameter_value().integer_value
        self.imu_pub = self.create_publisher(Imu,'imu/imu_raw',queue)
        # Create timer
        timer_period = 1 / self.get_parameter('imu_frequency').get_parameter_value().integer_value
        self.timer = self.create_timer(timer_period, self.imuCallback)

    def imuInit(self):
        address = None
        bus = smbus.SMBus(1)
        self.imu = MPU9250.MPU9250(bus,address)
        self.imu.begin()

    def imuCallback(self):
        self.imu.readSensor()


def main(args=None):
    rclpy.init(args=args)
    imu_data = ImuData()
    rclpy.spin(imu_data)
    imu_data.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()