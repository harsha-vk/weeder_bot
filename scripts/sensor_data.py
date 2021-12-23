#!/usr/bin/env python3

import math
import os
import rclpy
import serial
import smbus
import sys
import threading
import time

from imusensor.MPU9250 import MPU9250
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node


class SensorData(Node):
    def __init__(self):
        super().__init__('sensor_data')
        self.declare_parameters(namespace='',
                                parameters=[('port',None),
                                            ('baudrate',57600),
                                            ('timeout',1),
                                            ('queue',10),
                                            ('wheel_pub_id',None),
                                            ('wheel_pub_len',None),
                                            ('wheel_sub_id',None),
                                            ('wheel_sub_len',None),
                                            ('imu_address',None)])
        self.serialParams()
        # Initialize serial port
        self.serialInit()
        # Create publishers
        queue = self.get_parameter('queue').get_parameter_value().integer_value
        self.wheel_pub = self.create_publisher(Vector3Stamped,'sensors/wheel_speed',queue)
        # Create subscribers
        self.wheel_sub = self.create_subscription(Vector3Stamped,'actuator/wheel_speed',self.wheelSubCallback,queue)

    def serialParams(self):
        self.wheel_pub_id = self.get_parameter('wheel_pub_id').get_parameter_value().integer_value
        self.wheel_pub_len = self.get_parameter('wheel_pub_len').get_parameter_value().integer_value
        self.wheel_sub_id = self.get_parameter('wheel_pub_id').get_parameter_value().integer_value
        self.wheel_sub_len = self.get_parameter('wheel_sub_len').get_parameter_value().integer_value

    def serialInit(self):
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        port = self.get_parameter('port').get_parameter_value().string_value
        self.header = b'\xff'
        self.protocol_ver = b'\xfe'
        self.read_lock = threading.RLock()
        self.write_lock = threading.RLock()
        self.lastsync = None
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self.synced = False
        while rclpy.ok():
            try:
                self.port = serial.Serial(port=port,
                                          baudrate=baudrate,
                                          timeout=self.timeout,
                                          write_timeout=self.timeout*2)
                break
            except serial.SerialException as e:
                self.get_logger().fatal('Error opening serial: %s', e)
        time.sleep(0.1)

    def requestTopics(self):
        data = self.header + self.protocol_ver + b'\x00\x00\xff\x00\x00\xff'
        try:
            with self.write_lock:
                self.port.write(data)
        except serial.SerialTimeoutException as e:
            self.get_logger().error('Write timeout: %s' % e)

    def serialRead(self):
        read_step = None
        if (self.get_clock().now().nanoseconds - self.lastsync) > (self.timeout * 3 * 1e9):
            if self.synced:
                self.get_logger().error('Lost to sync with device')
            else:
                self.get_logger().error('Unable to sync with device')
            self.requestTopics()
            self.lastsync = self.get_clock().now().nanoseconds
        try:
            with self.read_lock:
                if self.port.inWaiting() < 1:
                    time.sleep(0.001)
                    return
            flag = [0, 0]
            read_step = 'syncflag'
            flag[0] = self.tryRead(1)
            if (flag[0] != self.header):
                return
            read_step = 'protocol'
            flag[1] = self.tryRead(1)
            if flag[1] != self.protocol_ver:
                self.get_logger().error('Mismatched protocol version in packet (%s): lost sync' % repr(flag[1]))
                return
            read_step = 'message length'
            msg_len_bytes = self.tryRead(3)
            msg_length, _ = struct.unpack('<hB', msg_len_bytes)
            if sum(array.array('B', msg_len_bytes)) % 256 != 255:
                self.get_logger().info('Wrong checksum for msg length, length %d, dropping message.' % (msg_length))
                return
            read_step = 'topic id'
            topic_id_header = self.tryRead(2)
            topic_id, = struct.unpack('<H', topic_id_header)
            read_step = 'data'
            try:
                msg = self.tryRead(msg_length)
            except IOError:
                self.get_logger().info('Packet Failed; expected msg length is %d' % msg_length)
                raise
            read_step = 'data checksum'
            chk = self.tryRead(1)
            checksum = sum(array.array('B', topic_id_header + msg + chk))
            if checksum % 256 == 255:
                self.synced = True
                self.serialCallback(topic_id,msg_length,msg)
            else:
                self.get_logger().info('wrong checksum for topic id and msg')
        except IOError as e:
            self.get_logger().warn('Last read step: %s' % read_step)
            self.get_logger().warn('Run loop error: %s' % e)
            with self.read_lock:
                self.port.flushInput()
            with self.write_lock:
                self.port.flushOutput()
            self.requestTopics()

    def tryRead(self,length):
        try:
            read_start = time.time()
            bytes_remaining = length
            result = bytearray()
            while bytes_remaining != 0 and time.time() - read_start < self.timeout:
                with self.read_lock:
                    received = self.port.read(bytes_remaining)
                if len(received) != 0:
                    self.last_read = rospy.Time.now()
                    result.extend(received)
                    bytes_remaining -= len(received)
            if bytes_remaining != 0:
                raise IOError('Returned short (expected %d bytes, received %d instead).' % (length, length - bytes_remaining))
            return bytes(result)
        except Exception as e:
            raise IOError('Serial Port read failure: %s' % e)
    
    def serialCallback(self,topic_id,msg_length,msg):
        if topic_id == self.wheel_pub_id and msg_length == self.wheel_msg_len:
            seconds = struct.unpack('<I',msg_[0:4])
            nanoseconds = struct.unpack('<I',msg_[4:8])
            v_left = struct.unpack('<f',msg_[8:12])
            v_right = struct.unpack('<f',msg_[12:16])
            wheel_msg = Vector3Stamped()
            wheel_msg.header.stamp.sec = seconds
            wheel_msg.header.stamp.nanosec = nanoseconds
            wheel_msg.vector.x = v_left
            wheel_msg.vector.y = v_right
            self.wheel_pub.publish(wheel_msg)

    def wheelSubCallback(self,msg):
        v_left = struct.pack('<f',msg.vector.x)
        v_right = struct.pack('<f',msg.vector.y)
        try: #TODO
            with self.write_lock:
                self.port.write(data)
        except serial.SerialTimeoutException as e:
            self.get_logger().error('Write timeout: %s' % e)

    def imuInit(self):
        address = None
        bus = smbus.SMBus(1)
        self.imu = MPU9250.MPU9250(bus,address)
        self.imu.begin()

    def imuRead(self):
        if (self.get_clock().now().nanoseconds - self.lastsync) > (self.timeout * 3 * 1e9): #TODO
            self.imu.readSensor()


def main(args=None):
    rclpy.init(args=args)
    sensor_data = SensorNode()
    sensor_data.requestTopics()
    sensor_data.lastsync = sensor_data.get_clock().now().nanoseconds
    while rclpy.ok():
        sensor_data.serialRead()
        sensor_data.imuRead()
        rclpy.spin_once(sensor_data,timeout_sec=0.1)
    rect_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()