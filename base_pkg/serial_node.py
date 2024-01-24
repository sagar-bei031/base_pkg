import serial
import math
import struct
from math import sin, cos
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from crc8 import crc8
import time

START_BYTE = 0xA5

RED_TTL = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0'
BLACK_TTL = '/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0'
USING_TTL = BLACK_TTL

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.odom_publisher_ = self.create_publisher(
            Odometry, 'freewheel/odom', 10)
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.joy_callback, 10)
        
        self.enc_count_publisher_ = self.create_publisher(
            Int32MultiArray, 'freewheel/count', 10)
        
        self.is_waiting_for_start_byte = True
        self.timer = self.create_timer(0.1, self.odom_serial_receive)
        self.serial_port = serial.Serial(USING_TTL, 115200)
        self.odom_seq = 0
        self.last_pub_time = 0
        self.last_write_time = 0

        self.get_logger().info('Serial node is running...')

    def joy_callback(self, twist_msg):
        if (time.time() - self.last_write_time > 0.05):
            data = [
                bytes(struct.pack("B", START_BYTE)),
                bytes(struct.pack("f", float(twist_msg.linear.x))),
                bytes(struct.pack("f", float(twist_msg.linear.y))),
                bytes(struct.pack("f", float(twist_msg.angular.z)))]
            data = b''.join(data)
            hash = self.calc_crc(data[1:])
            data = [data, bytes(struct.pack('B', hash))] 
            data = b''.join(data)
            self.serial_port.reset_output_buffer()
            self.serial_port.write(data)
            self.last_write_time = time.time()
            # print(data)

    def odom_serial_receive(self):
        if (self.serial_port.in_waiting < 38):
            return

        if self.is_waiting_for_start_byte:
            byte = self.serial_port.read(1)
            if int.from_bytes(byte, 'big') == START_BYTE:
                # print(byte)
                self.is_waiting_for_start_byte = False
            else:
                self.get_logger().info("Start Byte Not Matched")
        else:
            self.is_waiting_for_start_byte = True
            data_str = self.serial_port.read(37)
            hash = self.calc_crc(data_str[:-1])
            if hash == data_str[-1]:
                # data = [x, y, theta, vx, vy, omega]
                now = time.time()
                if (now - self.last_pub_time >= 50):
                    data = struct.unpack("ffffffiii", data_str[0:36])
                    odom_msg = Odometry()
                    odom_msg.header.stamp = self.get_clock().now().to_msg()
                    odom_msg.header.frame_id = 'odom'
                    odom_msg.child_frame_id = 'base_link'
                    odom_msg.pose.pose.position.x = data[0]
                    odom_msg.pose.pose.position.y = data[1]
                    odom_msg.pose.pose.position.z = 0.0
                    qw, qx, qy, qz = rollpitchyaw_to_quaternion(0.0, 0.0, data[2])
                    odom_msg.pose.pose.orientation.w = qw
                    odom_msg.pose.pose.orientation.x = qx
                    odom_msg.pose.pose.orientation.y = qy
                    odom_msg.pose.pose.orientation.z = qz
                    odom_msg.pose.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                                                0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                                                0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                                0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                                0.0, 0.0, 0.0, 0.0, 0.0, 0.030461]
                    odom_msg.twist.twist.linear.x = data[3]
                    odom_msg.twist.twist.linear.y = data[4]
                    odom_msg.twist.twist.linear.z = 0.0
                    odom_msg.twist.twist.angular.x = 0.0
                    odom_msg.twist.twist.angular.y = 0.0
                    odom_msg.twist.twist.angular.z = data[5]
                    odom_msg.twist.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                 0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                                 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                                                 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                                 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.04]
                    self.odom_publisher_.publish(odom_msg)
                    self.odom_seq += 1

                    count_msg = Int32MultiArray()
                    count_msg.data = [data[6], data[7], data[8]]
                    self.enc_count_publisher_.publish(count_msg)

                    self.get_logger().info('"%f %f %f %f %f %f %i %i %i"'
                                        %(data[0], data[1], data[2], 
                                          data[3], data[4], data[5],
                                          data[6], data[7], data[8]))
                    
                    self.last_pub_time = now
            else:
                self.get_logger().info('Hash error')

    def calc_crc(self, data=[]):
        hash_func = crc8()
        hash_func.update(data)
        return hash_func.digest()[0]

    def calc_checksum(self, data=[]):
        checksum = 0
        for i in range(0, len(data)):
            checksum = checksum ^ data[i]
        return checksum


def rollpitchyaw_to_quaternion(roll, pitch, yaw):
    roll_rad = math.radians(roll)
    pitch_rad = math.radians(pitch)
    yaw_rad = math.radians(yaw)

    cy = cos(yaw_rad * 0.5)
    sy = sin(yaw_rad * 0.5)
    cp = cos(pitch_rad * 0.5)
    sp = sin(pitch_rad * 0.5)
    cr = cos(roll_rad * 0.5)
    sr = sin(roll_rad * 0.5)

    qw = cy * cp * cr + sy * sp * sr
    qx = cy * cp * sr - sy * sp * cr
    qy = sy * cp * sr + cy * sp * cr
    qz = sy * cp * cr - cy * sp * sr

    return qw, qx, qy, qz


def main(args=None):
    rclpy.init(args=args)
    myserial = SerialNode()
    while True:
        try:
            rclpy.spin(node=myserial)
        except KeyboardInterrupt:
            if rclpy.ok():
                myserial.destroy_node()
                rclpy.shutdown()
            exit()


if __name__ == '__main__':
    main()