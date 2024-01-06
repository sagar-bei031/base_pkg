import serial
import math
import struct
from math import sin, cos
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import  Twist, Point
import crc8
import numpy as np

START_BYTE = 0xA5
class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.odom_publisher_ = self.create_publisher(Odometry, 'robot/odom0', 10)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.joy_callback, 10)

        self.serial_port = serial.Serial('/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0', 115200)
        self.timer = self.create_timer(0.02, self.process_buffer)  # 50Hz (1/50 = 0.02 sec)

        self.odom_seq = 0
        self.pose_buffer = []
        self.twist_buffer = []

        self.get_logger().info('Serial node is running...')

    def joy_callback(self, twist_msg):
        data = [
            bytes(struct.pack("B", START_BYTE)),
            bytes(struct.pack("f", twist_msg.linear.x)),
            bytes(struct.pack("f", twist_msg.linear.y)),
            bytes(struct.pack("f", twist_msg.angular.w))
        ]
        data = b''.join(data)
        hash = self.calc_crc(data[1:])
        data = [data, bytes(struct.pack('B', hash))]
        data = b''.join(data)
        self.serial_port.write(data)
        self.serial_port.reset_output_buffer()

    def odom_serial_receive(self):
        while True:
            if self.serial_port.in_waiting >= 26:
                start_byte_found = False
                while not start_byte_found:
                    byte = self.serial_port.read(1)
                    if int.from_bytes(byte, 'big') == START_BYTE:
                        data_str = self.serial_port.read(25)
                        start_byte_found = True
                hash = self.calc_crc(data_str[:-1])
                if hash == data_str[-1]:
                    self.serial_port.reset_input_buffer()
                    data = struct.unpack("ffffff", data_str[0:24])  # data = [x, y, theta, vx, vy, omega]
                    self.position.append(data[0], data[1], 0.0, 0.0, 0.0, data[2])          # [x, y, z, roll, pitch, yaw]
                    self.twist_buffer.append(data[3], data[4], 0.0, 0.0, 0.0,  data[5])     # [vx, vy, vz, v, vroll, vpitch, vyaw]
            else:
                break

    def process_buffer(self):
        if self.pose_buffer and self.twist_buffer:
            pose_means = np.mean(self.pose_buffer, axis=0)
            pose_covs = np.cov(self.pose_buffer, rowvar=False)
            twist_means = np.mean(self.twist_buffer, axis=0)
            twist_covs = np.cov(self.twist_buffer, rowvar=False)

            odom_msg = Odometry()
            odom_msg.header.seq = self.odom_seq
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'

            odom_msg.pose.pose.position = Point(pose_means[0], pose_means[1], pose_means[2])

            qx, qy, qz, qw = euler_to_quaternion(pose_means[3], pose_means[4], pose_means[5]) 
            odom_msg.pose.pose.orientation(qx, qy, qz, qw)

            odom_msg.pose.covariance = pose_covs

            odom_msg.twist.twist = Twist(twist_means[0], twist_means[1], twist_means[2],
                                         twist_means[3], twist_means[4], twist_means[5])
            
            odom_msg.twist.covariance = twist_covs

            self.odom_publisher_.publish(odom_msg)
            self.odom_seq += 1
            self.pose_buffer = []
            self.twist_buffer= []

    def calc_crc(self, data=[]):
        hash_func = crc8.crc8()
        hash_func.update(data)
        return hash_func.digest()[0]

    def calc_checksum(self, data=[]):
        checksum = 0
        for i in range(0, len(data)):
            checksum = checksum ^ data[i]
        return checksum

def euler_to_quaternion(roll, pitch, yaw):
    roll_rad = math.radians(roll)
    pitch_rad = math.radians(pitch)
    yaw_rad = math.radians(yaw)

    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)

    qw = cy * cp * cr + sy * sp * sr
    qx = cy * cp * sr - sy * sp * cr
    qy = sy * cp * sr + cy * sp * cr
    qz = sy * cp * cr - cy * sp * sr

    return qx, qy, qz, qw


def main(args=None):
    rclpy.init(args=args)
    myserial = SerialNode()
    try:
        rclpy.spin(myserial)
    except KeyboardInterrupt:
        pass
    myserial.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
