import serial
import struct
import rclpy
from math import sin, cos, radians, pi
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
import time
from crc8 import crc8


START_BYTE = 0xA5

RED_TTL = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0'
BLACK_TTL = '/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0'
USING_TTL = BLACK_TTL

 rclcpp::QoS qos_profile(10);
 qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

class SerialNode(Node):
    def __init__(self):
        super().__init__('odom_node')
        self.odom_publisher_ = self.create_publisher(
            Odometry, 'odometry/filtered', qos_profile)
        self.serial_port = serial.Serial(USING_TTL, 115200)
        self.last_sent_time = time.time()
        self.odom_seq = 0
        self.is_waiting_for_start_byte = True
        self.get_logger().info('odom_node is running..')
        

    def receive_and_publish(self):
        if self.is_waiting_for_start_byte:
            if self.serial_port.in_waiting >= 1:
                byte = self.serial_port.read(1)
                if int.from_bytes(byte, 'big') == START_BYTE:
                    # print(byte)
                    self.is_waiting_for_start_byte = False
                else:
                    self.get_logger().info("Start Byte Not Matched")

        else:
            if self.serial_port.in_waiting >= 49:
                self.is_waiting_for_start_byte = True
                data_str = self.serial_port.read(49)
                hash = self.calc_crc(data_str[:-1])
                if hash == data_str[-1]:
                    # data = [x, y, theta, vx, vy, omega]
                    #now = time.time()
                    #if (now - self.last_sent_time > 0.05):
                    if (1):
                        data = struct.unpack("fffffffff", data_str[0:36])
                        odom_msg = Odometry()
                        odom_msg.header.stamp = self.get_clock().now().to_msg()
                        odom_msg.header.frame_id = 'odom'
                        odom_msg.child_frame_id = 'base_link'
                        odom_msg.pose.pose.position.x = -data[0]
                        odom_msg.pose.pose.position.y = -data[1]
                        odom_msg.pose.pose.position.z = 0.0
                        qw, qx, qy, qz = rollpitchyaw_to_quaternion(data[8], data[7], data[6])
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
                        odom_msg.twist.twist.linear.x = -data[3]
                        odom_msg.twist.twist.linear.y = -data[4]
                        # odom_msg.twist.twist.linear.z = 0.0
                        # odom_msg.twist.twist.angular.x = 0.0
                        # odom_msg.twist.twist.angular.y = 0.0
                        odom_msg.twist.twist.angular.z = data[5]
                        odom_msg.twist.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                     0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                                     0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                                                     0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                                     0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.04]
                        self.odom_publisher_.publish(odom_msg)
                        now = time.time()
                        self.odom_seq += 1
                        dt = now - self.last_sent_time
                        self.last_sent_time = now

                        self.get_logger().info('"dt:%f x:%f y:%f yaw:%f pitch:%f roll:%f"'
                                            %(dt, -data[0]*100, -data[1]*100, data[2]*180/pi, 
                                              data[7]*180/pi, data[8]*180/pi))
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
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qw, qx, qy, qz


def main(args=None):
    rclpy.init(args=args)
    myserial = SerialNode()
    while True:
        try:
            myserial.receive_and_publish()
        except KeyboardInterrupt:
            if rclpy.ok():
                myserial.destroy_node()
                rclpy.shutdown()
            exit()


if __name__ == '__main__':
    main()

