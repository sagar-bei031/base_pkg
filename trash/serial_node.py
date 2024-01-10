import serial
import struct
import numpy as np
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import crc8

START_BYTE = 0xA5
BLACK_TTL = '/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0'
RED_TTL = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0'
USING_TTL = RED_TTL

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.state_publisher_ = self.create_publisher(Float32MultiArray, '/raw_robot_state', 10)
        self.cmd_subscription = self.create_subscription(Float32MultiArray, '/cmd_robot_vel', self.joy_callback, 10)
        self.get_logger().info('Serial node is running...')
        self.serial_port = serial.Serial(USING_TTL, 115200)
        self.timer = self.create_timer(0.005, self.odom_serial_receive)

    def joy_callback(self, msg):
        # print(msg.data)
        data = [bytes(struct.pack("B", START_BYTE)),
                bytes(struct.pack("f", msg.data[0])),
                bytes(struct.pack("f", msg.data[1])),
                bytes(struct.pack("f", msg.data[2]))]
        data = b''.join(data)
        hash_value = self.tx_calc_crc(data)
        data = [data, bytes(struct.pack('B', hash_value))]
        data = b''.join(data)
        self.serial_port.write(data)
        self.serial_port.reset_output_buffer()
        # print("sending...")

    def odom_serial_receive(self):
        if self.serial_port.in_waiting >= 26:
            start_byte_found = False
            while not start_byte_found:
                byte = self.serial_port.read(1)
                # print(byte)
                if int.from_bytes(byte, 'big') == START_BYTE:
                    # print("new data")
                    data_str = self.serial_port.read(25)
                    start_byte_found = True
                else:
                    print("START_BYTE_NOT_FOUND")
            hash = self.rx_calc_crc(data_str)
            if hash == data_str[-1]:
                self.serial_port.reset_input_buffer()
                # print("hash matched")
                msg = Float32MultiArray()
                msg.data = struct.unpack("ffffff", data_str[0:24])
                self.state_publisher_.publish(msg)
                self.get_logger().info('raw_data "%f %f %f %f %f %f"' %(msg.data[0]*100, msg.data[1]*100, msg.data[2]*180/math.pi, msg.data[3]*100, msg.data[4]*100, msg.data[5]*180/math.pi))
                # print(msg.data)

    # To check crc while recieving data
    def rx_calc_crc(self, data=[]):
        hash_func = crc8.crc8()
        hash_func.update(data[0:-1])
        return hash_func.digest()[0]

    def tx_calc_crc(self, data=[]):
        hash_func = crc8.crc8()
        hash_func.update(data[1:])
        return hash_func.digest()[0]

    def calc_checksum(self, data=[]):
        checksum = 0
        for i in range(0, len(data)):
            checksum = checksum ^ data[i]
        return checksum

def main(args=None):
    rclpy.init(args=args)
    ser = SerialNode()
    try:
        rclpy.spin(ser)
    except KeyboardInterrupt:
        pass
    ser.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
