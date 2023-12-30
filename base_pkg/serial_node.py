import serial
import struct
import numpy as np
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import crc8

START_BYTE = 0xA5
CMD_VEL_ID = 0x00
PID_CFG_ID = 0xFF

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.raw_state_publisher = self.create_publisher(Float32MultiArray, '/raw_robot_state', 10)
        self.cmd_subscription = self.create_subscription(Float32MultiArray, '/cmd_robot_vel', self.pid_callback, 10)
        self.pid_subscription = self.create_subscription(Float32MultiArray, '/pid_config', self.joy_callback, 10)
        self.get_logger().info('Serial node is running...')
        self.serial_port = serial.Serial('/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0', 115200)
        self.timer = self.create_timer(0.005, self.odom_serial_receive)

    def pid_callback(self, msg):
        data = [bytes(struct.pack("B", START_BYTE)),
                bytes(struct.pack("B", PID_CFG_ID)),
                bytes(struct.pack("f", msg.data[0])),
                bytes(struct.pack("f", msg.data[1])),
                bytes(struct.pack("f", msg.data[2])),
                bytes(struct.pack("f", msg.data[3])),
                bytes(struct.pack("f", msg.data[4])),
                bytes(struct.pack("f", msg.data[5])),
                bytes(struct.pack("f", msg.data[6])),
                bytes(struct.pack("f", msg.data[7])),
                bytes(struct.pack("f", msg.data[8])),
                bytes(struct.pack("f", msg.data[9])),
                bytes(struct.pack("f", msg.data[10])),
                bytes(struct.pack("f", msg.data[11]))]
        data = b''.join(data)
        hash_value = self.calculate_crc(data)
        data = [data, bytes(struct.pack('B', hash_value))]
        data = b''.join(data)
        self.serial_port.write(data)
        self.serial_port.reset_output_buffer()

    def joy_callback(self, msg):
        # print(msg.data)
        data = [bytes(struct.pack("B", START_BYTE)),
                bytes(struct.pack("B", CMD_VEL_ID)),
                bytes(struct.pack("f", msg.data[0])),
                bytes(struct.pack("f", msg.data[1])),
                bytes(struct.pack("f", msg.data[2]))]
        data = b''.join(data)
        hash_value = self.calculate_crc(data)
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
            hash = self.calc_crc(data_str)
            if hash == data_str[-1]:
                self.serial_port.reset_input_buffer()
                # print("hash matched")
                msg = Float32MultiArray()
                msg.data = struct.unpack("ffffff", data_str[0:24])
                self.raw_state_publisher.publish(msg)
                self.get_logger().info('raw_data "%f %f %f %f %f %f"' %(msg.data[0]*100, msg.data[1]*100, msg.data[2]*180/math.pi, msg.data[3]*100, msg.data[4]*100, msg.data[5]*180/math.pi))
                # print(msg.data)

    # To check crc while recieving data
    def calc_crc(self, data=[]*23):
        hash_func = crc8.crc8()
        hash_func.update(data[0:-1])
        return hash_func.digest()[0]

    def calc_checksum(self, data=[]):
        checksum = 0
        for i in range(0, len(data)):
            checksum = checksum ^ data[i]
        return checksum

    # To compute crc while transmitting data
    def calculate_crc(self, data=[]):
        hash_func = crc8.crc8()
        hash_func.update(data[1:])
        return hash_func.digest()[0]

def main(args=None):
    rclpy.init(args=args)
    ser = SerialNode()
    try:
        rclpy.spin(ser)
    except KeyboardInterrupt:
        pass
    ser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
