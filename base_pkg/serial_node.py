import serial
import struct
import numpy as np
import threading
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import crc8

START_BYTE = 0xA5

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/raw_robot_state', 10)
        self.subscription = self.create_subscription(Float32MultiArray, '/cmd_robot_vel', self.listener_callback, 10)
        self.get_logger().info('Serial node is running...')
        self.serial_port = serial.Serial('/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0', 115200)
        self.receive_thread = threading.Thread(target=self.serial_receive)
        self.receive_thread.daemon = True  # Set the thread as daemon to terminate it when the main program ends
        self.receive_thread.start()

    def listener_callback(self, msg):
        data = struct.pack("fff", msg.data[0], msg.data[1], msg.data[2])
        # print(msg.data)
        hash_value = self.calc_crc(data)

        self.serial_port.write(struct.pack('B', START_BYTE))
        # print(struct.pack('B', START_BYTE))
        self.serial_port.write(data)
        # print(data)
        self.serial_port.write(struct.pack('B', hash_value))
        # print(struct.pack('B', hash_value))
        self.serial_port.reset_output_buffer()

    def serial_receive(self):
        while True:
            if self.serial_port.in_waiting >= 26:
                start_byte_found = False
                while not start_byte_found:
                    byte = self.serial_port.read(1)
                    # print(byte)
                    if int.from_bytes(byte, 'big') == START_BYTE:
                        # print("new data")
                        data_str = self.serial_port.read(25)
                        start_byte_found=True

                hash = self.calc_crc(data_str)
                if hash == data_str[-1]:
                    self.serial_port.reset_input_buffer()
                    # print("hash matched")
                    msg = Float32MultiArray()
                    msg.data = struct.unpack("ffffff", data_str[0:24])
                    self.publisher_.publish(msg)
                    self.get_logger().info('raw_data "%f %f %f %f %f %f"' %(msg.data[0]*100, msg.data[1]*100, msg.data[2]*180/math.pi, msg.data[3]*100, msg.data[4]*100, msg.data[5]*180/math.pi))
                    # print(msg.data)

                
    def calc_crc(self, data=[]*23):
        hash_func=crc8.crc8()
        hash_func.update(data[0:-1])
        return hash_func.digest()[0]
    
    def calc_checksum(self, data=[]):
        for i in range(0,len(data)):
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
    rclpy.shutdown()

if __name__ == '__main__':
    main()
