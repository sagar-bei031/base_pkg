import serial
import struct
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import crc8

START_BYTE = 0xA5


class SerialReceiveNode(Node):
    def __init__(self):
        super().__init__('serial_receive_node')
        self.publisher_ = self.create_publisher(
            Float32MultiArray, 'raw_robot_state', 10)
        self.get_logger().info('Serial receive node is running...')
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200)
        self.crc = crc8.crc8()

    def serial_receive(self):
        while self.serial_port.in_waiting >= 26:
            start_byte = self.serial_port.read()
            if start_byte == bytes([START_BYTE]):
                state = struct.unpack("ffffff", self.serial_port.read(24))
                rem = self.serial_port.read()
                self.crc.reset()
                self.crc.update(struct.pack("ffffff", *state))
                hash = self.crc.digest()
                if hash == rem:
                    msg = Float32MultiArray()
                    msg.data = state
                    self.publisher_.publish(msg)
            else:
                self.serial_port.reset_input_buffer()


def main(args=None):
    rclpy.init()
    serial_node = SerialReceiveNode()

    try:
        while rclpy.ok():
            serial_node.serial_receive()
            rclpy.spin_once(serial_node)    # Allow ROS to process callbacks
            time.sleep(0.01)                # Add a small delay to prevent busy-waiting
    except KeyboardInterrupt:
        pass

    serial_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
