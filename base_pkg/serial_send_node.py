
import serial
import struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import crc8

START_BYTE = 0xA5

class SerialSendNode(Node):
    def __init__(self):
        super().__init__('serial_send_node')
        self.subscription = self.create_subscription(Float32MultiArray, 'base_twist', self.listener_callback, 10)
        self.get_logger().info('Serial send node is running...')
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200)
        self.crc = crc8.crc8()

    def listener_callback(self, msg):
        data = struct.pack("fff", msg.data[0], msg.data[1], msg.data[2])
        self.crc.reset()
        self.crc.update(data)
        hash_value = self.crc.digest()

        self.serial_port.write(struct.pack('B', START_BYTE))
        self.serial_port.write(data)
        self.serial_port.write(struct.pack('B', hash_value))
        
def main(args=None):
    rclpy.init(args=args)
    ser = SerialSendNode()
    try:
        rclpy.spin(ser)
    except KeyboardInterrupt:
        pass
    ser.destroy_node()
    rclpy.shutdown()
    
if __name__== '__main__':
    main()
