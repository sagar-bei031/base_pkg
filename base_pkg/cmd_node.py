import rclpy
from rclpy.node import Node
import time
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import struct
import serial
from crc8 import crc8

MAX_VELOCITY = 1.0
MAX_OMEGA = 1.0
START_BYTE = 0xA5

RED_TTL = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5XK3RJT-if00-port0'
BLACK_TTL = '/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0'
USING_TTL = RED_TTL

def map_value(value, min_value, max_value, new_min, new_max):
    mapped_value = ((value - min_value) * (new_max - new_min)) / (max_value - min_value) + new_min
    return mapped_value

class CmdNode(Node):
    def __init__(self):
        super().__init__('cmd_node')
        self.cmd_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joy_subscriber = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.isEmergencyBrake = False
        self.twist = Twist()
        self.serial_port = serial.Serial(USING_TTL, 115200)
        self.last_sent_time = time.time()
        self.get_logger().info('cmd_node is running...')

    def joy_callback(self, msg):
        dt = time.time() - self.last_sent_time

        if (dt > 0.05):
            # R1
            if (msg.buttons[5] == 1):
                speedFactor = 0.2
            else:
                speedFactor = 1.0

            # Map left joystick to velocity_x and velocity_y
            # horizontal of left joystick
            self.twist.linear.x = map_value(msg.axes[0], -1.0, 1.0,
                           -MAX_VELOCITY, MAX_VELOCITY) * speedFactor
            # verticle of left joystick
            self.twist.linear.y = map_value(msg.axes[1], -1.0,  1.0,
                           -MAX_VELOCITY, MAX_VELOCITY) * speedFactor

            # Map L2 and R2 to omega
            self.twist.angular.z = map_value(msg.axes[4] - msg.axes[5], 2.0, -2.0,
                          MAX_OMEGA, -MAX_OMEGA) * speedFactor  # L2 - R2

            # left pad buttoons, horizontal, verticle
            if ((msg.axes[6] != 0) | (msg.axes[7] != 0)):
                self.twist.linear.y = msg.axes[6] * MAX_VELOCITY * speedFactor
                self.twist.linear.x = msg.axes[7] * MAX_VELOCITY * speedFactor

            # ps4_button
            if (msg.buttons[10]):
                self.isEmergencyBrake = True

            # L1 R1
            if (msg.buttons[4] and msg.buttons[5] and msg.buttons[10]):
                self.isEmergencyBrake = False

            if (self.isEmergencyBrake):
                self.twist.linear.x = 0
                self.twist.linear.y = 0
                self.twist.angular.z = 0

            self.set_speed_and_publish()
            self.last_sent_time = time.time()

    def set_speed_and_publish(self):
        data = [
            bytes(struct.pack("B", START_BYTE)),
            bytes(struct.pack("f", float(self.twist.linear.x))),
            bytes(struct.pack("f", float(self.twist.linear.y))),
            bytes(struct.pack("f", float(self.twist.angular.z)))]
        data = b''.join(data)
        hash = self.calc_crc(data[1:])
        data = [data, bytes(struct.pack('B', hash))] 
        data = b''.join(data)
        # self.serial_port.reset_output_buffer()
        self.serial_port.write(data)
        self.cmd_publisher_.publish(self.twist)

        self.get_logger().info('vx vy w: "%f %f %f"' 
            %(self.twist.linear.x, self.twist.linear.y, self.twist.angular.z))
        
    def calc_crc(self, data=[]):
        hash_func = crc8()
        hash_func.update(data)
        return hash_func.digest()[0]
        
def main():
    rclpy.init()
    while True:
        cmd = CmdNode()
        try:
            rclpy.spin(cmd)
        except KeyboardInterrupt:
            if rclpy.ok():
                cmd.destroy_node()
                rclpy.shutdown()
            exit()

if __name__ == '__main__':
    main()
