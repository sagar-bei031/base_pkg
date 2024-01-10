import rclpy
from rclpy.node import Node
import time
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

MAX_VELOCITY = 1.0
MAX_OMEGA = 1.5

def map_value(value, min_value, max_value, new_min, new_max):
    mapped_value = ((value - min_value) * (new_max - new_min)
                    ) / (max_value - min_value) + new_min
    return mapped_value

class PS4Node(Node):
    def __init__(self):
        super().__init__('ps4_node')
        self.cmd_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joy_subscriber = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.isEmergencyBrake = False
        self.last_published_time = time.time()
        self.get_logger().info('ps4_node is running...')

    def joy_callback(self, msg):
        dt = time.time() - self.last_published_time

        if (dt > 0.033):
            # print(msg.axes)

            if (msg.buttons[5] == 1):
                speedFactor = 0.2
            else:
                speedFactor = 1.0

            # Map left joystick to velocity_x and velocity_y
            # X-axis of left joystick
            vx = map_value(msg.axes[0], -1.0, 1.0,
                           MAX_VELOCITY, -MAX_VELOCITY) * speedFactor
            # Y-axis of left joystick
            vy = map_value(msg.axes[1], -1.0,  1.0, -
                           MAX_VELOCITY, MAX_VELOCITY) * speedFactor

            # Map L2 and R2 to omega
            w = map_value(msg.axes[2] - msg.axes[5], -2.0, 2.0,
                          MAX_OMEGA, -MAX_OMEGA) * speedFactor  # L2 - R2

            # if ((msg.axes[6] != 0) | (msg.axes[7] != 0)):
            #     vy = msg.axes[7] * MAX_VELOCITY * speedFactor
            #     vx = -msg.axes[6] * MAX_VELOCITY * speedFactor

            if (msg.buttons[10]):
                self.isEmergencyBrake = True

            if (msg.buttons[4] and msg.buttons[5] and msg.buttons[10]):
                self.isEmergencyBrake = False

            if (self.isEmergencyBrake):
                vx = vy = w = 0.0

            print(msg.buttons)
            self.set_speed(vx, vy, w)
            last_published_time = time.time()

    def set_speed(self, vx, vy, w):
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.angular.z = vx, vy, w
        self.cmd_publisher_.publish(twist)
        print(twist.linear.x, twist, twist.linear.y, twist.angular.z)

def main():
    rclpy.init()
    ps4 = PS4Node()
    rclpy.spin(ps4)
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
