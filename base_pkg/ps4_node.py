import rclpy
import time
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

MAX_VELOCITY = 1.0
MAX_OMEGA = 1.5

isEmergencyBrake = False
last_published_time = 0

def joy_callback(msg):
    global last_published_time
    dt = time.time() - last_published_time

    if (dt>0.05):
        global isEmergencyBrake

        # print(msg.axes)

        if (msg.buttons[4] == 1):
            speedFactor = 0.2
        else:
            speedFactor = 1.0

        # Map left joystick to velocity_x and velocity_y
        vx = map_value(msg.axes[0], -1.0, 1.0, MAX_VELOCITY, -MAX_VELOCITY) * speedFactor   # X-axis of left joystick
        vy = map_value(msg.axes[1], -1.0,  1.0, -MAX_VELOCITY, MAX_VELOCITY) * speedFactor  # Y-axis of left joystick

        # Map L2 and R2 to omega
        w = map_value(msg.axes[2] - msg.axes[5], -2.0, 2.0, MAX_OMEGA, -MAX_OMEGA) * speedFactor  # L2 - R2

        if ((msg.axes[6] != 0) | (msg.axes[7] != 0)):
            vy = msg.axes[7] * MAX_VELOCITY * speedFactor
            vx = -msg.axes[6] * MAX_VELOCITY * speedFactor

        if (msg.buttons[10]):
            isEmergencyBrake = True

        if (msg.buttons[4] and msg.buttons[5] and msg.buttons[10]):
            isEmergencyBrake = False

        if (isEmergencyBrake):
            vx = vy = w = 0.0

        # print(msg.buttons)
        set_speed(vx, vy, w)
        last_published_time += dt


def set_speed(vx, vy, w):
    twist_array = Float32MultiArray()
    twist_array.data = [vx, vy, w]
    pub.publish(twist_array)
    print(twist_array.data[0], twist_array.data[1], twist_array.data[2])


def map_value(value, min_value, max_value, new_min, new_max):
    mapped_value = ((value - min_value) * (new_max - new_min)) / (max_value - min_value) + new_min
    return mapped_value


def main():
    rclpy.init()
    node = rclpy.create_node('ps4_controller')

    global pub
    pub = node.create_publisher(Float32MultiArray, '/cmd_robot_vel', 10)
    sub = node.create_subscription(Joy, '/joy', joy_callback, 10)
    print("PS4 node is running...")

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


