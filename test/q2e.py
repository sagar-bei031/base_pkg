import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math

class Q2E(Node):
    def __init__(self):
        super().__init__('q2e_conersion_node')
        # self.imu_sub = self.create_subscription(Imu,
        #                                         '/imu/data', 
        #                                         self.imu_callback,
        #                                         10)
        self.odom_sub = self.create_subscription(Odometry,
                                                 '/freewheel/odom',
                                                 self.odom_callback,
                                                 10)
        # self.filter_sub = self.create_subscription(Odometry,
        #                                            '/odometry/filtered',
        #                                            self.filter_callback,
        #                                            10)
        

    def imu_callback(self, imu_msg):
        qw = imu_msg.orientation.w
        qx = imu_msg.orientation.x
        qy = imu_msg.orientation.y
        qz = imu_msg.orientation.z

        r, p, y = quaternian_to_rollpitchyaw(qw, qx, qy, qz)
        # print(r, p, y)

    def odom_callback(self, odom_msg):
        qw = odom_msg.pose.pose.orientation.w
        qx = odom_msg.pose.pose.orientation.x
        qy = odom_msg.pose.pose.orientation.y
        qz = odom_msg.pose.pose.orientation.z

        r, p, y = quaternian_to_rollpitchyaw(qw, qx, qy, qz)
        print(y*180/math.pi)

    def filter_callback(self, odom_msg):
        qw = odom_msg.pose.pose.orientation.w
        qx = odom_msg.pose.pose.orientation.x
        qy = odom_msg.pose.pose.orientation.y
        qz = odom_msg.pose.pose.orientation.z

        r, p, y = quaternian_to_rollpitchyaw(qw, qx, qy, qz)
        # print(y)

def rollpitchyaw_to_quaternian(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qw, qx, qy, qz

def quaternian_to_rollpitchyaw(qw, qx, qy, qz):
    # roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = math.sqrt(1 + 2 * (qw * qy - qx * qz))
    cosp = math.sqrt(1 - 2 * (qw * qy - qx * qz))
    pitch = 2 * math.atan2(sinp, cosp) - math.pi / 2

    # yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def main():
    rclpy.init()
    q2e = Q2E()
    rclpy.spin(q2e)
    q2e.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()