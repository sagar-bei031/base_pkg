import rclpy
from math import sin, cos, atan2, sqrt,  pi
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist

Filtered = '/odometry/filtered'
Raw = '/freewheel/odometry'

class OdomSubNode(Node):
    def __init__(self):
        super().__init__("odom_subscriber_node")
        
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        self.prev_yaw = 0
        self.yaw_sum = 0
        
        self.imu_subscriber = self.create_subscription(Odometry, 'odometry/filtered', self.process_data, qos_profile)
        
    def process_data(self, odom_msg):
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        z = odom_msg.pose.pose.position.z

        yaw, pitch, roll = quaternion_to_yawpitchroll(odom_msg.pose.pose.orientation.w,
                                                      odom_msg.pose.pose.orientation.x,
                                                      odom_msg.pose.pose.orientation.y,
                                                      odom_msg.pose.pose.orientation.z)
        self.yaw_sum += yaw - self.prev_yaw
        self.prev_yaw = yaw

        #self.get_logger().info('sxyz:"%f %f %f %f" ypr: "%f %f %f"' %(self.prev_yaw * 180/pi, x, y, z, yaw*180/pi, pitch*180/pi, roll*180/pi))
        self.get_logger().info('sypr: "%f %f %f %f"' %(self.prev_yaw * 180/pi, yaw*180/pi, pitch*180/pi, roll*180/pi))

        
def yawpitchroll_to_quaternion(yaw, pitch, roll):
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

def quaternion_to_yawpitchroll(w, x, y, z):
    # roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = sqrt(1 + 2 * (w * y - x * z))
    cosp = sqrt(1 - 2 * (w * y - x * z))
    pitch = 2 * atan2(sinp, cosp) - pi / 2

    # yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = atan2(siny_cosp, cosy_cosp)

    return  yaw, pitch, roll

def main(args=None):
    rclpy.init()
    odom_node = OdomSubNode()
    try:
        rclpy.spin(odom_node)
    except KeyboardInterrupt:
        odom_node.destroy_node()
        rclpy.try_shutdown()
        exit()
   
if __name__ =='__main__':
    main()
