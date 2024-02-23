import rclpy
from math import sin, cos, atan2, sqrt,  pi
from rclpy.node import Node 
from sensor_msgs.msg import Imu


class ImuSubNode(Node):
    def __init__(self):
        super().__init__("imu_subscriber_node")
        self.imu_subscriber = self.create_subscription(Imu, 'imu/data', self.process_data, 10)

        
    def process_data(self, imu_msg):
        yaw, pitch, roll = quaternion_to_yawpitchroll(imu_msg.orientation.w,
                                                      imu_msg.orientation.x,
                                                      imu_msg.orientation.y,
                                                      imu_msg.orientation.z)
        self.get_logger().info('ypr: "%f %f %f"' %(yaw*180/pi, pitch*180/pi, roll*180/pi))

        
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
    imu_node = ImuSubNode()
    try:
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        imu_node.destroy_node()
        rclpy.try_shutdown()
        exit()
   
if __name__ =='__main__':
    main()