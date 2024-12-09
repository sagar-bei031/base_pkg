import rclpy
import time
from math import sin, cos, atan2, sqrt,  pi
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class CmdSubNode(Node):
    def __init__(self):
        super().__init__("cmd_subscriber_node")
        
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        
        self.imu_subscriber = self.create_subscription(Twist, '/cmd_vel', self.process_data, qos_profile)
        self.last_rx = 0

    def process_data(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        w = msg.angular.z
        self.get_logger().info('"dt:%f vx:%f vy:%f w:%f"' %((time.time()-self.last_rx)*1000, vx, vy, w))
        self.last_rx = time.time()

def main(args=None):
    rclpy.init()
    node = CmdSubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.try_shutdown()
        exit()
   
if __name__ =='__main__':
    main()
