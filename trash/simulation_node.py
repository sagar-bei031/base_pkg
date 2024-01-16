import json
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

HOST = '127.0.0.1'
PORT = 5550

class SimulationNode(Node):
    def __init__(self):
        super().__init__('simulation_node')
        self.subscription = self.create_subscription(Odometry,
                                                      '/filtered/odometry',
                                                      self.listener_callback,
                                                      10)
        self.get_logger().info('simulation_node is running..')

    def listener_callback(self, msg):  
        x = msg.pose.pose.position.x * 100
        y = msg.pose.pose.position.y * 100
        roll, pitch, yaw = quaternion_to_rollpitchyaw(msg.pose.pose.orientation.w,
                                                      msg.pose.pose.orientation.x,
                                                      msg.pose.pose.orientation.y,
                                                      msg.pose.pose.orientation.z)
        theta = yaw
        # Construct dictionary
        data_dict = {
            "x": float(-y),
            "y": float(x),
            "theta": float(yaw)
        }
        try:
            with open('gamefield/data/positionR1.json', 'w') as f:
                json.dump(data_dict, f, indent=4)
        except IOError as e:
                self.get_logger().info(f"Exception: {e}")

def quaternion_to_rollpitchyaw(qw, qx, qy, qz):
    # Roll (x-axis rotation)
    roll_rad = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx**2 + qy**2))
    roll_deg = math.degrees(roll_rad)

    # Pitch (y-axis rotation)
    pitch_rad = math.asin(2 * (qw * qy - qz * qx))
    pitch_deg = math.degrees(pitch_rad)

    # Yaw (z-axis rotation)
    yaw_rad = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))
    yaw_deg = math.degrees(yaw_rad)

    return roll_deg, pitch_deg, yaw_deg

def main(args=None):
    rclpy.init(args=args)
    sim = SimulationNode()
    rclpy.spin(sim)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



