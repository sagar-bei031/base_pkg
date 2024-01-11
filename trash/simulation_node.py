import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

HOST = '127.0.0.1'
PORT = 5550

class SimulationNode(Node):
    def __init__(self):
        super().__init__('filter_node')
        self.subscription = self.create_subscription(Float32MultiArray,
                                                      'filtered_odometry',
                                                      self.listener_callback,
                                                      10)
        self.get_logger().info('simulation_node is running..')

    def listener_callback(self, msg):  
        x = msg.data[0]
        y = msg.data[1]
        theta = msg.data[2]
        # Construct dictionary
        data_dict = {
            "x": float(x),
            "y": float(y),
            "theta": float(theta)
        }
        try:
            with open('gamefield/data/positionR1.json', 'w') as f:
                json.dump(data_dict, f, indent=4)
        except IOError as e:
                self.get_logger().info(f"Exception: {e}")

def main(args=None):
    rclpy.init(args=args)
    sim = SimulationNode()
    rclpy.spin(sim)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



