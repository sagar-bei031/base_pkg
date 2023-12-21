import time
import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise as w_noise
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
        

class FilterNode(Node):
    def __init__(self):
        super().__init__('filter_node')
        self.subscription = self.create_subscription(Float32MultiArray,
                                                      '/raw_robot_state',
                                                      self.listener_callback,
                                                      10)
        self.publisher_ = self.create_publisher(Float32MultiArray, 
                                                'filtered_odometry',
                                                10)
        self.kf = KalmanFilter(dim_x=3, dim_z=3, dim_u=3)
        self.kf.x = np.array([[0.0], [0.0], [0.0]])
        self.kf.F = np.eye(3)
        self.kf.H = np.eye(3)
        self.kf.P = np.array([[25.0, 0.0, 0.0], [0.0, 25.0, 0.0], [0.0, 0.0, 25.0]])
        self.kf.R = np.array([[2.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 2.0]])
        self.kf.Q = w_noise(dim=3, dt=0.05, var=1.0)
        self.last_predict_time = time.time()
        self.get_logger().info('filternode is running...')

    def listener_callback(self, state_msg):
        dt = time.time() - self.last_predict_time
        u = np.array([[state_msg.data[3]],
                      [state_msg.data[4]],
                      [state_msg.data[5]]])
        B = np.array([[dt, 0.0, 0.0],
                      [0.0, dt, 0.0],
                      [0.0, 0.0, dt]])
        self.kf.predict(u, B)
        self.last_predict_time = time.time()
        
        z = np.array([[state_msg.data[0]],
                      [state_msg.data[1]],
                      [state_msg.data[2]]])
        self.kf.update(z)
        self.publish()
        # self.get_logger().info('received_state:: "%f %f %f %f %f %f' %(
        #     state_msg.data[0], state_msg.data[1], state_msg.data[2],
        #     state_msg.data[3], state_msg.data[4], state_msg.data[5]))

    def publish(self):
        odom_msg = Float32MultiArray()
        odom_msg.data = [float(self.kf.x[0]), float(self.kf.x[1]), float(self.kf.x[2])]
        self.publisher_.publish(odom_msg)
        self.get_logger().info('filtered_data:: "%f %f %f"' %(self.kf.x[0], self.kf.x[1], self.kf.x[2]))

def main(args=None):
    rclpy.init(args=args)
    filter = FilterNode()
    rclpy.spin(filter)
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()