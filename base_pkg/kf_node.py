import time
import numpy as np
import math
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
                                                     self.odom_callback,
                                                     10)
        self.publisher_ = self.create_publisher(Float32MultiArray,
                                                '/filtered_odometry',
                                                10)
        dt = 0.05
        self.kf = KalmanFilter(dim_x=6, dim_z=6, dim_u=3)
        self.kf.x = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])
        self.kf.F = np.array([[1.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                              [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                              [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
        self.kf.B = self.generate_B(dt)
        self.kf.P *= 10.0
        self.kf.Q = self.generate_Q(dt)
        self.kf.H = np.eye(6)
        self.kf.R * 1.0
        self.last_predict_time = time.time()
        # self.y = 0
        self.get_logger().info('filternode is running...')


    def generate_B(self, dt):
        B = np.array([[dt,  0.0, 0.0, 0.0, 0.0, 0.0],
                      [0.0, dt,  0.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, dt,  0.0, 0.0, 0.0],
                      [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]])
        return B

    def generate_Q(self, dt):
        q = w_noise(dim=2, dt=dt, var=0.01)
        Q = np.zeros(6,6)
        Q[0,0] = Q[1,1] = Q[2,2] = q[0,0]
        Q[3,3] = Q[4,4] = Q[5,5] = q[1,1]
        Q[0,3] = Q[1,4] = Q[2,5] = q[0,1]
        Q[3,0] = Q[4,1] = Q[5,2] = q[1,0]
        return Q

    def odom_callback_callback(self, state_msg):
        dt = time.time() - self.last_predict_time
        # print('dt::', dt)
        u = np.array([[state_msg.data[3]],
                      [state_msg.data[4]],
                      [state_msg.data[5]]])
        # dy = state_msg.data[4]*dt
        # self.y += dy
        # print(self.y*100, dy*100)
        x, P = self.kf.predict(u, self.generate_B(dt), None, self.kf.generate_Q(dt))
        self.last_predict_time = time.time()
        # self.get_logger().info('predicted_data:: "%f %f %f"' %(self.kf.x[0]*100, self.kf.x[1]*100, self.kf.x[2]*180/math.pi))
        z = np.array([[state_msg.data[0]],
                      [state_msg.data[1]],
                      [state_msg.data[2]]])
        x, P = self.kf.update(z)

        odom_msg = Float32MultiArray()
        odom_msg.data = [float(x[0]), float(x[1]), float(x[2])]
        self.publisher_.publish(odom_msg)
        # self.get_logger().info('filtered_data:: "%f %f %f"' %(self.kf.x[0]*100, self.kf.x[1]*100, self.kf.x[2]*180/math.pi))

def main(args=None):
    rclpy.init(args=args)
    filter = FilterNode()
    rclpy.spin(filter)
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
