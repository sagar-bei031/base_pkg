import time
import numpy as np
import math
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.common import Q_discrete_white_noise as w_noise
from math import sin, cos, tan2, pi
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

CPR = 4000
xR = 0.260
yrR = 0.255
ylR = 0.223
Wheel_Diameter = 0.0574


class UKFNode(Node):
    ''' Reasonable Choices for the Parameters
        β=2 is a good choice for Gaussian problems, 
        κ=3-n where n is the dimension of x is a good choice for κ, 
        and 0≤α≤1 is an appropriate choice for α, 
        where a larger value for α spreads the sigma points further from the mean.
    '''

    def __init__(self):
        super().__init__('ukf_node')
        self.subscription = self.create_subscription(Float32MultiArray,
                                                     '/raw_encoder_count',
                                                     self.odom_callback,
                                                     10)
        self.publisher_ = self.create_publisher(Float32MultiArray,
                                                '/filtered_odometry',
                                                10)
        dt = 0.001
        sigmas = MerweScaledSigmaPoints(n=6, alpha=0.1, beta=2, kappa=-3,
                                        subtract=self.residual_x)
        self.ukf = UKF(dim_x=3, dim_z=4, fx=self.f_cv, hx=self.h_cv, points=sigmas,
                       x_mean_fn=self.state_mean, z_mean_fn=self.z_mean,
                       residual_x=self.residual_x, residual_z=self.residual_h)

        self.last_received_time = time.time()
        self.get_logger().info('ukf_node is running...')

    def move(x, u, dt):
        new_x = x[0] + u[0] * dt
        new_y = x[1] + u[1] * dt
        new_theta = x[2] + u[2] * dt
        if (new_theta > pi):
            new_theta -= 2*pi
        elif (new_theta <= (-pi)):
            new_theta += 2*pi

        return np.array([[new_x], [new_y], [new_theta]])

    def measure(x, u, dt):
        backX_dist = pi * Wheel_Diameter * x[0] / CPR
        rightY_dist = pi * Wheel_Diameter * x[1] / CPR
        leftY_dist = pi * Wheel_Diameter * x[2] / CPR

    def f_cv(x, dt):
        F = np.array([[1.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

    def generate_Q(self, dt):
        q = w_noise(dim=2, dt=dt, var=0.01)
        Q = np.zeros(6, 6)
        Q[0, 0] = Q[1, 1] = Q[2, 2] = q[0, 0]
        Q[3, 3] = Q[4, 4] = Q[5, 5] = q[1, 1]
        Q[0, 3] = Q[1, 4] = Q[2, 5] = q[0, 1]
        Q[3, 0] = Q[4, 1] = Q[5, 2] = q[1, 0]
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
        x, P = self.ukf.predict(u, self.generate_B(
            dt), None, self.ukf.generate_Q(dt))
        self.last_predict_time = time.time()
        # self.get_logger().info('predicted_data:: "%f %f %f"' %(self.ukf.x[0]*100, self.ukf.x[1]*100, self.ukf.x[2]*180/math.pi))
        z = np.array([[state_msg.data[0]],
                      [state_msg.data[1]],
                      [state_msg.data[2]]])
        x, P = self.ukf.update(z)

        odom_msg = Float32MultiArray()
        odom_msg.data = [float(x[0]), float(x[1]), float(x[2])]
        self.publisher_.publish(odom_msg)
        # self.get_logger().info('filtered_data:: "%f %f %f"' %(self.ukf.x[0]*100, self.ukf.x[1]*100, self.ukf.x[2]*180/math.pi))


def main(args=None):
    rclpy.init(args=args)
    filter = UKFNode()
    rclpy.spin(filter)
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
