import socket
import struct
import time
import math
import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


HOST = '10.42.0.1'
PORT = 8082

class ESPNode(Node):
    def __init__(self):
        super().__init__('esp_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/raw_robot_state', 10)
        self.subscription = self.create_subscription(Float32MultiArray, '/cmd_robot_vel', self.listener_callback, 10)
        self.get_logger().info('ESP node is running')
        self.last_sent_time = time.time()

        # Establish the socket connection here
        self.comm_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.comm_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.comm_socket.bind((HOST, PORT))
        self.comm_socket.listen(1)

        # Set up a separate thread for socket communication
        self.socket_thread = threading.Thread(target=self.communicate)
        self.socket_thread.daemon = True  # Set as a daemon thread to terminate with the main program

    def start_socket_communication(self):
        self.socket_thread.start()

    def communicate(self):
        try:
            while True:  # Continuously listen for socket data
                conn, addr = self.comm_socket.accept()
                with conn:
                    try:
                        state_received_data = conn.recv(24)
                        if len(state_received_data) >= 24:
                            state_msg = Float32MultiArray()
                            state_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                            state_msg.data[0], state_msg.data[1], state_msg.data[2], state_msg.data[3], state_msg.data[4], state_msg.data[5] = struct.unpack('ffffff', state_received_data)
                            self.publisher_.publish(state_msg)
                            self.get_logger().info('raw_state:: "%f %f %f %f %f %f"' % (
                                state_msg.data[0] * 100, state_msg.data[1] * 100, state_msg.data[2] * 180 / math.pi,
                                state_msg.data[3] * 100, state_msg.data[4] * 100, state_msg.data[5] * 180 / math.pi))

                    except socket.error as e:
                        self.get_logger().info(f"Socket error: {e}")
                        break

        except Exception as e:
            self.get_logger().info(f"Exception occurred: {e}")
        finally:
            if self.comm_socket:
                self.comm_socket.shutdown(socket.SHUT_RDWR)
                self.comm_socket.close()

    def listener_callback(self, msg):
        try:
            conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            conn.connect((HOST, PORT))
            twist_sending_data = struct.pack('fff', msg.data[0], msg.data[1], msg.data[2])
            conn.sendall(twist_sending_data)
            conn.close()
            self.last_sent_time = time.time()
        except socket.error as e:
            self.get_logger().info(f"Socket error: {e}")

def main(args=None):
    rclpy.init(args=args)
    esp = ESPNode()
    esp.start_socket_communication()  # Start socket communication in a separate thread
    rclpy.spin(esp)                   # Spin the node
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()