import socket
import struct
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


HOST = '10.42.0.1'
PORT = 8082

class ESPNode(Node):
    def __init__(self):
        super().__init__('esp_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'raw_robot_state', 10)
        self.get_logger().info('ESP node is running')
        self.last_sent_time = time.time()

    def communicate(self):
        comm_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        comm_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        try:
            comm_socket.bind((HOST, PORT))
            self.get_logger().info("Waiting for connection...")
            comm_socket.listen(1)
            self.get_logger().info(f"Listening to {comm_socket}")

            while rclpy.ok():
                conn, addr = comm_socket.accept()
                with conn:
                    try:
                        state_received_data = conn.recv(24)

                        if len(state_received_data) >= 24:
                            state_msg = Float32MultiArray()
                            state_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                            state_msg.data[0], state_msg.data[1], state_msg.data[2], state_msg.data[3], state_msg.data[4], state_msg.data[5] = struct.unpack('ffffff', state_received_data)
                            self.publisher_.publish(state_msg)
                            self.get_logger().info('raw_state:: "%f %f %f %f %f %f"' %(state_msg.data[0], state_msg.data[1], state_msg.data[2], state_msg.data[3], state_msg.data[4], state_msg.data[5]))

                        if ((time.time() - self.last_sent_time) > 0.05):
                            twist_sending_data = struct.pack('fff', 0.0, 0.0, 0.5) 
                            conn.sendall(twist_sending_data)
                            self.get_logger().info('sent')
                            self.last_sent_time = time.time()

                    except socket.error as e:
                        self.get_logger().info(f"Socket error: {e}")
                        break

        except Exception as e:
            self.get_logger().info(f"Exception occurred: {e}")
        except KeyboardInterrupt:
            comm_socket.close()

        finally:
            if comm_socket:
                comm_socket.shutdown(socket.SHUT_RDWR)
                comm_socket.close()

def main(args=None):
    rclpy.init(args=args)
    esp = ESPNode()
    esp.communicate()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
