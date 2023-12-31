import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class PidConfigListener(Node):
    def __init__(self):
        super().__init__('pid_config_listener')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/pid_config',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info('PID Config Listener is running...')

    def listener_callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")

def main():
    rclpy.init()
    listener_node = PidConfigListener()
    rclpy.spin(listener_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
